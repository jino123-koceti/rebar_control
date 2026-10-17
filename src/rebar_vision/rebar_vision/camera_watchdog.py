#!/usr/bin/env python3
"""카메라가 **살아서 멈췄을 때** 되살린다.

## 무엇을 고치는가

Orbbec 노드는 죽지 않는다 — 살아서 `Device with serial number ... not found` 를
5.5초마다 영원히 찍는다. 그래서 `respawn` 으로는 안 잡힌다 (크래시가 아니다).

원인은 **경쟁 상태**다. 서비스를 재시작하면 이전 카메라 프로세스가 장치를 놓으면서
USB 가 재열거되는데, 새 노드가 **그 재열거 중에 뜨면** SDK 가 그때의 장치 목록을
들고 못 빠져나온다. 실측(2026-10-17):

    13:07:4x  서비스 재시작
    13:07:49  usb 1-4.3.2 disconnect → device 36
    13:07:52  또 disconnect → device 37     ← 이 사이에 노드가 떴다
    그 뒤      "not found" 무한 반복. 5분 지나도 안 붙었다
    서비스를 다시 재시작하니 "Matched device serial number" 로 붙었다

재시작 3번 중 1번 걸렸다. **현장에서 USB 를 뽑을 수는 없으니** 자동으로 풀어야 한다.

## 어떻게 고치는가

컬러 프레임을 보고 있다가 끊기면 **카메라 컨테이너만 죽이고 다시 띄운다.**
상류 `gemini2L.launch.py` 는 `respawn` 을 받지 않고 단독 노드 경로는 Foxy 전용이라
(Humble 은 항상 컴포저블 컨테이너), 이 노드가 직접 해 주는 수밖에 없다.

⚠ **죽일 대상을 이름으로 넓게 잡지 않는다.** `component_container` 만 보고 죽이면
  다른 컨테이너까지 잡는다. `__node:=camera_container` 가 들어간 cmdline 만 고른다.
  (2026-10-17 에 `pkill -f` 로 엉뚱한 노드를 잡은 일이 있다.)

⚠ **유예·냉각·횟수 제한이 있다.** 카메라는 기동에 30초쯤 걸리고, 되살리기가
  실패를 반복하면 그건 소프트웨어로 풀 문제가 아니다 (케이블·전원). 몇 번 해 보고
  안 되면 **멈추고 크게 알린다** — 무한 재시작 루프가 더 나쁘다.
"""

import os
import shlex
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraWatchdog(Node):
    def __init__(self):
        super().__init__('camera_watchdog')
        self.declare_parameter('topic', '/camera/color/image_raw')
        # 기동 유예 — 카메라는 뜨는 데 30초쯤 걸린다. 그 전에 손대면 오히려 방해다
        self.declare_parameter('grace_sec', 75.0)
        # 이만큼 프레임이 없으면 멈춘 것으로 본다. 정상은 10fps 다
        self.declare_parameter('stale_sec', 20.0)
        # 되살린 뒤 다시 판정하기까지 기다리는 시간 (기동 시간보다 넉넉히)
        self.declare_parameter('cooldown_sec', 120.0)
        # 이 횟수를 넘기면 포기하고 알리기만 한다 (물리적 문제로 본다)
        self.declare_parameter('max_restarts', 3)
        # 컨테이너를 고를 cmdline 조각. **좁게** 잡는다
        self.declare_parameter('match', '__node:=camera_container')
        # 되살릴 명령. 런치가 카메라 인자와 **같은 것**을 넣어 준다
        self.declare_parameter('relaunch', '')
        self.declare_parameter('enabled', True)

        g = self.get_parameter
        self.topic = str(g('topic').value)
        self.grace = float(g('grace_sec').value)
        self.stale = float(g('stale_sec').value)
        self.cooldown = float(g('cooldown_sec').value)
        self.max_restarts = int(g('max_restarts').value)
        self.match = str(g('match').value)
        self.relaunch = str(g('relaunch').value)
        self.enabled = bool(g('enabled').value)

        self.last_frame = None          # 마지막 프레임 시각
        self.t0 = time.time()
        self.acted_at = 0.0
        self.restarts = 0
        self.child = None
        self._gave_up = False

        self.create_subscription(Image, self.topic, self._on_frame,
                                 qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/camera/watchdog', 10)
        self.create_timer(2.0, self.tick)
        self.get_logger().info(
            f"카메라 워치독 — {self.topic} 가 {self.stale:.0f}초 끊기면 "
            f"컨테이너를 되살린다 (유예 {self.grace:.0f}s · 냉각 {self.cooldown:.0f}s · "
            f"최대 {self.max_restarts}회)"
            + ('' if self.enabled else '  ⚠ enabled=false — 알리기만 한다'))

    def _on_frame(self, _msg):
        if self.last_frame is None:
            self.get_logger().info('카메라 프레임 확인 — 감시 시작')
        self.last_frame = time.time()

    # ---- 복구 ---------------------------------------------------------------
    def _targets(self):
        """`match` 가 들어간 프로세스들. 자기 자신과 셸은 제외한다."""
        out = []
        me = os.getpid()
        for pid in os.listdir('/proc'):
            if not pid.isdigit() or int(pid) == me:
                continue
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmd = f.read().replace(b'\0', b' ').decode(errors='ignore')
            except OSError:
                continue
            if self.match in cmd and 'camera_watchdog' not in cmd:
                out.append((int(pid), cmd.strip()))
        return out

    def _restart(self, why):
        self.restarts += 1
        self.acted_at = time.time()
        found = self._targets()
        self.get_logger().error(
            f"⛔ 카메라가 멈췄다 ({why}) — 되살린다 "
            f"[{self.restarts}/{self.max_restarts}]")
        for pid, cmd in found:
            self.get_logger().warning(f"   죽인다: PID {pid}  {cmd[:110]}")
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError as e:
                self.get_logger().warning(f"   SIGTERM 실패: {e}")
        if not found:
            self.get_logger().warning(
                f"   '{self.match}' 에 맞는 프로세스가 없다 — 죽일 것 없이 띄운다")
        time.sleep(3.0)                      # 장치가 풀릴 시간을 준다
        for pid, _ in found:                 # 아직 살아 있으면 확실히 끝낸다
            try:
                os.kill(pid, 0)
                os.kill(pid, signal.SIGKILL)
                self.get_logger().warning(f"   SIGKILL: PID {pid}")
            except OSError:
                pass
        if not self.relaunch:
            self.get_logger().error(
                "   `relaunch` 가 비어 있다 — 다시 띄우지 못한다 (런치 설정 확인)")
            return
        self.get_logger().info(f"   다시 띄운다: {self.relaunch}")
        try:
            # 부모가 죽어도 남도록 세션을 분리한다
            self.child = subprocess.Popen(
                shlex.split(self.relaunch), start_new_session=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:              # 되살리기 실패가 워치독을 죽이면 안 된다
            self.get_logger().error(f"   띄우기 실패: {e}")

    # ---- 주기 ---------------------------------------------------------------
    def tick(self):
        now = time.time()
        age = None if self.last_frame is None else now - self.last_frame
        since_start = now - self.t0
        state = 'ok'
        if self.last_frame is None and since_start < self.grace:
            state = '기동 대기'
        elif age is not None and age < self.stale:
            state = 'ok'
            self.restarts = 0 if age < 2.0 else self.restarts
        elif self._gave_up:
            state = '포기'
        elif since_start < self.grace:
            state = '기동 대기'
        elif now - self.acted_at < self.cooldown:
            state = '냉각'
        elif self.restarts >= self.max_restarts:
            self._gave_up = True
            state = '포기'
            self.get_logger().error(
                f"⛔⛔ 카메라를 {self.max_restarts}번 되살렸는데도 안 붙는다 — "
                f"**소프트웨어로는 더 할 게 없다.** 케이블·허브·전원을 보라 "
                f"(tools/camera_recover.sh 의 3단계)")
        else:
            why = ('프레임이 한 번도 없다' if age is None
                   else f'{age:.0f}초 동안 프레임 없음')
            if self.enabled:
                self._restart(why)
                state = '되살리는 중'
            else:
                state = '멈춤(조치 안 함)'
                self.get_logger().error(f"⛔ 카메라가 멈췄다 ({why}) — enabled=false")
        self.status.publish(String(data=(
            f'{{"state": "{state}", "age_sec": '
            f'{"null" if age is None else round(age, 1)}, '
            f'"restarts": {self.restarts}}}')))


def main():
    rclpy.init()
    n = CameraWatchdog()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
