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

⚠⚠ **죽일 대상을 넓게 잡지 않는다 — 두 번 당했다.**
  2026-10-17: `component_container` 로 잡아 다른 컨테이너까지 죽였다.
  2026-11-05: 그래서 `__node:=camera_container` 로 좁혔는데, 나중에 들어온
    **측면 305 두 대가 같은 컨테이너 이름**을 써서 또 같이 죽었다. 2L 을
    되살릴 때마다 측면이 죽고 USB 가 재열거됐고, 그게 반복되다 좌측 305 가
    통째로 사라져 **전원 재투입으로야 살아났다.**
  → 지금은 **네임스페이스까지 포함해 토큰 단위로** 비교한다.

⚠ **유예·냉각·횟수 제한이 있다.** 카메라는 기동에 30초쯤 걸리고, 되살리기가
  실패를 반복하면 그건 소프트웨어로 풀 문제가 아니다 (케이블·전원). 몇 번 해 보고
  안 되면 **멈추고 크게 알린다** — 무한 재시작 루프가 더 나쁘다.
"""

import glob
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
        # 공백으로 여러 개를 줄 수 있다 — **어느 하나라도** 끊기면 발동한다.
        #   측면 305 는 좌·우가 한 런치(`orbbec_side.launch.py`)로 같이 뜨니
        #   쌍으로 다뤄야 한다. 한쪽만 죽이고 그 런치를 다시 돌리면
        #   **다른 쪽이 두 개**가 된다.
        self.declare_parameter('topic', '/camera/color/image_raw')
        # ⚠⚠ 이 토큰이 cmdline 에 **있으면 대상에서 뺀다.** `match` 만으로는
        #   좌·우를 함께 고를 수 없다 — 둘의 구분 토큰이 서로 다르고
        #   (`__ns:=/camera_left` vs `/camera_right`) 공통 토큰은 2L 에도
        #   있는 `__node:=camera_container` 뿐이다. 그래서 '컨테이너 전부
        #   중 2L 만 제외' 로 고른다. **이게 없으면 측면을 되살리다가 2L 을
        #   죽이고, 그 재열거 반복이 USB PHY 를 래치시킨다** (어제 좌측
        #   305 를 그렇게 잃었다).
        self.declare_parameter('exclude', '')
        # 인스턴스가 둘 이상이면 상태 토픽이 겹친다
        self.declare_parameter('status_topic', '/camera/watchdog')
        # 기동 유예 — 카메라는 뜨는 데 30초쯤 걸린다. 그 전에 손대면 오히려 방해다
        self.declare_parameter('grace_sec', 75.0)
        # 이만큼 프레임이 없으면 멈춘 것으로 본다. 정상은 10fps 다
        self.declare_parameter('stale_sec', 20.0)
        # 되살린 뒤 다시 판정하기까지 기다리는 시간 (기동 시간보다 넉넉히)
        self.declare_parameter('cooldown_sec', 120.0)
        # 이 횟수를 넘기면 포기하고 알리기만 한다 (물리적 문제로 본다)
        self.declare_parameter('max_restarts', 3)
        # 컨테이너를 고를 cmdline 조각. **좁게** 잡는다
        # ⚠ **토큰 전부가 cmdline 에 있어야** 대상이다 (부분문자열 아님).
        #   네임스페이스를 반드시 포함할 것 — 측면 305 도 컨테이너 이름이 같다.
        self.declare_parameter('match', '__node:=camera_container __ns:=/camera')
        # 되살릴 명령. 런치가 카메라 인자와 **같은 것**을 넣어 준다
        self.declare_parameter('relaunch', '')
        # ⚠⚠ [2026-10-17] **되살리기 전에 USB 권한부터 고친다.**
        #   udev 가 카메라의 빠른 연속 재열거를 못 따라가고 이벤트를 흘리면,
        #   노드가 devtmpfs 생짜 기본값 `0600 root:root` 로 남는다. 기본 규칙
        #   (0664)조차 적용이 안 된 상태다. 서비스는 koceti 로 돌고 SDK 는
        #   usbfs 로 장치를 여므로 **"장치가 없다" 로 보인다.**
        #   벤더 규칙 자체는 멀쩡하다 — `udevadm test` 에서 MODE 0666 ·
        #   GROUP 44(video) 로 정상 매칭된다. **흘린 이벤트를 다시 돌려주면 된다.**
        #   권한을 안 고치고 컨테이너만 되살리면 똑같이 실패한다 (실측 1회차).
        #   root 가 필요하므로 이 명령 하나만 sudoers 에 열어 둔다
        #   (`tools/sudoers/rebar-udev-retrigger`). 비어 있거나 권한이 없으면
        #   **조용히 건너뛰고** 되살리기만 한다 — 워치독이 죽으면 안 된다.
        self.declare_parameter('pre_cmd', '')
        # ⚠⚠ [2026-11-06] **USB 트리에 장치가 있는지 먼저 본다.**
        #   오늘 측면 305 가 버스에서 통째로 사라졌는데, 워치독이 그걸 모르고
        #   컨테이너를 3번 재시작했다. 없는 장치는 못 살린다 — 그러면서
        #   **재열거만 반복해 상황을 악화시킨다** (반복 재열거가 USB PHY 를
        #   래치시킨다; 자동절전과 겹치면 더 빨리 빠진다).
        #   여기에 시리얼을 주면, 하나라도 없을 때 **재시작을 건너뛰고**
        #   크게 알린다. 그 상태는 전원 재투입으로만 풀린다.
        #   빈 값이면 검사하지 않는다 (기존 동작).
        self.declare_parameter('usb_serials', '')
        self.declare_parameter('enabled', True)

        g = self.get_parameter
        self.topics = [t for t in str(g('topic').value).split() if t]
        self.topic = ' '.join(self.topics)      # 로그용
        self.exclude = [t for t in str(g('exclude').value).split() if t]
        self.grace = float(g('grace_sec').value)
        self.stale = float(g('stale_sec').value)
        self.cooldown = float(g('cooldown_sec').value)
        self.max_restarts = int(g('max_restarts').value)
        self.match = str(g('match').value)
        self.relaunch = str(g('relaunch').value)
        self.pre_cmd = str(g('pre_cmd').value)
        self.usb_serials = [x for x in str(g('usb_serials').value).split() if x]
        self.enabled = bool(g('enabled').value)

        self.seen = {t: None for t in self.topics}   # 토픽별 마지막 프레임
        self.t0 = time.time()
        self.acted_at = 0.0
        self.restarts = 0
        self.child = None
        self._gave_up = False

        for t in self.topics:
            self.create_subscription(
                Image, t, lambda m, k=t: self._on_frame(k), qos_profile_sensor_data)
        self.status = self.create_publisher(
            String, str(g('status_topic').value), 10)
        self.create_timer(2.0, self.tick)
        self.get_logger().info(
            f"카메라 워치독 — {self.topic} 가 {self.stale:.0f}초 끊기면 "
            f"컨테이너를 되살린다 (유예 {self.grace:.0f}s · 냉각 {self.cooldown:.0f}s · "
            f"최대 {self.max_restarts}회)"
            + ('' if self.enabled else '  ⚠ enabled=false — 알리기만 한다'))

    def _on_frame(self, key):
        if self.seen.get(key) is None:
            self.get_logger().info(f'{key} 프레임 확인 — 감시 시작')
        self.seen[key] = time.time()

    @property
    def last_frame(self):
        """감시 중 **가장 오래 끊긴** 토픽의 시각. 하나라도 죽으면 발동한다."""
        if any(v is None for v in self.seen.values()):
            return None
        return min(self.seen.values())

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
            # ⚠⚠ [2026-11-05] **토큰 단위로 본다. 부분문자열이면 안 된다.**
            #   `__node:=camera_container` 로 부분 비교했더니 **측면 305 두 대를
            #   같이 죽였다.** 셋 다 컨테이너 이름이 같고 네임스페이스만 다르다:
            #       __node:=camera_container -r __ns:=/camera        ← 2L (대상)
            #       __node:=camera_container -r __ns:=/camera_left   ← 측면
            #       __node:=camera_container -r __ns:=/camera_right  ← 측면
            #   게다가 `/camera` 는 `/camera_left` 의 **접두어**라 네임스페이스를
            #   더해도 부분 비교로는 못 가른다. 토큰으로 비교해야 한다.
            #   실제 피해: 2L 을 되살릴 때마다 측면이 죽고 USB 가 재열거되는데,
            #   그게 반복되면 PHY 가 래치돼 카메라가 통째로 사라진다
            #   ([[rebar-usb-error71-needs-power-cycle]]). 2026-11-05 에 좌측
            #   305 를 그렇게 잃고 전원 재투입으로야 살렸다.
            toks = cmd.split()
            if (all(m in toks for m in self.match.split())
                    and not any(x in toks for x in self.exclude)
                    and 'camera_watchdog' not in cmd):
                out.append((int(pid), cmd.strip()))
        return out

    def _usb_missing(self):
        """`usb_serials` 중 USB 트리에 **없는** 것. 빈 목록이면 검사 안 함."""
        if not self.usb_serials:
            return []
        have = set()
        for f in glob.glob('/sys/bus/usb/devices/*/serial'):
            try:
                with open(f) as fh:
                    have.add(fh.read().strip())
            except OSError:
                pass
        return [s for s in self.usb_serials if s not in have]

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
        if self.pre_cmd:
            self.get_logger().info(f"   권한 복구: {self.pre_cmd}")
            try:
                r = subprocess.run(shlex.split(self.pre_cmd), timeout=15,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                if r.returncode != 0:
                    self.get_logger().warning(
                        f"   권한 복구 실패(rc={r.returncode}) — 그대로 진행한다. "
                        f"sudoers 설정을 확인하라: "
                        f"{(r.stdout or b'').decode(errors='ignore')[:120]}")
                else:
                    time.sleep(1.0)          # udev 가 적용할 시간
            except Exception as e:
                self.get_logger().warning(f"   권한 복구 건너뜀: {e}")
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
            missing = self._usb_missing()
            if missing:
                # 버스에 없는 장치는 재시작으로 못 살린다. 시도 자체가 해롭다.
                state = '장치 없음'
                self.get_logger().error(
                    f"⛔⛔ {', '.join(missing)} 가 **USB 트리에 없다** — "
                    f"컨테이너를 되살려도 소용없어 재시작을 건너뛴다. "
                    f"**하드웨어 전원 재투입**이 필요하다 "
                    f"(케이블·허브도 함께 보라)",
                    throttle_duration_sec=30.0)
            elif self.enabled:
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
