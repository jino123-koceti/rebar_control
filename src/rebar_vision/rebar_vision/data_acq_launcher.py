#!/usr/bin/env python3
"""UI 버튼 → 현장 학습데이터 수집(field_collect.py) 실행기.

## 프로토콜
```
/mission/command  (std_msgs/String, JSON)
  {"command": "DATA_ACQ_START"}   → 수집 시작
  {"command": "DATA_ACQ_STOP"}    → 수집 정지 (요약 저장)
```
상태는 `/data_acq/status`(JSON):
```
{"running": true, "session": "20260908_181500", "saved": 137,
 "dir": "/home/koceti/ros2_ws/data/field/20260908_181500", "sec": 92}
```

## 왜 프로세스를 띄우나 (노드 안에서 직접 안 찍고)
`field_collect.py`는 이미 현장에서 쓰던 도구다 — 유사프레임 제거, 디스크 보호,
주행맥락(방향·deck_edge·odom) 메타까지 검증돼 있다. 노드로 옮겨 적으면 그걸
다시 만들고 다시 검증해야 한다. 그대로 띄우고 **정지는 SIGINT**로 보내
도구가 스스로 summary.json 을 쓰고 나가게 한다.

⚠ 자식은 `setsid`로 띄워 프로세스 그룹째 정리한다.

## 샘플링
천천히 수동 주행하며 모으므로 **1Hz는 너무 잦다** — 같은 장면만 쌓인다.
기본 0.5Hz + 유사프레임 임계 12. 현장에서 `ros2 param set` 으로 바로 바꿀 수
있다(다음 START부터 적용):
```
ros2 param set /data_acq_launcher hz 0.3
ros2 param set /data_acq_launcher diff 16     # 클수록 더 엄격(적게 저장)
```

⚠ 이 노드는 **구독만** 하는 도구를 띄운다. 주행·결속에 영향이 없어
   자율결속과 동시에 켜도 된다.
"""
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

TOOL = '/home/koceti/ros2_ws/tools/collect/field_collect.py'
OUT_ROOT = '/home/koceti/ros2_ws/data/field'
LOG_DIR = '/home/koceti/ros2_ws/data/logs/data_acq'


class DataAcqLauncher(Node):
    def __init__(self):
        super().__init__('data_acq_launcher')

        # 전방/후방 ZED X + 작업영역 Gemini2L. 툴캠·좌측은 기본에서 뺐다.
        self.declare_parameter('cams', 'front,back,work')
        self.declare_parameter('hz', 0.5)          # 카메라당 최대 저장률
        self.declare_parameter('diff', 12.0)       # 유사프레임 임계(클수록 엄격)
        self.declare_parameter('depth', True)      # Orbbec depth 동시 저장
        self.declare_parameter('quality', 90)
        self.declare_parameter('min_free_gb', 20.0)
        self.declare_parameter('tag', 'ui')
        self.declare_parameter('extra_args', '')

        self.proc = None
        self.session = None
        self.root = None
        self.t0 = 0.0
        self.logf = None

        self.create_subscription(String, '/mission/command', self._cmd, 10)
        self.status_pub = self.create_publisher(String, '/data_acq/status', 10)
        self.create_timer(2.0, self._tick)

        self.get_logger().warn(
            '📸 데이터 수집 실행기 준비 — /mission/command 에 '
            '{"command":"DATA_ACQ_START"} / {"command":"DATA_ACQ_STOP"}')

    # ────────────────────────────── 명령
    def _cmd(self, msg):
        d = msg.data
        if not d.startswith('{'):
            if d.strip() == 'DATA_ACQ_START':
                self._start()
            elif d.strip() in ('DATA_ACQ_STOP', 'E-STOP'):
                self._stop(f'문자열 {d.strip()}')
            return
        try:
            cmd = str(json.loads(d).get('command', ''))
        except json.JSONDecodeError:
            return
        if cmd == 'DATA_ACQ_START':
            self._start()
        elif cmd == 'DATA_ACQ_STOP':
            self._stop('DATA_ACQ_STOP 수신')
        # ⚠ E-STOP/CANCEL 로는 멈추지 않는다 — 수집은 주행을 방해하지 않으므로
        #   비상정지 뒤 "그 장면"이 오히려 남아 있어야 쓸모가 있다.

    # ────────────────────────────── 실행 / 정지
    def _alive(self):
        return self.proc is not None and self.proc.poll() is None

    def _start(self):
        if self._alive():
            self.get_logger().warn(
                f'  이미 수집 중이다 (세션 {self.session}, '
                f'{self._saved()}장) — 무시')
            self._publish()
            return
        if not os.path.exists(TOOL):
            self.get_logger().error(f'❌ 수집 도구 없음: {TOOL}')
            return

        tag = str(self.gp('tag'))
        self.session = time.strftime('%Y%m%d_%H%M%S') + (f'_{tag}' if tag else '')
        self.root = os.path.join(OUT_ROOT, self.session)

        args = ['python3', TOOL,
                # ⚠ 폴더명을 **직접 지정**한다. 도구가 스스로 시각을 찍게 두면
                #   여기서 계산한 이름과 1초 어긋나 status의 dir/saved가 틀린다.
                '--session', self.session,
                '--cams', str(self.gp('cams')),
                '--hz', str(float(self.gp('hz'))),
                '--diff', str(float(self.gp('diff'))),
                '--quality', str(int(self.gp('quality'))),
                '--min-free-gb', str(float(self.gp('min_free_gb')))]
        if bool(self.gp('depth')):
            args += ['--depth']
        args += str(self.gp('extra_args')).split()

        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f'{self.session}.log')
        try:
            self.logf = open(path, 'w')
            self.proc = subprocess.Popen(
                args, stdout=self.logf, stderr=subprocess.STDOUT,
                start_new_session=True, cwd='/home/koceti/ros2_ws')
        except Exception as e:
            self.get_logger().error(f'❌ 수집 실행 실패: {e}')
            self.proc = None
            self._publish()
            return

        self.t0 = time.time()
        self.get_logger().warn(
            f'📸 데이터 수집 시작 (PID {self.proc.pid})\n'
            f'   카메라 {self.gp("cams")}  {self.gp("hz")}Hz  '
            f'유사임계 {self.gp("diff")}  '
            f'depth {"ON" if self.gp("depth") else "OFF"}\n'
            f'   저장: {self.root}\n'
            f'   로그: {path}')
        self._publish()

    def _stop(self, why):
        if not self._alive():
            self.get_logger().info(f'  수집 중이 아님 ({why})')
            self._cleanup()
            self._publish()
            return
        pid = self.proc.pid
        n = self._saved()
        # ⚠ SIGINT 여야 한다 — 도구가 KeyboardInterrupt 를 받아 summary.json 을
        #   쓰고 정상 종료한다. SIGKILL 이면 요약이 없다.
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        for _ in range(50):                       # 요약 쓸 시간을 준다 (최대 5초)
            if self.proc.poll() is not None:
                break
            time.sleep(0.1)
        if self.proc.poll() is None:
            self.get_logger().error('  SIGINT 무응답 → SIGKILL (요약 없음)')
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.proc.wait(timeout=3)
        self.get_logger().warn(
            f'⏹ 데이터 수집 정지: {why}\n'
            f'   {n}장,  {(time.time()-self.t0)/60:.1f}분\n'
            f'   저장 위치: {self.root}')
        self._cleanup()
        self._publish()

    def _cleanup(self):
        if self.logf:
            try:
                self.logf.close()
            except Exception:
                pass
            self.logf = None
        self.proc = None

    # ────────────────────────────── 상태
    def _saved(self):
        """meta.jsonl 줄 수 = 저장한 프레임 수 (도구 로그 파싱보다 정확하다)."""
        if not self.root:
            return 0
        p = os.path.join(self.root, 'meta.jsonl')
        try:
            with open(p, 'rb') as f:
                return sum(1 for _ in f)
        except OSError:
            return 0

    def _tick(self):
        if self.proc is not None and self.proc.poll() is not None:
            # 도구가 스스로 멈춘 경우(디스크 부족 등)도 알려야 한다
            self.get_logger().warn(
                f'■ 수집 종료 (exit {self.proc.returncode}) — '
                f'{self._saved()}장, {self.root}')
            self._cleanup()
        self._publish()

    def _publish(self):
        if not rclpy.ok():
            return
        run = self._alive()
        m = String()
        m.data = json.dumps({
            'running': run,
            'session': self.session if run else None,
            'dir': self.root if run else None,
            'saved': self._saved() if run else 0,
            'sec': round(time.time() - self.t0, 1) if run else 0,
        }, ensure_ascii=False)
        try:
            self.status_pub.publish(m)
        except Exception:
            pass

    def gp(self, name):
        return self.get_parameter(name).value

    def destroy_node(self):
        if self._alive():
            self._stop('실행기 종료')
        super().destroy_node()


def main():
    rclpy.init()
    n = DataAcqLauncher()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
