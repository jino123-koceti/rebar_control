#!/usr/bin/env python3
"""UI 버튼 → 비전 자율결속(rebar_drive) 실행기.

## 왜 별도 노드인가
`rebar_drive`는 매번 파라미터 20개를 손으로 붙여 `ros2 run` 하던 노드다.
UI에서 누르게 하려면 누군가 그 프로세스를 대신 띄워줘야 하는데,
**navigator에 넣으면 안 된다** — [A]경로주행과 [B]비전주행은 둘 다 `/cmd_vel`에
쏘고 중재가 없어서(rebar_drive_node.py 머리말) 섞이면 덜컥거린다.
그래서 navigator를 건드리지 않고 **명령어 이름을 분리**했다.

## 프로토콜
```
/mission/command  (std_msgs/String, JSON)
  {"command": "AUTO_TYING", "direction": "FWD"}   → 전진 자율결속 시작
  {"command": "AUTO_TYING", "direction": "REV"}   → 후진 자율결속 시작
  {"command": "AUTO_TYING_STOP"}                  → 중지
  {"command": "CANCEL"} / "E-STOP"                → 중지 (기존 명령에 편승)
```
⚠ `START_MISSION`은 **쓰지 않는다.** navigator가 같은 이름을 받아 웨이포인트
   주행을 시작해버린다(navigator.py `START_MISSION` → `sm.start()`).

상태는 `/auto_tying/status`(JSON)로 되돌려준다:
```
{"running": true, "direction": "FWD", "pid": 12345, "state": "FWD_STEP"}
```

## 실행중 재요청 처리
`rebar_drive`는 **DONE이 되어도 프로세스가 살아 있다.** 그래서 "이미 떠 있으면
거절"만 하면 한 번 쓰고 다시는 못 누른다. `/rebar_drive/state`를 같이 보고

  · 주행중(FWD_*/REV_*)          → 거절 (달리는 중에 갈아타면 위험)
  · DONE / ABORT / IDLE / 죽음   → 정리하고 새로 띄움

⚠ 자식은 `setsid`로 띄워 **프로세스 그룹째** 죽인다. `ros2 run`은 래퍼와 실제
   노드가 별개 프로세스라 부모만 죽이면 노드가 남아 `/cmd_vel`을 계속 쏜다.
"""
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

DRIVING = ('FWD_DETECT', 'FWD_STEP', 'FWD_SETTLE',
           'REV_DETECT', 'REV_STEP', 'REV_SETTLE', 'LATERAL')
LOG_DIR = '/home/koceti/ros2_ws/data/logs/auto_tying'


class AutoTyingLauncher(Node):
    def __init__(self):
        super().__init__('auto_tying_launcher')

        # ── rebar_drive에 넘길 파라미터 (yaml/CLI로 바꿔가며 쓰라고 다 노출) ──
        self.declare_parameter('pkg', 'rebar_vision')
        self.declare_parameter('exe', 'rebar_drive')
        self.declare_parameter('arm', True)
        self.declare_parameter('one_way', True)
        self.declare_parameter('lateral_enabled', False)
        # 횡이동 방향 고정 — rebar_drive 의 같은 이름 파라미터로 넘긴다. ''=자동.
        #   ⚠ 2026-09-22 기본값 'right': 좌측 모노 카메라 불량(GMSL 링크 없음).
        #     좌측 끝단에서 출발해 우측으로만 'ㄹ'자 커버. 좌측 복구되면 '' 로 되돌릴 것.
        #   ★ 2026-09-22 저녁 좌측 복구(전원 차단 후 재인가로 GMSL 링크 회복, 판정 0.942 정상) → '' 복귀.
        #   ★★ 같은 날 밤: GMSL **4대 동시 가동이 원인**으로 판명(4대면 6~26분 안에 무너짐) →
        #     좌측 모노를 서비스에서 뺐다(3대 구성). 좌측 판정이 없으므로 다시 'right'.
        self.declare_parameter('lateral_fixed_dir', 'right')
        self.declare_parameter('do_tying', True)
        self.declare_parameter(
            'orbbec_model',
            '/home/koceti/ros2_ws/src/rebar_vision/model/orbbec_crossing.pt')
        self.declare_parameter('tie_every_n', 2)
        # heading **켜고 끄기**는 실행기의 몫이다(모드 선택).
        self.declare_parameter('heading_enabled', True)
        # ★ 튜닝값은 여기서 기본값을 갖지 않는다 (2026-09-15).
        #   왜: 여기에 숫자를 박아두면 rebar_drive_node의 declare_parameter 기본값을
        #   **항상** 덮어쓴다. 실제로 그렇게 돼 있었다 —
        #       kp 0.015→0.08(5.3배) · deadband 0.6→0.2 · max 0.08→0.18
        #       offset_front -1.13→0.0 · offset_back +1.52→0.0   ← 실측 영점이 소멸
        #   영점을 아무리 재캘리브해도 자율모드에선 0으로 덮여 무효였다.
        #   → NaN = "지정 안 함". 그때는 인자를 아예 안 넘겨 rebar_drive 기본값이 산다.
        #     바꾸고 싶으면 이 파라미터를 명시로 주면 그때만 넘어간다.
        for _k in ('heading_offset_front_deg', 'heading_offset_back_deg',
                   'heading_rev_gain', 'heading_kp', 'heading_max',
                   'heading_deadband_deg'):
            self.declare_parameter(_k, float('nan'))
        # ★ 2026-09-09 UI 실행은 0.30 → 0.15 (절반). SLOW·접근 감속은 이 값에
        #   곱해지는 배율이라, 여기만 줄이면 전 구간이 비례해서 느려진다.
        #   스텝 주행시간이 2배가 되지만 실측 14s → 28s 라 max_step_sec(120s)에
        #   여유가 넉넉하다.
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('max_step_sec', 120.0)
        self.declare_parameter('approach_mm', 40.0)
        self.declare_parameter('tie_timeout_sec', 180.0)
        # ★ 2026-09-22 상부 결속부 속도 50 → 70%. Z 과부하 임계는 속도에 비례해 자동으로
        #   커진다(2650→3250mA, tying_orchestrator.yaml). 단 감속구간 임계
        #   z_torque_decel_mA(1200)는 고정값이라 오탐이 나면 그쪽을 볼 것.
        self.declare_parameter('tie_speed_pct', 70)
        # 위에 없는 파라미터를 급히 넣어야 할 때. "-p a:=1 -p b:=2" 형태 그대로.
        self.declare_parameter('extra_args', '')

        self.proc = None
        self.direction = None
        self.drive_state = 'IDLE'
        self.state_t = 0.0
        self.logf = None

        self.create_subscription(String, '/mission/command', self._cmd, 10)
        self.create_subscription(String, '/rebar_drive/state', self._state, 10)
        self.status_pub = self.create_publisher(String, '/auto_tying/status', 10)
        self.create_timer(1.0, self._tick)

        self.get_logger().warn(
            '🎛  자율결속 실행기 준비 — /mission/command 에 '
            '{"command":"AUTO_TYING","direction":"FWD"|"REV"} 를 보내면 시작한다')

    # ────────────────────────────── 명령 수신
    def _state(self, msg):
        self.drive_state = msg.data
        self.state_t = time.time()

    def _cmd(self, msg):
        d = msg.data
        if not d.startswith('{'):
            # 문자열 명령도 받아준다 (UI가 JSON을 안 쓰는 경로 대비)
            if d.strip() in ('E-STOP', 'CANCEL', 'AUTO_TYING_STOP'):
                self._stop(f'문자열 명령 {d.strip()}')
            return
        try:
            data = json.loads(d)
        except json.JSONDecodeError:
            return
        cmd = str(data.get('command', ''))

        if cmd in ('AUTO_TYING_STOP', 'CANCEL', 'E-STOP', 'ESTOP'):
            self._stop(f'{cmd} 수신')
            return
        if cmd != 'AUTO_TYING':
            return

        raw = str(data.get('direction', '')).upper()
        # RWD/REV/BWD 다 받아준다 — UI 표기가 흔들려도 동작해야 한다
        if raw in ('FWD', 'FORWARD', 'F'):
            start = 'fwd'
        elif raw in ('REV', 'RWD', 'BWD', 'BACKWARD', 'R'):
            start = 'rev'
        else:
            self.get_logger().error(
                f'❌ AUTO_TYING: direction "{raw}" 을 모르겠다 (FWD 또는 REV)')
            self._publish_status()
            return
        self._start(start, raw)

    # ────────────────────────────── 실행 / 중지
    def _alive(self):
        return self.proc is not None and self.proc.poll() is None

    def _start(self, start, shown):
        if self._alive():
            # 살아 있어도 '달리는 중'이 아니면 갈아타도 된다.
            fresh = (time.time() - self.state_t) < 5.0
            if fresh and self.drive_state in DRIVING:
                self.get_logger().error(
                    f'❌ 이미 자율결속 주행 중이다 (상태 {self.drive_state}). '
                    f'멈춘 뒤에 다시 누를 것')
                self._publish_status()
                return
            self._stop(f'이전 실행 정리 (상태 {self.drive_state})', quiet=True)

        args = ['ros2', 'run', self.gp('pkg'), self.gp('exe'), '--ros-args']
        for k in ('arm', 'one_way', 'lateral_enabled', 'lateral_fixed_dir', 'do_tying', 'orbbec_model',
                  'tie_every_n', 'heading_enabled', 'heading_offset_front_deg',
                  'heading_offset_back_deg', 'heading_rev_gain', 'heading_kp',
                  'heading_max', 'heading_deadband_deg', 'speed', 'max_step_sec',
                  'approach_mm', 'tie_timeout_sec', 'tie_speed_pct'):
            v = self.gp(k)
            # NaN = 지정 안 함 → 넘기지 않는다 (rebar_drive 기본값을 살린다).
            if isinstance(v, float) and math.isnan(v):
                continue
            if isinstance(v, str) and v == '':
                continue
            args += ['-p', f'{k}:={self._as_arg(v)}']
        args += ['-p', f'start:={start}']
        extra = str(self.gp('extra_args')).split()
        args += extra

        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(
            LOG_DIR, f'{time.strftime("%Y%m%d_%H%M%S")}_{start}.log')
        try:
            self.logf = open(path, 'w')
            # ⚠ start_new_session: 프로세스 그룹째 죽이기 위해 필수
            self.proc = subprocess.Popen(
                args, stdout=self.logf, stderr=subprocess.STDOUT,
                start_new_session=True)
        except Exception as e:
            self.get_logger().error(f'❌ 실행 실패: {e}')
            self.proc = None
            self._publish_status()
            return

        self.direction = start
        self.drive_state = 'IDLE'
        self.state_t = time.time()
        self.get_logger().warn(
            f'▶️ 자율결속 시작: {shown} → start:={start}  (PID {self.proc.pid})\n'
            f'   로그: {path}\n'
            f'   ⚠ 리모콘 S20+S23 을 넣어야 실제로 움직인다')
        self._publish_status()

    def _stop(self, why, quiet=False):
        if not self._alive():
            if not quiet:
                self.get_logger().info(f'  자율결속 실행중 아님 ({why})')
            self._cleanup()
            return
        pid = self.proc.pid
        self.get_logger().warn(f'⏹ 자율결속 중지: {why} (PID {pid})')
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)   # 노드가 정리하고 나가게
        except ProcessLookupError:
            pass
        for _ in range(30):                              # 최대 3초 기다린다
            if self.proc.poll() is not None:
                break
            time.sleep(0.1)
        if self.proc.poll() is None:
            self.get_logger().error('  SIGINT 무응답 → SIGKILL')
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.proc.wait(timeout=3)
        self._cleanup()
        self._publish_status()

    def _cleanup(self):
        if self.logf:
            try:
                self.logf.close()
            except Exception:
                pass
            self.logf = None
        self.proc = None
        self.direction = None

    # ────────────────────────────── 주기 처리
    def _tick(self):
        if self.proc is not None and self.proc.poll() is not None:
            rc = self.proc.returncode
            self.get_logger().warn(f'■ 자율결속 프로세스 종료 (exit {rc})')
            self._cleanup()
        self._publish_status()

    def _publish_status(self):
        # ⚠ 종료 경로(destroy_node → _stop → 여기)에서는 rclpy 컨텍스트가 이미
        #   내려가 있을 수 있다. 그대로 두면 종료 때마다 RCLError 역추적이 찍힌다.
        if not rclpy.ok():
            return
        m = String()
        m.data = json.dumps({
            'running': self._alive(),
            'direction': {'fwd': 'FWD', 'rev': 'REV'}.get(self.direction),
            'pid': self.proc.pid if self._alive() else None,
            'state': self.drive_state if self._alive() else None,
        })
        try:
            self.status_pub.publish(m)
        except Exception:
            pass                     # 종료 중 발행 실패는 무해하다

    # ────────────────────────────── 잡동사니
    def gp(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _as_arg(v):
        if isinstance(v, bool):
            return 'true' if v else 'false'
        return str(v)

    def destroy_node(self):
        # 실행기가 죽는데 자식이 살아서 /cmd_vel 을 쏘면 안 된다
        self._stop('실행기 종료', quiet=True)
        super().destroy_node()


def main():
    rclpy.init()
    n = AutoTyingLauncher()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass                       # SIGINT/SIGTERM 정상 종료 — 역추적 찍지 않는다
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
