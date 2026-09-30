#!/usr/bin/env python3
"""L3 — 상부 스테이지 원점복귀.

리미트 센서를 보고 축을 원점까지 보낸 뒤 레퍼런스를 기록한다.
**CAN 을 모른다.** 리미트는 `/limit_sensors/*`(L1 ezi_io_node)로 받고,
축은 `/joint_N/speed` 로 명령한다 (YEAR3_ARCHITECTURE.md §2 계층 규칙).

──────────────────────────────────────────────────────────────────────────
2차년도(`rebar_base_control/homing_controller.py`, 1,106줄)와 다른 점

  · 그 노드는 CAN ID·엔코더 상수·Yaw 자세복귀(L2R)까지 한 파일에 있었다.
    3차년도는 축 정의를 `config/axes.yaml` 에 두고, 이 노드는 순서만 관리한다.
  · **축을 하나씩 따로 호밍할 수 있다.** 3차년도는 호밍을 한 번도 돌려본 적이
    없으므로, 전체 시퀀스를 처음부터 돌리는 것은 위험하다. Z 단독 → X → Y → Yaw
    순서로 검증하고 마지막에 전체를 돌린다.
  · Yaw 자세복귀(L2R)는 아직 옮기지 않았다. 3차년도 Yaw 엔코더 기준값이 미측정이다.

## 명령 (`/homing_cmd`, String)

    "z" / "x" / "y" / "yaw"   해당 축만 원점으로
    "all"                     Z → X → Y → Yaw 순서로 전체
    "stop"                    중단 (전 축 정지)

## 상태 (`/homing_status`, String JSON)

    {"state": "...", "axis": "...", "detail": "...", "refs": {...}}

## 안전

  · 리미트 토픽이 `limit_stale_sec` 이상 갱신되지 않으면 시작하지 않고, 진행 중이면
    중단한다. 센서를 못 보는 상태로 축을 미는 것이 가장 위험하다.
  · 목표 리미트가 이미 눌려 있으면 그 축은 곧바로 BACK_OFF 로 간다.
  · 반대쪽 리미트가 눌리면 즉시 중단한다 (방향이 반대라는 뜻이다).
  · 축별 타임아웃. 넘으면 정지하고 실패로 끝낸다.
  · 비상정지(`/safety/state` 또는 `/remote_control`)는 아직 연결하지 않았다 — S2 에서
    safety_node 가 생기면 그쪽을 구독한다. 그때까지는 `stop` 명령과 타임아웃만 있다.

⚠ 브레이크: 이 노드는 브레이크를 풀지 않는다. X/Y 는 `0x77` 로 미리 풀어야 하고,
  Z 는 리프팅축이라 자중 낙하 위험이 있어 기구 상태를 보고 사람이 판단한다
  (`axes.yaml` 의 `never_auto_release`).
"""

import json
import os
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from .axis_config import (load_axis_motor_ids, load_home_offsets,
                          load_precheck, load_seek_dirs, precheck_violation)
from .axis_config import HOMING_AXES as AXES
from std_msgs.msg import Bool, Float32, Float64, Int32, String


class Phase(Enum):
    IDLE = 'idle'
    ARM = 'arm'            # 브레이크 해제가 모터에 먹을 때까지 기다린다
    SEEK = 'seek'          # 원점 리미트를 향해 이동
    BACK_OFF = 'back_off'  # 리미트에서 살짝 빠짐 (센서 해제)
    FINE = 'fine'          # 느린 속도로 재접근 → 레퍼런스 기록
    OFFSET = 'offset'      # 에지에서 작업 위치까지 더 간다 (yaw 12시)
    DONE = 'done'
    FAILED = 'failed'


# 축 정의 — **모터 ID 는 config/axes.yaml 에서 읽는다.** 코드에 0x14X 를 쓰지 않는다
# (tools/check 의 R1). 2차년도는 ID 가 10개 파일에 흩어져 있어 3차년도로 넘어올 때
# 주석과 실제가 어긋났다.
#   joint: 명령 토픽 번호,  home_limit: 원점 리미트,  far_limit: 반대쪽(안전 확인용)
#   dir: 원점 방향 부호 (실측으로 확정해야 한다 — 기본값은 미검증)
LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')
# 순서는 **기구 간섭 때문에** 이 순서여야 한다 (2026-09-30 실장비 확인):
#   z   : 먼저 올려둔다(z_min=위). 내려와 있으면 아래 동작들이 간섭한다
#   x   : x_min 은 yaw 자세와 무관하게 언제나 가능. 집어넣으면 yaw 회전 여유가 생긴다
#   yaw : 12시로. **y 보다 먼저여야 한다**
#   y   : y_min/y_max 는 **yaw 가 12시일 때만** 도달 가능. 아니면 상부 프레임을 친다
# ⚠ 기존 순서 (z,x,y,yaw) 는 y 가 yaw 보다 앞이라, 12시가 아닌 상태로 y 가 리미트까지
#   달려 프레임을 친다. 절대 되돌리지 말 것.
# 2026-09-30 확정 (사용자 지정):
#   z   : 먼저 올린다. z_max 쪽에 있으면 X 이동 중 철근 배근에 걸린다
#   yaw : 12시로. **사용자가 호밍 전 12시 ±3° 에 놓는다는 전제** 위에서 동작한다
#   x, y: 12시에서는 x_min·y_min 둘 다 간섭이 없다 (사용자 확인)
# ⚠ 동시 이동(x+y)은 아직 미구현이다 — 상태기계가 축 하나만 다룬다. 순차로도
#   순서 자체는 같으므로 안전성은 동일하고, 시간만 더 걸린다.
ALL_ORDER = ('z', 'yaw', 'x', 'y')


class HomingNode(Node):
    def __init__(self):
        super().__init__('homing_node')

        self.declare_parameter('seek_speed_dps', 30.0)
        # ⚠ 8.0 은 Z 에서 너무 느리다. 중력을 들어올리는 축이라 저속에서 힘이 부쳐
        #   1.5초에 명령의 1/3(5°)밖에 못 움직였고, 스톨 판정에 걸려 호밍이 실패했다.
        #   15 dps 에서는 1.5초에 19.3° 로 정상이었다 (2026-09-30 실측).
        self.declare_parameter('fine_speed_dps', 15.0)
        # 후퇴는 **시간이 아니라 센서가 풀릴 때까지** 한다. 검출편 폭이 축마다 다르기
        # 때문이다 (yaw 실측 건 1.75°). 2026-09-30 첫 실장비 호밍에서 Z 가 0.7초×8dps
        # = 출력축 0.45° 만 빼고 "센서가 안 풀림" 으로 실패했다.
        self.declare_parameter('back_off_sec', 0.3)        # 풀린 뒤 **추가로** 더 뺄 시간
        self.declare_parameter('back_off_max_sec', 8.0)    # 이 시간까지 안 풀리면 실패
        self.declare_parameter('back_off_speed_dps', 15.0)
        self.declare_parameter('axis_timeout_sec', 40.0)
        self.declare_parameter('limit_stale_sec', 1.0)
        # 브레이크 해제 후 명령까지의 대기. **이게 없으면 축이 안 움직인다.**
        # 2026-09-30 실장비: `/brake_cmd` 와 `/joint_N/speed` 는 다른 토픽이라 도착
        # 순서가 보장되지 않는다. 실제로 속도 명령이 해제보다 56ms 먼저 도착했고,
        # 그 뒤 yaw 가 브레이크를 문 채 5.4A 를 끌며 거의 안 움직였다. 해제를 먼저
        # 확실히 보내고 기다리면 3.2A 로 떨어지고 명령의 88% 가 나온다.
        self.declare_parameter('arm_sec', 1.0)
        # yaw 스톨 시 방향 반전을 쓸지. **현재 위치를 알고 있으면 꺼야 한다.**
        # 2026-09-30: 반전이 걸려 1번 자세(구동범위 하한)를 지나쳤다. 홈이 어느 쪽인지
        # 아는 상황에서는 반전이 도움이 아니라 위험이다.
        self.declare_parameter('yaw_sweep', True)
        # 이탈(breakaway): 출발 직후 이 시간 안에 안 움직이면 더 센 속도로 민다
        self.declare_parameter('breakaway_sec', 0.5)
        # 권한 요청 후 이 시간까지 못 받으면 실패한다 (중재기가 있는 경우에만 적용)
        self.declare_parameter('grant_timeout', 3.0)
        # 스톨 감지 — 명령을 보내는데 엔코더가 안 변하면 기계 끝에 닿은 것이다.
        # yaw 는 끝 리미트가 아예 없어서 이것이 **유일한 보호 수단**이다.
        self.declare_parameter('stall_sec', 2.0)
        self.declare_parameter('stall_deg', 0.8)      # 모터축 도
        # 축별 원점 방향 부호. **실측 전이므로 기본값을 믿지 말 것.**
        for name, cfg in AXES.items():
            self.declare_parameter(f'{name}_dir', cfg['dir'])

        g = self.get_parameter
        self.seek_speed = float(g('seek_speed_dps').value)
        self.fine_speed = float(g('fine_speed_dps').value)
        self.back_off_sec = float(g('back_off_sec').value)
        self.back_off_max = float(g('back_off_max_sec').value)
        self.back_off_speed = float(g('back_off_speed_dps').value)
        self.axis_timeout = float(g('axis_timeout_sec').value)
        self.limit_stale = float(g('limit_stale_sec').value)
        self.arm_sec = float(g('arm_sec').value)
        self.breakaway_sec = float(g('breakaway_sec').value)
        self.grant_timeout = float(g('grant_timeout').value)
        for name in AXES:
            AXES[name]['dir'] = int(g(f'{name}_dir').value)
        if not bool(g('yaw_sweep').value):
            AXES['yaw']['sweep'] = False

        self.speed_pubs = {n: self.create_publisher(Float32, f"/joint_{c['joint']}/speed", 10)
                           for n, c in AXES.items()}
        self.status_pub = self.create_publisher(String, '/homing_status', 10)
        # 브레이크는 L1(position_control_node) 이 건다. 여기서는 요청만 한다.
        # ⚠ Z 는 브레이크를 풀면 **즉시 떨어진다** (2026-09-30 실측 출력축 -12.74°).
        #   그래서 해제는 그 축을 움직이기 직전에만 하고, 끝나면 곧바로 잠근다.
        self.brake_pub = self.create_publisher(String, '/brake_cmd', 10)
        # 제어 권한. 호밍 중에는 리모콘이 같은 축 토픽에 쓰면 안 된다.
        # 2026-09-30: 중재가 없어서 리모콘의 "정지(0)" 와 호밍의 30dps 가 1:1 로
        # 번갈아 들어가 축이 툭툭 끊겼고 스톨로 실패했다 (mode_arbiter.py 참고).
        # ⚠ 중재기가 없으면 예전처럼 그냥 진행한다 — 이 의존이 호밍을 막으면 안 된다.
        self.mode_pub = self.create_publisher(String, '/control_mode_request', 10)
        self.create_subscription(String, '/control_mode', self._on_mode, 10)
        self._mode = None            # None = 중재기 없음 (아직 한 번도 못 받음)
        self._want_control = False
        self._request_t = 0.0

        self.limit = {n: None for n in LIMITS}
        self.limit_time = {n: 0.0 for n in LIMITS}
        self.pos = {}
        for n in LIMITS:
            self.create_subscription(Bool, f'/limit_sensors/{n}',
                                     lambda m, k=n: self._on_limit(k, m), 10)
        for name, d in load_seek_dirs().items():
            if name in AXES:
                AXES[name]['dir'] = d          # axes.yaml 이 코드 기본값을 덮는다
        self.precheck = load_precheck()
        self.single = {}                   # 축 → 단회전 절대값
        self.offsets = load_home_offsets()
        if self.offsets:
            self.get_logger().info(
                "원점 후 오프셋 이동: "
                + ", ".join(f"{k} {v:+.2f}°" for k, v in self.offsets.items()))
        motor_ids = load_axis_motor_ids()
        if not motor_ids:
            self.get_logger().warning(
                "axes.yaml 에서 축 CAN ID 를 읽지 못했습니다 — 위치 레퍼런스는 기록되지 않습니다")
        for name, mid in motor_ids.items():
            self.create_subscription(
                Int32, f"/motor_{mid}/encoder_single",
                (lambda k: (lambda m: self.single.__setitem__(k, m.data)))(name), 10)
        for name, mid in motor_ids.items():
            # ⚠ 타입은 **Float32** 다. position_control_node 가 Float32 로 발행한다.
            # Float64 로 구독하면 타입 불일치로 **한 건도 오지 않는다** (2026-09-30 실측:
            # `ros2 topic info` 가 Float32/Float64 두 타입을 같이 보여준다). 그 상태로는
            # 원점 레퍼런스가 영영 비어 있게 된다.
            self.create_subscription(Float32, f"/motor_{mid}_position",
                                     lambda m, k=name: self.pos.__setitem__(k, m.data), 10)
        self.create_subscription(String, '/homing_cmd', self._on_cmd, 10)

        self.phase = Phase.IDLE
        self.axis = None
        self.queue = []
        self.t_phase = 0.0
        self.refs = {}
        self.detail = ''
        self.stall_sec = float(g('stall_sec').value)
        self.stall_deg = float(g('stall_deg').value)
        self._stall_pos = None        # 마지막으로 "움직였다" 고 본 위치
        self._stall_t = 0.0
        self._reversed = False        # sweep 축이 이미 방향을 뒤집었는가
        self._off_since = None        # 후퇴 중 센서가 풀린 시각
        self._phase_pos0 = None       # 단계 시작 시 위치 (이탈 판정용)

        self.timer = self.create_timer(0.05, self.tick)      # 20Hz
        # 상태를 주기적으로도 알린다 (전이 때만 내면 관측자가 현재 상태를 못 본다)
        self.status_timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            "호밍 노드 시작 — /homing_cmd 로 'z'|'x'|'y'|'yaw'|'all'|'stop'")
        self.get_logger().info(
            f"순서 {' → '.join(ALL_ORDER)} / 방향 "
            + ', '.join(f"{k}:{v['dir']:+d}" for k, v in AXES.items()))
        self.get_logger().warning(
            "⚠ 브레이크는 이 노드가 축마다 풀고 잠급니다. Z 는 풀면 떨어지므로 "
            "끝나는 즉시 잠급니다. 호밍 중 리모콘을 건드리지 마세요(같은 토픽 충돌).")

    # ---- 입력 --------------------------------------------------------------
    def _on_limit(self, name, msg):
        self.limit[name] = msg.data
        self.limit_time[name] = time.time()

    def _on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'stop':
            self._stop_all('정지 명령')
            return
        if self.phase not in (Phase.IDLE, Phase.DONE, Phase.FAILED):
            self.get_logger().warning(f"호밍 진행 중({self.axis}) — '{cmd}' 무시")
            return
        bad = self._precheck_failed()
        if bad:
            self.get_logger().error(f"호밍 거부 — {bad}")
            self.detail = bad
            self._enter(Phase.FAILED, bad)
            return
        self._want_control = True
        self._request_t = time.time()
        self._request_control('homing')
        if cmd == 'all':
            self.queue = list(ALL_ORDER)
        elif cmd in AXES:
            self.queue = [cmd]
        else:
            self.get_logger().error(f"알 수 없는 명령: '{cmd}'")
            return
        if not self._limits_fresh():
            self._fail("리미트 토픽이 갱신되지 않습니다 — ezi_io_node 확인")
            return
        self._next_axis()

    # ---- 진행 --------------------------------------------------------------
    def _limits_fresh(self):
        now = time.time()
        stale = [n for n in LIMITS
                 if self.limit[n] is None or now - self.limit_time[n] > self.limit_stale]
        if stale:
            self.detail = f"갱신 안 된 리미트: {', '.join(stale)}"
            return False
        return True

    def _next_axis(self):
        if not self.queue:
            self._lock_all()
            self._release_control()
            self._enter(Phase.DONE, '완료')
            self.get_logger().info(f"호밍 완료 — 레퍼런스 {self.refs}")
            return
        prev = self.axis
        self.axis = self.queue.pop(0)
        if prev is not None and prev != self.axis:
            self._brake('lock', prev)           # 끝난 축은 바로 잠근다
        self._brake('release', self.axis)       # 움직일 축만 푼다
        self._reversed = False
        self._stall_pos = None
        # 해제가 먹을 때까지 기다렸다가 움직인다 (arm_sec 주석 참고)
        self._enter(Phase.ARM, '브레이크 해제 대기')

    def _enter(self, phase, detail=''):
        self.phase = phase
        self.detail = detail
        self.t_phase = time.time()
        self._stall_pos = None          # 단계가 바뀌면 스톨 기준을 다시 잡는다
        self._off_since = None
        self._phase_pos0 = None
        self._publish_status()
        if phase in (Phase.DONE, Phase.FAILED, Phase.IDLE):
            self.axis = None

    def _precheck_failed(self):
        return precheck_violation(self.precheck, self.single)

    def _on_mode(self, msg):
        try:
            self._mode = (json.loads(msg.data) or {}).get('mode', 'manual')
        except ValueError:
            pass

    def _request_control(self, what):
        """권한 요청/반납. 중재기가 없으면 아무도 안 듣지만 해가 없다."""
        self.mode_pub.publish(String(data=f"homing_node {what}"))

    def _granted(self):
        """지금 축을 움직여도 되는가.

        중재기가 없으면(`_mode is None`) 예전처럼 진행한다. 있으면 'homing' 일 때만.
        """
        return self._mode is None or self._mode == 'homing'

    def _brake(self, action, axis):
        """축 브레이크 요청. Z 는 자동해제 금지 축이라 force 를 붙여야 풀린다."""
        arg = f"{action} {axis}" + (' force' if action == 'release' and axis == 'z' else '')
        self.brake_pub.publish(String(data=arg))

    def _release_control(self):
        self._want_control = False
        self._request_control('release')

    def _lock_all(self):
        """전 축 잠금. **Z 를 먼저** 잠근다 — 떨어지는 축이 우선이다."""
        for a in ('z',) + tuple(x for x in AXES if x != 'z'):
            self._brake('lock', a)

    def _fine_of(self, axis):
        """축별 정밀 속도. 없으면 공통값."""
        return float(AXES[axis].get('fine', self.fine_speed))

    def _with_breakaway(self, axis, base, elapsed):
        """아직 안 움직였으면 더 센 속도로 민다 (정지마찰 이탈).

        속도 제어기가 명령 속도에 비례해서만 전류를 올리는 탓에, 낮은 속도로는
        정지 상태를 못 벗어나는 축이 있다 (yaw 실측: 30dps→2.2A 로 안 움직이고
        80dps→3.7A 에서 풀린다). 일단 움직이면 원래 속도로 돌아간다.
        """
        bk = AXES[axis].get('breakaway')
        if not bk or elapsed > self.breakaway_sec * 6:
            return base
        p = self.pos.get(axis)
        if p is None or self._phase_pos0 is None:
            return base
        if abs(p - self._phase_pos0) > self.stall_deg:
            return base                      # 이미 움직이고 있다
        if elapsed < self.breakaway_sec:
            return base                      # 잠깐은 정상 속도로 시도
        return bk if base >= 0 else -bk

    def _backoff_of(self, axis):
        return float(AXES[axis].get('back_off', self.back_off_speed))

    def _seek_of(self, axis):
        return float(AXES[axis].get('seek', self.seek_speed))

    def _cmd_speed(self, axis, dps):
        self.speed_pubs[axis].publish(Float32(data=float(dps)))

    def _stop_axis(self, axis):
        self._cmd_speed(axis, 0.0)

    def _stop_all(self, reason):
        for a in AXES:
            self._cmd_speed(a, 0.0)
        self._lock_all()                        # 속도 0 만으로는 Z 가 떨어진다
        self._release_control()
        self.queue = []
        self.get_logger().warning(f"호밍 중단 — {reason}")
        self._enter(Phase.IDLE, reason)

    def _fail(self, reason):
        for a in AXES:
            self._cmd_speed(a, 0.0)
        self._lock_all()
        self._release_control()
        self.queue = []
        self.get_logger().error(f"호밍 실패 — {reason}")
        self._enter(Phase.FAILED, reason)

    def tick(self):
        if self.phase in (Phase.IDLE, Phase.DONE, Phase.FAILED):
            return
        if not self._limits_fresh():
            self._fail(self.detail)
            return
        # ── 제어 권한 ────────────────────────────────────────────────────────
        if not self._granted():
            if time.time() - self._request_t > self.grant_timeout:
                self._fail(f"제어 권한을 못 받았다 (현재 모드 '{self._mode}') — "
                           "다른 노드가 잡고 있거나 안전 상태가 아니다")
                return
            self._request_control('homing')      # 아직 대기 중이면 계속 요청한다
            for a in AXES:
                self._cmd_speed(a, 0.0)
            return
        if time.time() - self._request_t > 1.0:
            self._request_t = time.time()
            self._request_control('homing')      # 하트비트 — 끊기면 중재기가 회수한다

        cfg = AXES[self.axis]
        elapsed = time.time() - self.t_phase

        if self.phase is Phase.ARM:
            # 대기 중에도 해제를 여러 번 보낸다 (첫 발행은 연결 직후라 유실될 수 있다)
            self._brake('release', self.axis)
            if elapsed >= self.arm_sec:
                if self.limit[cfg['home_limit']]:
                    self.get_logger().info(
                        f"{self.axis}: 이미 {cfg['home_limit']} 에 있음 → 후퇴부터")
                    self._enter(Phase.BACK_OFF, '이미 원점')
                else:
                    self._enter(Phase.SEEK, f"{cfg['home_limit']} 탐색")
            return

        if elapsed > self.axis_timeout:
            self._fail(f"{self.axis} {self.phase.value} 타임아웃 {self.axis_timeout:.0f}s")
            return
        # 반대쪽 리미트가 눌리면 방향이 반대다 — 즉시 멈춘다
        if cfg['far_limit'] and self.limit[cfg['far_limit']]:
            self._fail(f"{self.axis}: 반대쪽 리미트({cfg['far_limit']})에 닿음 — 방향 부호 확인")
            return

        at_home = bool(self.limit[cfg['home_limit']])

        # ── 스톨 감지 ────────────────────────────────────────────────────────
        # 명령을 보내는데 엔코더가 안 변하면 기계 끝에 닿은 것이다. yaw 는 끝 리미트가
        # 없어서 이것이 유일한 보호다. 다른 축은 far_limit 이 잡아주지만, 센서가
        # 고장나면 여기서 걸린다.
        if self._phase_pos0 is None:
            self._phase_pos0 = self.pos.get(self.axis)

        if self.phase in (Phase.SEEK, Phase.FINE, Phase.OFFSET) and not at_home:
            p = self.pos.get(self.axis)
            if p is None:
                pass                      # 위치 토픽이 없으면 스톨 판정을 못 한다
            elif self._stall_pos is None or abs(p - self._stall_pos) > self.stall_deg:
                self._stall_pos = p
                self._stall_t = time.time()
            elif time.time() - self._stall_t > self.stall_sec:
                if cfg.get('sweep') and not self._reversed and self.phase is Phase.SEEK:
                    # yaw: 반대쪽 끝이었다. 방향을 뒤집어 다시 쓴다
                    cfg['dir'] = -cfg['dir']
                    self._reversed = True
                    self._stall_pos = None
                    self.t_phase = time.time()          # 타임아웃도 다시 센다
                    self.get_logger().warning(
                        f"{self.axis}: 기계 끝에 닿음(스톨) — 방향을 {cfg['dir']:+d} 로 뒤집어 재탐색")
                elif self.phase is Phase.FINE and cfg.get('fine_stall_ok'):
                    # 홈 쪽으로 갈수록 부하가 커지는 축이 있다 (yaw: 12시 근처).
                    # 정밀 재접근에서 못 밀면 **이미 홈 직전**이라는 뜻이므로,
                    # SEEK 에서 잡은 에지를 원점으로 인정하고 넘어간다.
                    self._stop_axis(self.axis)
                    p = self.pos.get(self.axis)
                    self.refs[self.axis] = p
                    self.get_logger().warning(
                        f"{self.axis}: 정밀 재접근이 밀리지 않는다 — SEEK 에지를 원점으로 "
                        f"인정한다" + (f" (위치 {p:.2f}°)" if p is not None else ""))
                    if self.offsets.get(self.axis):
                        self._enter(Phase.OFFSET,
                                    f"작업 위치까지 {self.offsets[self.axis]:.2f}° 더")
                    else:
                        self._next_axis()
                    return
                else:
                    self._stop_axis(self.axis)
                    self._fail(f"{self.axis}: 스톨 — {self.stall_sec:.1f}초 동안 "
                               f"{self.stall_deg:.1f}° 도 못 움직였다 "
                               f"(기계 끝·브레이크 미해제·과부하 확인)")
                    return

        if self.phase is Phase.SEEK:
            if at_home:
                self._stop_axis(self.axis)
                self.get_logger().info(f"{self.axis}: {cfg['home_limit']} 도달 → 후퇴")
                self._enter(Phase.BACK_OFF, '리미트 도달')
            else:
                self._cmd_speed(self.axis, cfg['dir'] * self._seek_of(self.axis))

        elif self.phase is Phase.BACK_OFF:
            # 센서가 풀릴 때까지 뺀다. 풀린 뒤 back_off_sec 만큼 더 빼서 에지에서
            # 충분히 떨어뜨린 다음 정밀 재접근한다.
            if at_home:
                if elapsed > self.back_off_max:
                    self._stop_axis(self.axis)
                    self._fail(f"{self.axis}: {self.back_off_max:.0f}초를 뺐는데도 "
                               f"{cfg['home_limit']} 가 안 풀림 — 센서 위치·감도 확인")
                    return
                self._off_since = None
                self._cmd_speed(self.axis, self._with_breakaway(
                    self.axis, -cfg['dir'] * self._backoff_of(self.axis), elapsed))
            else:
                if self._off_since is None:
                    self._off_since = time.time()
                    self.get_logger().info(
                        f"{self.axis}: {cfg['home_limit']} 해제됨 ({elapsed:.1f}초 걸림)")
                if time.time() - self._off_since < self.back_off_sec:
                    self._cmd_speed(self.axis, self._with_breakaway(
                    self.axis, -cfg['dir'] * self._backoff_of(self.axis), elapsed))
                else:
                    self._stop_axis(self.axis)
                    self.get_logger().info(f"{self.axis}: 정밀 재접근")
                    self._enter(Phase.FINE, '정밀 재접근')

        elif self.phase is Phase.OFFSET:
            # 에지에서 작업 위치까지 **탐색 방향 그대로** 더 간다 (백래시 회피).
            off = self.offsets.get(self.axis, 0.0)
            p = self.pos.get(self.axis)
            ref = self.refs.get(self.axis)
            if p is None or ref is None:
                self._stop_axis(self.axis)
                self.get_logger().warning(f"{self.axis}: 위치를 몰라 오프셋 이동 생략")
                self._next_axis()
                return
            # 오프셋은 **명령 부호 기준의 부호 있는 값**이다 — 탐색 방향과 반대일 수
            # 있다. yaw 가 그렇다: 12시가 감지 구간보다 위라, 감소 방향으로 에지를
            # 찾은 뒤 **증가 방향으로 되돌아** 12시에 간다.
            # 위치 토픽은 counts 부호와 반대이므로(토픽 = -counts/728), 명령 부호가
            # 양수면 토픽은 **감소**한다. 그래서 진행량을 부호까지 보고 판단한다.
            sign = 1.0 if off >= 0 else -1.0
            progress = (ref - p) * sign            # 명령 방향으로 얼마나 갔나
            if progress >= abs(off):
                self._stop_axis(self.axis)
                self.get_logger().info(
                    f"{self.axis}: 오프셋 {off:+.2f}° 이동 완료 (실제 {progress:+.2f}°) "
                    f"→ 작업 위치")
                self._next_axis()
            else:
                self._cmd_speed(self.axis, self._with_breakaway(
                    self.axis, sign * self._fine_of(self.axis), elapsed))

        elif self.phase is Phase.FINE:
            if at_home:
                self._stop_axis(self.axis)
                p = self.pos.get(self.axis)
                self.refs[self.axis] = p
                self.get_logger().info(
                    f"{self.axis}: 원점 확정" + (f" (위치 {p:.2f}°)" if p is not None else
                                              " (위치 토픽 없음)"))
                if self.offsets.get(self.axis):
                    self._enter(Phase.OFFSET,
                                f"작업 위치까지 {self.offsets[self.axis]:.2f}° 더")
                else:
                    self._next_axis()
            else:
                self._cmd_speed(self.axis, self._with_breakaway(
                    self.axis, cfg['dir'] * self._fine_of(self.axis), elapsed))

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.phase.value,
            'axis': self.axis,
            'detail': self.detail,
            'queue': self.queue,
            'refs': {k: v for k, v in self.refs.items()},
        }, ensure_ascii=False)))

    def destroy_node(self):
        try:
            for a in AXES:
                self._cmd_speed(a, 0.0)
            self._lock_all()            # 노드가 죽어도 Z 는 잠가두고 간다
            time.sleep(0.1)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HomingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
