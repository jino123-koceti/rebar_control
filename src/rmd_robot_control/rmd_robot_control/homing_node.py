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
                          load_precheck, load_seek_dirs, precheck_violation,
                          load_pose_id, identify_pose, pose_label,
                          load_search_limit, load_ready_pose, ready_target)
from .axis_config import HOMING_AXES as AXES
from std_msgs.msg import Bool, Float32, Float64, Float64MultiArray, Int32, String


class Phase(Enum):
    IDLE = 'idle'
    ARM = 'arm'            # 브레이크 해제가 모터에 먹을 때까지 기다린다
    SEEK = 'seek'          # 원점 리미트를 향해 이동
    BACK_OFF = 'back_off'  # 리미트에서 살짝 빠짐 (센서 해제)
    FINE = 'fine'          # 느린 속도로 재접근 → 레퍼런스 기록
    OFFSET = 'offset'      # 에지에서 작업 위치까지 더 간다 (yaw 12시)
    READY = 'ready'        # 전 축 호밍 후 작업 시작 자세로 (Y 중앙 → yaw 1번)
    DONE = 'done'
    FAILED = 'failed'


# 축 정의 — **모터 ID 는 config/axes.yaml 에서 읽는다.** 코드에 0x14X 를 쓰지 않는다
# (tools/check 의 R1). 2차년도는 ID 가 10개 파일에 흩어져 있어 3차년도로 넘어올 때
# 주석과 실제가 어긋났다.
#   joint: 명령 토픽 번호,  home_limit: 원점 리미트,  far_limit: 반대쪽(안전 확인용)
#   dir: 원점 방향 부호 (실측으로 확정해야 한다 — 기본값은 미검증)
LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')
# **기구 간섭이 순서를 정한다** (2026-09-30 실장비 확인, 사용자 지정):
#   z   : 먼저 올린다(z_min=위). 내려와 있으면 X 이동 중 철근 배근에 걸린다
#   yaw : 12시로. **y 보다 먼저여야 한다**
#   x   : x_min 은 12시에서 간섭 없음
#   y   : y_min/y_max 는 **yaw 가 12시일 때만** 도달 가능. 아니면 상부 프레임을 친다
# ⚠ 옛 순서 (z,x,y,yaw) 는 y 가 yaw 보다 앞이라 12시가 아닌 상태로 y 가 리미트까지
#   달려 프레임을 친다. 되돌리지 말 것.
# ⚠ x+y 동시 이동은 미구현 — 순서는 같으므로 안전성은 동일하고 시간만 더 걸린다.
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
        self.declare_parameter('ready_tol_deg', 3.0)      # 준비자세 도달 판정 (모터축)
        # 이탈(breakaway): 출발 직후 이 시간 안에 안 움직이면 더 센 속도로 민다
        # 권한 요청 후 이 시간까지 못 받으면 실패한다 (중재기가 있는 경우에만 적용)
        self.declare_parameter('grant_timeout', 3.0)
        # 스톨 감지 — 명령을 보내는데 엔코더가 안 변하면 기계 끝에 닿은 것이다.
        # yaw 는 끝 리미트가 아예 없어서 이것이 **유일한 보호 수단**이다.
        # ⚠ **정지마찰 돌파보다 길어야 한다.** 2026-10-03 실측: 정지 상태에서 속도
        # 명령을 주면 전류가 0.22→5.41A 로 오르며 **2.4초간 거의 안 움직이다** 풀린다
        # (풀린 뒤는 1.2A). 2.0초로 두면 그 돌파를 스톨로 오판한다.
        self.declare_parameter('stall_sec', 4.0)
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
        self.ready_tol = float(g('ready_tol_deg').value)
        self.grant_timeout = float(g('grant_timeout').value)
        for name in AXES:
            AXES[name]['dir'] = int(g(f'{name}_dir').value)

        self.speed_pubs = {n: self.create_publisher(Float32, f"/joint_{c['joint']}/speed", 10)
                           for n, c in AXES.items()}
        # 아는 거리는 위치 제어로 보낸다(READY). stage_node 와 같은 규약:
        # Float64MultiArray [목표각도, 최대속도dps].
        self.pos_pubs = {n: self.create_publisher(
            Float64MultiArray, f"/joint_{c['joint']}/position", 10)
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
        # {축: {탐색방향: 모터축 도}} — 접근 방향마다 다르다 (감지판 폭 때문)
        self.offsets = load_home_offsets()
        if self.offsets:
            self.get_logger().info(
                "원점 후 오프셋 (탐색방향별): " + ", ".join(
                    f"{k} " + "/".join(f"{d:+d}→{v:+.2f}°" for d, v in sorted(o.items()))
                    for k, o in self.offsets.items()))
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
        # 브레이크 해제 확인용. **고정 시간 대기로는 모자란다** — 0x77 이 먹기까지
        # 1.50초 걸린 경우를 봤다(2026-10-03). 문 채로 명령하면 최대 전류로 밀면서
        # 거의 안 움직인다 — 증상이 "모터 고장" 과 같다.
        self.brake_ok = {}
        for name, mid in motor_ids.items():
            self.create_subscription(
                Bool, f"/motor_{mid}/brake",
                (lambda k: (lambda m: self.brake_ok.__setitem__(k, m.data)))(name), 10)
        self.pose_id = load_pose_id('yaw')       # yaw 자세 판별 (12시 ±3° 전제를 대체)
        self.search_limit = load_search_limit('yaw')
        (self.ready_order, self.ready_off,
         self.ready_yaw_motor, self.ready_speed) = load_ready_pose()
        self._off_used = {}            # 축 → OFFSET 에서 실제 적용한 오프셋
        self._ready_queue = []
        self._ready_sent = 0.0
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

    def _finish(self):
        self._lock_all()
        self._release_control()
        self._enter(Phase.DONE, '완료')
        self.get_logger().info(f"호밍 완료 — 레퍼런스 {self.refs}")

    def _start_ready(self):
        """준비자세로 옮긴다 — 호밍이 끝난 자리가 작업 시작 자세가 아니다.

        순서는 `axes.yaml` 의 `ready.order` 를 따른다 (**Y 먼저, yaw 나중**).
        Y 는 yaw 가 12시일 때만 리미트에 닿는다고 기록돼 있어, 검증된 조건에서
        Y 를 먼저 옮기는 쪽이 안전하다.
        """
        self._ready_queue = [a for a in self.ready_order
                             if self._ready_target(a) is not None]
        skip = [a for a in self.ready_order if a not in self._ready_queue]
        if skip:
            self.get_logger().warning(
                f"준비자세 건너뜀: {', '.join(skip)} (원점 레퍼런스나 환산값이 없다)")
        if not self._ready_queue:
            self._finish()
            return
        self.axis = None
        self._next_ready()

    def _next_ready(self):
        if not self._ready_queue:
            self.get_logger().info("준비자세 완료")
            self._finish()
            return
        prev = self.axis
        self.axis = self._ready_queue.pop(0)
        if prev is not None and prev != self.axis:
            self._cmd_speed(prev, 0.0)
            self._brake('shutdown', prev)
        self._brake('release', self.axis)
        self._ready_sent = 0.0
        tgt = self._ready_target(self.axis)
        self._enter(Phase.READY, f"{self.axis} → {tgt:+.1f}°")

    def _ready_target(self, axis):
        return ready_target(axis, self.refs.get(axis), self.ready_off,
                            self.ready_yaw_motor,
                            self._off_used.get(axis) or self._offset_of(axis))

    def _cmd_pos(self, axis, deg, speed):
        self.pos_pubs[axis].publish(
            Float64MultiArray(data=[float(deg), float(speed)]))

    def _next_axis(self):
        if not self.queue:
            prev = self.axis
            if prev is not None:
                self._cmd_speed(prev, 0.0)
            self._start_ready()
            return
        prev = self.axis
        self.axis = self.queue.pop(0)
        if prev is not None and prev != self.axis:
            self._brake('lock', prev)           # 끝난 축은 바로 잠근다
        self._brake('release', self.axis)       # 움직일 축만 푼다
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
        """호밍 시작 전제 — yaw 는 **자세 판별**로 본다.

        옛 전제 "사용자가 12시 ±3° 에 놓는다" 는 운용 현실(전원 차단 시 1~4번 중
        하나)과 맞지 않았다. 단회전값이 자세마다 건 4.54° 이상 떨어져 유일하게
        갈리므로 손으로 맞출 필요가 없다. 판별된 자세로 **탐색 방향까지** 정한다 —
        에지가 자세 범위 안쪽이라 방향이 ± 두 가지이고 틀리면 끝단으로 달린다.
        """
        if self.pose_id and 'yaw' in AXES:
            n, info = identify_pose(self.pose_id, self.single.get('yaw'))
            if n is None:
                return f"yaw {info}"
            AXES['yaw']['dir'] = int(info['dir'])
            self.get_logger().info(
                f"yaw 자세 판별: {pose_label(n)} (오차 건 {info['err_gun']:+.2f}°) "
                f"→ 탐색 방향 {info['dir']:+d}, 에지까지 건 {info['to_edge_gun']:+.2f}°")
            return None
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
        # action: release / lock / shutdown (= lock + 0x80)
        self.brake_pub.publish(String(data=arg))

    def _release_control(self):
        self._want_control = False
        self._request_control('release')

    def _lock_all(self):
        """전 축 주차. **Z 를 먼저** 잠근다 — 떨어지는 축이 우선이다.

        `shutdown` = 잠금 후 여자 해제. **잠금만으로는 전류가 끊기지 않는다** —
        모터가 여자된 채 마지막 속도 명령(0 이어도)을 계속 수행해 브레이크와 반력을
        상대로 밀면서 발열한다 (2026-10-03 yaw 실측: -4.02A 계속, 29→43°C).
        부하가 없으면 증상이 안 보여 놓치기 쉽다.
        """
        for a in ('z',) + tuple(x for x in AXES if x != 'z'):
            self._cmd_speed(a, 0.0)          # 먼저 명령을 거둔다
            self._brake('shutdown', a)

    def _offset_of(self, axis):
        """이 축에 지금 쓸 오프셋 (모터축 도, 명령 부호).

        **탐색 방향에 따라 다르다.** 감지판에 폭이 있어서 증가 방향으로 접근하면
        아래 경계, 감소 방향이면 위 경계에서 켜진다. 방향을 무시하고 한 값만 쓰면
        그 차이만큼 작업 위치를 비껴간다 (2026-10-03 실장비에서 그렇게 틀어졌다).
        """
        per = self.offsets.get(axis)
        if not per:
            return 0.0
        d = int(AXES[axis]['dir'])
        return float(per.get(d, per.get(-d, 0.0)))

    def _fine_of(self, axis):
        """축별 정밀 속도. 없으면 공통값."""
        return float(AXES[axis].get('fine', self.fine_speed))

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
            # **해제를 확인하고 넘어간다.** 고정 시간은 모자랄 수 있다 (실측 1.50초).
            # 상태를 못 받는 경우(토픽 없음)에는 종전처럼 시간으로만 판단한다.
            told = self.brake_ok.get(self.axis)
            if told is False:
                if elapsed > self.arm_sec * 6:
                    self._fail(f"{self.axis}: 브레이크가 {elapsed:.1f}초 동안 풀리지 "
                               f"않았다 — 0x77 이 먹는지 확인하세요")
                return
            if elapsed >= self.arm_sec:
                if self.limit[cfg['home_limit']]:
                    self.get_logger().info(
                        f"{self.axis}: 이미 {cfg['home_limit']} 에 있음 → 후퇴부터")
                    self._enter(Phase.BACK_OFF, '이미 원점')
                else:
                    self._enter(Phase.SEEK, f"{cfg['home_limit']} 탐색")
            return

        if self.phase is Phase.READY:
            # 아는 거리를 가는 단계다 → 위치 제어. 브레이크 해제를 먼저 확인한다.
            told = self.brake_ok.get(self.axis)
            if told is False:
                self._brake('release', self.axis)
                if elapsed > self.arm_sec * 6:
                    self._fail(f"{self.axis}: 준비자세 전 브레이크가 풀리지 않았다")
                return
            tgt = self._ready_target(self.axis)
            p = self.pos.get(self.axis)
            if p is not None and abs(p - tgt) <= self.ready_tol:
                self.get_logger().info(
                    f"{self.axis}: 준비자세 도달 ({p:+.1f}°, 목표 {tgt:+.1f}°)")
                self._next_ready()
                return
            if elapsed > self.axis_timeout:
                self._fail(f"{self.axis}: 준비자세 타임아웃 — "
                           f"현재 {p if p is None else f'{p:+.1f}'}°, 목표 {tgt:+.1f}°")
                return
            # **0xA4 는 명령의 일부만 가는 경우가 있다** (실측: +2.00° 명령에 +1.23°).
            # 그래서 한 번 보내고 끝내지 않고 2Hz 로 같은 절대목표를 다시 보낸다.
            if time.time() - self._ready_sent > 0.5:
                self._ready_sent = time.time()
                self._cmd_pos(self.axis, tgt, self.ready_speed)
            return

        if elapsed > self.axis_timeout:
            self._fail(f"{self.axis} {self.phase.value} 타임아웃 {self.axis_timeout:.0f}s")
            return
        # 반대쪽 리미트가 눌리면 방향이 반대다 — 즉시 멈춘다
        if cfg['far_limit'] and self.limit[cfg['far_limit']]:
            self._fail(f"{self.axis}: 반대쪽 리미트({cfg['far_limit']})에 닿음 — 방향 부호 확인")
            return

        at_home = bool(self.limit[cfg['home_limit']])

        # ── 탐색 상한 ───────────────────────────────────────────────────────
        # yaw 는 리미트가 하나뿐이고 에지가 자세 범위 **안쪽**이라, 방향을 틀리면
        # 에지를 못 만나고 기계 끝단으로 달린다. ⚠ **방향을 뒤집지 않는다** —
        # 역방향 재탐색은 이미 박은 뒤의 동작이다 (9/30 에 10.7A/86°C 까지 갔다).
        # **거리가 아니라 시간으로 잰다**: 위치 토픽은 is_moving 인 모터만 갱신되므로
        # 첫 값이 묵으면 거리가 엉뚱해진다 — 움직이지도 않은 yaw 가 "415° 를 갔다" 며
        # 2초 만에 헛실패했다 (2026-10-03). 시간은 그 실패 모드가 없다.
        if (self.phase is Phase.SEEK and self.search_limit
                and self.axis == 'yaw' and not at_home):
            budget = self.search_limit / max(self._seek_of(self.axis), 1.0) * 2.0
            if elapsed > budget:
                self._stop_axis(self.axis)
                self._fail(f"{self.axis}: {elapsed:.1f}초 탐색했는데 "
                           f"{cfg['home_limit']} 가 켜지지 않았다 "
                           f"(상한 {budget:.1f}초 = 모터축 {self.search_limit:.0f}° 분) "
                           f"— 탐색 방향이나 기구를 확인하세요")
                return

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
                # ~~fine_stall_ok: 못 밀면 SEEK 에지를 원점으로 인정~~ → 2026-10-03
                # 제거. "홈 쪽이 무거워서 못 민다" 가 아니라 **정지마찰 돌파**였다
                # (실측: 0.22→5.41A 로 2.4초 버틴 뒤 풀리고, 그 뒤 1.2A). 그 2.4초
                # 동안 더 나쁜 레퍼런스를 받아들이면 정밀 재접근의 의미가 없다.
                # stall_sec 을 돌파 시간보다 넉넉히 두는 것이 옳은 대처다.
                self._stop_axis(self.axis)
                self._fail(f"{self.axis}: 스톨 — {self.stall_sec:.1f}초 동안 "
                           f"{self.stall_deg:.1f}° 도 못 움직였다 "
                           f"(기계 끝·브레이크 미해제·과부하 확인)")
                return

        if self.phase is Phase.SEEK:
            if at_home:
                self._stop_axis(self.axis)
                self.get_logger().info(f"{self.axis}: {cfg['home_limit']} 도달 "
                                       f"(단 {self.single.get(self.axis)}) → 후퇴")
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
                self._cmd_speed(self.axis, -cfg['dir'] * self._backoff_of(self.axis))
            else:
                if self._off_since is None:
                    self._off_since = time.time()
                    self.get_logger().info(f"{self.axis}: {cfg['home_limit']} 해제됨 "
                                           f"({elapsed:.1f}초, 단 {self.single.get(self.axis)})")
                if time.time() - self._off_since < self.back_off_sec:
                    self._cmd_speed(self.axis, -cfg['dir'] * self._backoff_of(self.axis))
                else:
                    self._stop_axis(self.axis)
                    self.get_logger().info(f"{self.axis}: 정밀 재접근")
                    self._enter(Phase.FINE, '정밀 재접근')

        elif self.phase is Phase.OFFSET:
            # 에지 → 작업 위치는 **거리가 알려진 이동**이다 → 위치 제어.
            # 속도로 밀며 진행량을 지켜보면 **감속 구간이 그대로 오버슈트**로 남는다
            # (2026-10-03 실측: +71.82° 명령에 실제 +78.53°, 모터축 6.71° 초과).
            # 그 오차가 12시 기준과 준비자세로 그대로 전파됐다.
            # 오프셋은 **명령 부호** 기준이고 토픽은 counts 와 반대이므로(토픽 =
            # −counts/728), 목표 토픽각 = ref − off 다.
            off = self._off_used.get(self.axis) or self._offset_of(self.axis)
            ref, p = self.refs.get(self.axis), self.pos.get(self.axis)
            if p is None or ref is None:
                self._stop_axis(self.axis)
                self.get_logger().warning(f"{self.axis}: 위치를 몰라 오프셋 이동 생략")
                self._next_axis()
                return
            tgt = ref - off
            if abs(p - tgt) <= self.ready_tol:
                self._stop_axis(self.axis)
                self.get_logger().info(
                    f"{self.axis}: 오프셋 {off:+.2f}° 완료 "
                    f"(실제 {ref - p:+.2f}°, 목표오차 {p - tgt:+.2f}°) → 작업 위치")
                self._next_axis()
            elif time.time() - self._ready_sent > 0.5:
                # 0xA4 는 명령의 일부만 가는 경우가 있다 → 같은 절대목표를 다시 보낸다
                self._ready_sent = time.time()
                self._cmd_pos(self.axis, tgt, self._fine_of(self.axis))

        elif self.phase is Phase.FINE:
            if at_home:
                self._stop_axis(self.axis)
                p = self.pos.get(self.axis)
                self.refs[self.axis] = p
                # **단회전값을 같이 찍는다.** 위치 토픽은 전원 세션마다 기준이 달라져
                # 기록된 에지값과 비교할 수 없다. 단회전은 전원과 무관해서, 원점이
                # 실제로 에지에 잡혔는지 바로 대조된다 (2026-10-03: 에지보다 건 1.2°
                # 아래에 잡히는 것 같았는데 역산으로는 확정할 수 없었다).
                self.get_logger().info(
                    f"{self.axis}: 원점 확정 (위치 {p}, 단 {self.single.get(self.axis)})")
                off = self._offset_of(self.axis)
                if off:
                    self._off_used[self.axis] = off
                    self._ready_sent = 0.0
                    self._enter(Phase.OFFSET, f"작업 위치까지 {off:+.2f}° 더")
                else:
                    self._next_axis()
            else:
                self._cmd_speed(self.axis, cfg['dir'] * self._fine_of(self.axis))

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
