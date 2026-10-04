#!/usr/bin/env python3
"""L4 — 결속 지점 하나를 끝까지 수행하는 시퀀스.

    자세 선택 → Z 확보 → XY 후퇴 → 회전 → XY 이동 → Z 하강 → 결속건 → Z 상승

**Z 와 결속건은 기본이 꺼짐이다.** 목표에 z 가 없으면 Z 단계를 건너뛰고 XY·자세만
한다 (2026-10-04 까지 실장비로 검증된 범위). `gun_enabled` 도 기본 False 다 —
되돌릴 수 없는 동작이라 켜는 것이 명시적 결정이어야 한다.

⚠ 후퇴 목표는 창 **경계에서 `transit_pad_mm` 안쪽**이다. 경계에 붙여 놓고 돌리면
여유가 0 이고, 자세 사이 범위는 표본 몇 점으로 가늠한 것이라 그 사이에 더 나쁜
각도가 있을 수 있다 (실측 셋 다 양 끝보다 나빴다).

`stage_node` 는 "축을 목표까지 어떻게 보내나" 만 안다. 이 노드는 그 위에서
**순서**를 정한다 (YEAR3_ARCHITECTURE.md §4 L4). 축도 CAN 도 직접 건드리지
않는다 — `/stage/*` 로만 명령한다.

## 왜 "선택 → 회전 → 이동" 이 아니라 네 단계인가

**회전은 중간 자세를 지나간다.** 1번에서 3번으로 가면 12시와 2번을 지나므로,
회전을 시작하기 전에 XY 가 *지나가는 자세 전부의 교집합* 안에 있어야 한다.
그래서 **후퇴**가 한 단계 들어간다:

    PRECHECK → (Z_CLEAR) → RETRACT → ROTATE → MOVE_XY → (Z_DOWN → FIRE → Z_UP)

**Z 는 회전·이동보다 먼저 올린다.** 건이 내려간 채로 돌리거나 옮기면 철근을
긁는다. 작업영역 검사(`envelope`)에는 X·Y 만 있어서 이것은 코드가 지켜야 한다.

이게 빠지면 회전 도중에 프레임을 친다. 2026-10-03 에 12시의 X 상한(361.0mm)을
안 재고 3번(383.2)을 최악으로 쓰던 동안 그 여유가 22mm 과했다.

## 자세는 왜 상위가 지정할 수 있나

캘리브레이션 모델은 **같은 교차점을 자세마다 다른 스테이지 좌표로** 내놓는다
(자세 오프셋 최대 122mm). 그러니 좌표를 계산한 쪽 — 모델을 가진 상위 — 만이
어느 자세의 좌표인지 안다. `/tying/goal_pose` 가 그 통로다. 지정이 없으면
아래 사분면 규칙으로 직접 고른다 (손으로 한 점 보낼 때의 경로다).

지정이 없으면 **지금 자세로 닿는지 먼저 본다** — 닿으면 바꾸지 않고, 못 닿으면
닿는 자세 중 회전이 가장 짧은 쪽으로 바꾼다 (사용자 지정 규칙, 2026-10-04).
자세 변경이 20초가 넘기 때문이다.

⚠ `axes.yaml` 의 `stage.pose_select`(사분면 규칙)는 **더 쓰지 않는다.** 점마다
독립으로 자세를 정해서 1→4→1 같은 왕복 회전이 생겼다. 설정은 남겨 두었다 —
지웠다가 "왜 없앴는지" 를 잃는 쪽이 더 나쁘다.

## 토픽

  구독  /tying/goal     geometry_msgs/Point  결속 지점 mm. z 는 결속 깊이
                                             (NaN 이면 `z_tie_mm`, 그것도 NaN 이면 Z 생략)
        /tying/goal_pose std_msgs/Int32      쓸 자세. 목표보다 **먼저** 와야 한다.
                                             없으면 사분면 규칙으로 직접 고른다
        /tying/abort    std_msgs/Empty       즉시 중단
        /stage/status   String(JSON)         현재 mm·자세·이동 여부
        /safety/state   SafetyState          비상정지·STOP
  발행  /stage/goal     geometry_msgs/Point  XY·Z 이동
        /motor_0/vel    std_msgs/Float32     결속건 (Pololu, `gun_enabled` 일 때만)
        /stage/yaw_pose std_msgs/Int32       자세 회전
        /stage/stop     std_msgs/Empty       중단 전파
        /tying/status   String(JSON)         단계·목표·사유

## 실패는 전부 거부다

호밍 안 됨 / 자세 판별 실패(자세 사이) / 목표가 선택 자세로 안 닿음 / 단계
타임아웃 — 모두 멈추고 사유를 남긴다. 추정해서 움직이지 않는다. yaw 는 끝
리미트가 없고 상부축에는 기구 끝단 보호가 없다.
"""

import json
import math
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Int32, String
from rebar_base_interfaces.msg import SafetyState

from .axis_config import (envelope_violation, load_envelope, load_pose_id,
                          pose_label, transit_window)

NAN = float('nan')


class Step(Enum):
    IDLE = 'idle'
    PRECHECK = 'precheck'
    Z_CLEAR = 'z_clear'        # Z 를 안전높이로 (회전·이동 전)
    RETRACT = 'retract'        # XY 를 회전 안전창 안으로
    ROTATE = 'rotate'          # yaw 를 목표 자세로
    MOVE_XY = 'move_xy'        # 결속 지점으로
    Z_DOWN = 'z_down'          # 결속 깊이로 하강
    FIRE = 'fire'              # 결속건 작동
    Z_UP = 'z_up'              # 안전높이로 복귀
    DONE = 'done'
    FAILED = 'failed'


class TyingSequence(Node):
    def __init__(self):
        super().__init__('tying_sequence')

        self.declare_parameter('step_timeout_sec', 45.0)
        self.declare_parameter('arrive_tol_mm', 2.0)
        # 후퇴 목표를 회전 안전창 **경계에서 이만큼 안쪽**으로 잡는다.
        # 경계에 딱 붙여 놓고 회전하면 여유가 0 이다 — 도달 판정이 ±2mm 고,
        # 자세 사이 범위는 표본 몇 점으로 가늠한 것이라 그 사이에 더 나쁜 각도가
        # 있을 수 있다 (실측 셋 다 양 끝보다 나빴다). `envelope.margin_mm` 은
        # 센서·측정 오차용이고, 이것은 **회전 중 자세 변화분**을 위한 별도 여유다.
        self.declare_parameter('transit_pad_mm', 15.0)
        # ── Z ──────────────────────────────────────────────────────────────
        # Z 는 **기본이 꺼짐**이다. 목표에 z 가 없고 `z_tie_mm` 도 NaN 이면
        # XY·자세만 하고 끝낸다 — 2026-10-04 까지 실장비로 검증된 범위가 그것이다.
        self.declare_parameter('z_safe_mm', 0.0)     # 회전·이동 중 Z 안전높이
        self.declare_parameter('z_tie_mm', NAN)      # 목표에 z 가 없을 때 쓸 깊이
        self.declare_parameter('z_tol_mm', 2.0)
        # ⚠ 캘리브레이션 모델이 틀린 깊이를 내도 **매트를 찍지 않도록** 범위를
        #   강제한다. 작업영역 검사(`envelope`)에는 X·Y 만 있다 — Z 는 자세별
        #   실측이 없다. 모델 잔차가 Z 2.7mm 라 여유를 넉넉히 둔다.
        self.declare_parameter('z_tie_min_mm', -95.0)
        self.declare_parameter('z_tie_max_mm', -40.0)
        # ── 결속건 ─────────────────────────────────────────────────────────
        # **기본이 꺼짐**이다. 켜는 것은 명시적 결정이어야 한다 — 되돌릴 수 없고
        # 작업영역 안에 사람이 있을 수 있다. 꺼져 있으면 단계는 지나가되
        # 모터로는 아무것도 보내지 않는다 (시연 전 예행연습이 그 상태다).
        self.declare_parameter('gun_enabled', False)
        self.declare_parameter('gun_topic', '/motor_0/vel')
        self.declare_parameter('gun_speed', 1.0)     # Pololu Float32 -1.0~1.0
        self.declare_parameter('gun_fire_sec', 1.0)
        self.declare_parameter('gun_return_sec', 1.0)

        self.step_timeout = float(self.get_parameter('step_timeout_sec').value)
        self.tol = float(self.get_parameter('arrive_tol_mm').value)
        self.pad = float(self.get_parameter('transit_pad_mm').value)
        self.z_safe = float(self.get_parameter('z_safe_mm').value)
        self.z_tie_default = float(self.get_parameter('z_tie_mm').value)
        self.z_tol = float(self.get_parameter('z_tol_mm').value)
        self.z_lo = float(self.get_parameter('z_tie_min_mm').value)
        self.z_hi = float(self.get_parameter('z_tie_max_mm').value)
        self.gun_on = bool(self.get_parameter('gun_enabled').value)
        self.gun_speed = float(self.get_parameter('gun_speed').value)
        self.gun_fire = float(self.get_parameter('gun_fire_sec').value)
        self.gun_back = float(self.get_parameter('gun_return_sec').value)

        self.env = load_envelope()
        self.pose_id = load_pose_id('yaw')
        if self.env is None or self.pose_id is None:
            self.get_logger().error(
                "axes.yaml 의 stage.envelope / yaw 자세표를 못 읽었다 — "
                "자세 선택도 회전 안전 검사도 할 수 없다. 목표를 거부한다")

        self.goal_pub = self.create_publisher(Point, '/stage/goal', 10)
        self.yaw_pub = self.create_publisher(Int32, '/stage/yaw_pose', 10)
        self.stop_pub = self.create_publisher(Empty, '/stage/stop', 10)
        self.status_pub = self.create_publisher(String, '/tying/status', 10)
        self.gun_pub = self.create_publisher(
            Float32, str(self.get_parameter('gun_topic').value), 10)

        self.stage = None            # /stage/status 최신 JSON
        self.safety = None
        self.create_subscription(String, '/stage/status', self._on_stage, 10)
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 1)
        self.create_subscription(Point, '/tying/goal', self._on_goal, 10)
        # 자세를 **상위가 지정**할 수 있다. 캘리브레이션 모델은 같은 교차점을
        # 자세마다 다른 스테이지 좌표로 내놓는다 (오프셋 최대 122mm) — 그러니
        # 자세를 고른 쪽만이 그 자세의 좌표를 계산할 수 있다. 지정이 없으면
        # 사분면 규칙으로 직접 고른다 (손으로 한 점 보낼 때의 경로다).
        self.create_subscription(Int32, '/tying/goal_pose',
                                 self._on_goal_pose, 10)
        self.create_subscription(Empty, '/tying/abort',
                                 lambda m: self._fail('중단 명령'), 10)

        self.step = Step.IDLE
        self.goal = None             # (x, y) mm
        self.goal_z = None           # 결속 깊이 mm (None = Z 단계 안 함)
        self.want_pose = None
        self.asked_pose = None       # /tying/goal_pose 로 지정된 자세
        self._fire_t = 0.0           # 결속건 하위단계 시작 시각
        self._fire_phase = 0
        # 접수한 목표 수. **상위가 "내 목표가 들어갔나" 를 가리는 유일한 근거다.**
        # `step` 만 보면 안 된다 — 목표가 **이미 충족된 자리**에서는
        # precheck→done 이 한 tick 안에 끝나서 상위는 'done' 밖에 못 보고
        # "시작하지 않았다" 로 오판한다 (2026-10-04 에 검출 자세에 이미 서 있어서
        # 미션이 첫 단계에서 죽었다).
        self.goals = 0
        self._retract_tgt = None     # 보낸 후퇴 목표 (도착까지 붙잡는다)
        self._rej0 = None            # 단계 시작 시점의 stage_node 거부 횟수
        self._passed = []            # 회전이 지나가는 자세들
        self.detail = '대기'
        self.t_step = 0.0
        self._sent = 0.0

        self.create_timer(0.2, self.tick)
        self.create_timer(0.5, self._publish)
        self.get_logger().info(
            "결속 시퀀스 시작 — /tying/goal (x,y,z mm) 로 결속 지점을 준다. "
            "자세 선택 → Z 확보 → 후퇴 → 회전 → 이동"
            + (" → Z 하강 → 결속건 → Z 상승" if self.gun_on else
               " → Z 하강 → (결속건 꺼짐) → Z 상승")
            + f". 결속건 {'켜짐' if self.gun_on else '꺼짐'}, "
              f"Z 안전높이 {self.z_safe:.0f}mm, "
              f"결속깊이 허용 [{self.z_lo:.0f}, {self.z_hi:.0f}]mm")

    # ---- 입력 --------------------------------------------------------------
    def _on_stage(self, msg):
        try:
            self.stage = json.loads(msg.data)
        except ValueError:
            pass

    def _on_safety(self, msg):
        self.safety = msg

    def _on_goal_pose(self, msg):
        """다음 목표에 쓸 자세. 목표보다 **먼저** 와야 한다."""
        self.asked_pose = int(msg.data)

    def _on_goal(self, msg):
        if self.step not in (Step.IDLE, Step.DONE, Step.FAILED):
            return self._reject(f'이미 진행 중이다 ({self.step.value})')
        if math.isnan(msg.x) or math.isnan(msg.y):
            return self._reject('결속 지점은 X·Y 가 둘 다 있어야 한다')
        z = float(msg.z) if not math.isnan(msg.z) else self.z_tie_default
        if not math.isnan(z) and not (self.z_lo <= z <= self.z_hi):
            return self._reject(
                f'결속 깊이 z={z:.1f}mm 가 허용 범위 '
                f'[{self.z_lo:.0f}, {self.z_hi:.0f}] 밖이다 — 모델 예측을 의심하라')
        self.goal = (float(msg.x), float(msg.y))
        self.goal_z = None if math.isnan(z) else z
        self.want_pose = None
        self._retract_tgt = None
        self._fire_phase = 0
        self.goals += 1
        self._enter(Step.PRECHECK,
                    f'목표 x={self.goal[0]:.1f} y={self.goal[1]:.1f}mm'
                    + ('' if self.goal_z is None else f' z={self.goal_z:.1f}mm'))

    # ---- 단계 전이 ---------------------------------------------------------
    def _enter(self, step, detail):
        # 이 단계를 시작하는 시점의 거부 횟수를 기억한다 — 이후에 늘면 내 명령 탓이다
        self._rej0 = (self.stage or {}).get('rejects')
        self.step = step
        self.detail = detail
        self.t_step = time.time()
        self._sent = 0.0
        self.get_logger().info(f"[{step.value}] {detail}")
        self._publish()

    def _reject(self, why):
        self.get_logger().error(f"거부 — {why}")
        self.step = Step.FAILED
        self.detail = f'거부: {why}'
        self._publish()

    def _fail(self, why):
        """진행 중이던 것을 멈추고 실패로 끝낸다. 축 정지는 stage_node 가 한다."""
        if self.step in (Step.IDLE, Step.DONE, Step.FAILED):
            return
        self.stop_pub.publish(Empty())
        # ⚠ 결속건이 돌고 있을 수 있다 — 축을 멈춰도 건은 안 멈춘다
        if self.gun_on:
            self.gun_pub.publish(Float32(data=0.0))
        self._fire_phase = 0
        self.step = Step.FAILED
        self.detail = f'중단: {why}'
        self.get_logger().warning(self.detail)
        self._publish()

    # ---- 상태 읽기 ---------------------------------------------------------
    def _mm(self):
        c = (self.stage or {}).get('current_mm') or {}
        return c.get('x'), c.get('y')

    def _z(self):
        return ((self.stage or {}).get('current_mm') or {}).get('z')

    def _pose(self):
        return (self.stage or {}).get('pose')

    def _stage_busy(self):
        return bool((self.stage or {}).get('moving'))

    def _stage_detail(self):
        return (self.stage or {}).get('detail', '')

    # ---- 진행 --------------------------------------------------------------
    def tick(self):
        if self.step in (Step.IDLE, Step.DONE, Step.FAILED):
            return
        s = self.safety
        if s is not None and (s.estop or s.stop_switch or s.inputs_stale):
            return self._fail('안전 정지')
        if self.stage is None:
            return self._fail('/stage/status 가 없다 — stage_node 가 떠 있는가')
        # stage_node 가 거부했으면 사유를 그대로 올린다 (삼키면 진단이 어렵다).
        # ⚠ `detail` 문자열로 보면 **한참 전의 거부**에 걸린다 — 그 필드는 다음
        #   일이 생길 때까지 남는다. 2026-10-03 실장비에서 그래서 시작 즉시
        #   중단됐다 (몇 분 전 가드 시험의 "모르는 자세 7" 이 남아 있었다).
        #   **거부 횟수가 늘었는지**로 본다.
        rej = (self.stage or {}).get('rejects')
        if rej is not None and self._rej0 is not None and rej > self._rej0:
            self._rej0 = rej
            return self._fail(f"stage_node — {self._stage_detail()[4:]}")
        if time.time() - self.t_step > self.step_timeout:
            return self._fail(f'{self.step.value} 타임아웃 {self.step_timeout:.0f}s')

        getattr(self, '_do_' + self.step.value)()

    def _do_precheck(self):
        if self.env is None or self.pose_id is None:
            return self._reject('범위 표나 yaw 자세표가 없다')
        x, y = self._mm()
        if x is None or y is None:
            return self._reject('현재 X·Y mm 를 모른다 — 먼저 호밍하세요 (/homing_cmd)')
        cur = self._pose()
        if cur is None:
            return self._reject(
                f"yaw 자세를 못 가린다 — {(self.stage or {}).get('pose_detail')}. "
                f"호밍하면 1번 자세로 정렬된다")
        if self.asked_pose is not None:
            want, why = self.asked_pose, f'{pose_label(self.asked_pose)} 지정'
            self.asked_pose = None            # 지정은 **이 목표 한 번만** 쓴다
        else:
            want, why = self._pick_pose(cur)
        if want is None:
            return self._reject(f'자세를 고를 수 없다 — {why}')
        bad = envelope_violation(self.env, want,
                                 {'x': self.goal[0], 'y': self.goal[1]})
        if bad:
            return self._reject(f'{why} 인데 그 자세로 닿지 않는다 — {bad}')
        self.want_pose = want
        # ⚠ **Z 를 먼저 올린다.** 회전도 XY 이동도 건이 내려간 상태로 하면
        #   철근을 긁는다. 작업영역 검사는 X·Y 뿐이라 이것을 코드가 지켜야 한다.
        if self._needs_z_clear():
            return self._enter(Step.Z_CLEAR,
                               f'{why} — 회전·이동 전 Z 를 {self.z_safe:.0f}mm 로')
        return self._after_z_clear(why)

    def _needs_z_clear(self):
        if self.goal_z is None:
            return False                      # Z 를 안 쓰는 운전이다
        z = self._z()
        return z is not None and z < self.z_safe - self.z_tol

    def _after_z_clear(self, why):
        if self.want_pose == self._pose():
            # 자세가 그대로면 후퇴도 회전도 필요 없다
            return self._enter(Step.MOVE_XY, f'{why} (자세 유지) → 바로 이동')
        return self._enter(Step.RETRACT, f'{why} — 회전 전 XY 후퇴')

    def _send_z(self, z):
        self._sent = time.time()
        self.goal_pub.publish(Point(x=NAN, y=NAN, z=float(z)))

    def _z_arrived(self, z):
        cur = self._z()
        return cur is not None and abs(cur - z) <= self.z_tol

    def _do_z_clear(self):
        if self._z_arrived(self.z_safe):
            return self._after_z_clear('Z 안전높이 확보')
        if self._stage_busy():
            return
        if time.time() - self._sent > 1.0:
            self._send_z(self.z_safe)
            self.detail = f'Z 를 {self.z_safe:.0f}mm 로 올리는 중'

    def _pick_pose(self, cur):
        """지정이 없을 때 쓸 자세. **지금 자세로 닿으면 바꾸지 않는다.**

        사용자 지정 규칙이다 (2026-10-04): 자세 변경이 20초가 넘으니 갈 수 있으면
        그대로 간다. 못 가면 닿는 자세 중 **회전이 가장 짧은** 쪽으로 바꾼다.

        ⚠ 사분면 규칙은 쓰지 않는다 — 점마다 독립으로 자세를 정하므로 왕복
        회전이 생긴다. 자동 순회에서는 상위(`tying_planner`)가 자세를 지정하고,
        이 경로는 손으로 한 점만 보낼 때 쓰인다.
        """
        g = {'x': self.goal[0], 'y': self.goal[1]}
        if envelope_violation(self.env, cur, g) is None:
            return cur, f'{pose_label(cur)} 유지 (닿는다)'
        ok = [p for p in sorted(self.pose_id['poses'])
              if envelope_violation(self.env, p, g) is None]
        if not ok:
            return None, '어느 자세로도 닿지 않는다'
        want = min(ok, key=lambda p: (abs(p - cur), p))
        return want, f'{pose_label(cur)}로는 안 닿아 {pose_label(want)}로 변경'

    def _retract_target(self):
        """회전 안전창 안으로 옮길 목표. 이미 다 안에 있으면 None.

        **목표를 창에 끼워 맞춘 값**으로 보낸다 (현재 위치를 끼운 값이 아니다).
        둘 다 창 안이라 회전 안전은 같은데, 이렇게 하면 축이 **되돌아가지 않는다** —
        현재 위치를 끼우면 Y 가 300 → 275.1 → 100 처럼 왔다 갔다 한다.
        창 안으로 끼운 값은 통과하는 모든 자세의 교집합 안이므로 **현재 자세에서도
        반드시 허용된다** (현재 자세가 통과 자세에 포함되기 때문이다).

        한 축이라도 창 밖이면 두 축을 함께 보낸다. 안에 있던 축도 어차피 목표
        방향으로 가는 것이라 헛걸음이 아니고, 회전 뒤 이동이 없어질 때가 많다.
        """
        win, passed = transit_window(self.env, self.pose_id,
                                     self._pose(), self.want_pose)
        if not win:
            return None, [], None
        cur = dict(zip(('x', 'y'), self._mm()))
        if all(win[ax][0] <= cur[ax] <= win[ax][1] for ax in ('x', 'y')):
            return None, passed, win
        tgt = {ax: self._inside(g, win[ax])
               for ax, g in zip(('x', 'y'), self.goal)}
        return tgt, passed, win

    def _inside(self, v, rng):
        """창 안으로 끼우되 **경계에 붙이지 않는다** (`transit_pad_mm` 만큼 안쪽).

        창이 여유 두 배보다 좁으면 **중앙**을 쓴다 — 그때는 어느 쪽 경계에서도
        최대한 떨어지는 것이 최선이다.
        """
        lo, hi = rng
        if hi - lo <= 2 * self.pad:
            return (lo + hi) / 2.0
        return min(max(v, lo + self.pad), hi - self.pad)

    def _do_retract(self):
        """후퇴를 **끝까지 기다린 뒤** 회전으로 넘어간다.

        ⚠ 창 안에 들어간 순간 넘어가면 안 된다. `stage_node` 는 이동 중 회전
        명령을 **거부**하므로(한 번에 한 동작) 거기서 시퀀스가 깨진다.
        2026-10-03 시뮬레이션에서 그렇게 잡혔다 — 창 안에 들자마자 회전을 보내
        XY 와 yaw 가 동시에 움직였고, 실장비라면 거부로 멈췄다.
        """
        if self._retract_tgt is None:
            tgt, passed, win = self._retract_target()
            if win is None:
                return self._fail('회전 안전창을 계산할 수 없다')
            self._passed = passed
            if tgt is None:
                names = ', '.join(passed)
                return self._enter(Step.ROTATE,
                                   f'[{names}] 교집합 안 — 후퇴 불필요, 회전')
            if self._stage_busy():
                return
            self._retract_tgt = tgt
            self._sent = time.time()
            self.goal_pub.publish(Point(x=tgt['x'], y=tgt['y'], z=NAN))
            self.detail = ('후퇴 ' + ', '.join(f'{k}={v:.1f}'
                                             for k, v in sorted(tgt.items())))
            return
        if self._stage_busy() or time.time() - self._sent < 1.0:
            return
        cur = dict(zip(('x', 'y'), self._mm()))
        off = {ax: cur[ax] - v for ax, v in self._retract_tgt.items()
               if cur[ax] is None or abs(cur[ax] - v) > self.tol}
        if not off:
            names = ', '.join(self._passed)
            return self._enter(Step.ROTATE, f'후퇴 완료 — [{names}] 안에서 회전')
        # 멈췄는데 목표에 못 닿았다 — 안전 차단이나 리미트다. 다시 보내지 않는다
        return self._fail(
            '후퇴가 끝나지 않았다 ('
            + ', '.join(f'{k} {v:+.1f}mm 남음' for k, v in sorted(off.items()))
            + f") — stage_node: {self._stage_detail()}")

    def _do_rotate(self):
        if self._pose() == self.want_pose:
            return self._enter(Step.MOVE_XY,
                               f'{pose_label(self.want_pose)} 도착 → 결속 지점으로')
        if self._stage_busy():
            return
        if time.time() - self._sent > 1.5:
            self._sent = time.time()
            self.yaw_pub.publish(Int32(data=int(self.want_pose)))
            self.detail = f'{pose_label(self.want_pose)}로 회전 중'

    def _do_move_xy(self):
        x, y = self._mm()
        if (x is not None and y is not None
                and abs(x - self.goal[0]) <= self.tol
                and abs(y - self.goal[1]) <= self.tol):
            if self.goal_z is None:
                return self._finish(f'x={x:.1f} y={y:.1f}mm')
            return self._enter(Step.Z_DOWN,
                               f'결속 깊이 {self.goal_z:.1f}mm 로 하강')
        if self._stage_busy():
            return
        if time.time() - self._sent > 1.0:
            self._sent = time.time()
            self.goal_pub.publish(Point(x=self.goal[0], y=self.goal[1], z=NAN))
            self.detail = f'결속 지점으로 이동 중 ({self.goal[0]:.1f}, {self.goal[1]:.1f})'

    def _finish(self, what):
        self.step = Step.DONE
        self.detail = f'완료 — {what}, {pose_label(self.want_pose)}'
        self.get_logger().info(f"[done] {self.detail}")
        self._publish()

    def _do_z_down(self):
        if self._z_arrived(self.goal_z):
            return self._enter(Step.FIRE,
                               '결속건 작동' if self.gun_on else '결속건 꺼짐 — 건너뜀')
        if self._stage_busy():
            return
        if time.time() - self._sent > 1.0:
            self._send_z(self.goal_z)
            self.detail = f'결속 깊이 {self.goal_z:.1f}mm 로 하강 중'

    def _do_fire(self):
        """당김 → 정지 → 역방향 원복 → 정지. 2차년도 시퀀스와 같은 순서다.

        ⚠ `gun_enabled` 가 꺼져 있으면 **모터로 아무것도 보내지 않는다.** 단계는
        지나간다 — 예행연습에서 나머지 흐름을 그대로 보려는 것이다.
        """
        if not self.gun_on:
            return self._enter(Step.Z_UP, 'Z 상승')
        now = time.time()
        if self._fire_phase == 0:
            self._fire_phase, self._fire_t = 1, now
            self.gun_pub.publish(Float32(data=self.gun_speed))
            self.detail = f'결속건 당김 {self.gun_fire:.1f}s'
        elif self._fire_phase == 1 and now - self._fire_t >= self.gun_fire:
            self._fire_phase, self._fire_t = 2, now
            self.gun_pub.publish(Float32(data=0.0))
            self.detail = '결속건 정지'
        elif self._fire_phase == 2 and now - self._fire_t >= 0.2:
            self._fire_phase, self._fire_t = 3, now
            self.gun_pub.publish(Float32(data=-self.gun_speed))
            self.detail = f'결속건 원복 {self.gun_back:.1f}s'
        elif self._fire_phase == 3 and now - self._fire_t >= self.gun_back:
            self._fire_phase, self._fire_t = 4, now
            self.gun_pub.publish(Float32(data=0.0))
            self.detail = '결속건 원복 완료'
        elif self._fire_phase == 4 and now - self._fire_t >= 0.2:
            self._fire_phase = 0
            self._enter(Step.Z_UP, 'Z 상승')

    def _do_z_up(self):
        if self._z_arrived(self.z_safe):
            x, y = self._mm()
            return self._finish(f'x={x:.1f} y={y:.1f}mm z={self.z_safe:.0f}mm')
        if self._stage_busy():
            return
        if time.time() - self._sent > 1.0:
            self._send_z(self.z_safe)
            self.detail = f'Z 를 {self.z_safe:.0f}mm 로 올리는 중'

    # ---- 발행 --------------------------------------------------------------
    def _publish(self):
        x, y = self._mm()
        self.status_pub.publish(String(data=json.dumps({
            'step': self.step.value,
            'goal_mm': ({'x': self.goal[0], 'y': self.goal[1],
                         'z': self.goal_z} if self.goal else None),
            'pose_now': self._pose(),
            'pose_want': self.want_pose,
            'current_mm': {'x': x, 'y': y, 'z': self._z()},
            'gun_enabled': self.gun_on,
            'goals': self.goals,          # 늘었으면 내 목표가 접수된 것이다
            'detail': self.detail,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TyingSequence()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node._fail('노드 종료')
                time.sleep(0.1)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
