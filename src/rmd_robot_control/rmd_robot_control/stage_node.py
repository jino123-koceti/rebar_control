#!/usr/bin/env python3
"""L3 — 상부 스테이지를 **mm 목표 위치로** 보낸다.

`homing_node` 가 "어떤 순서로 원점을 찾나" 를 맡듯, 이 노드는 "축을 목표까지 어떻게
보내나" 를 맡는다 (YEAR3_ARCHITECTURE.md §4 L3). **CAN 을 모른다** — 축 명령은
토픽으로 내고, 위치는 토픽으로 받는다.

## 왜 mm 인가

교차점 검출은 카메라 좌표에서 **mm** 를 낸다. 축은 **도** 로 움직인다. 그 사이를
누군가 바꿔야 하는데, 상위(인지·미션)가 축의 감속비를 알 이유가 없다. 여기서 바꾼다.

    검출 (mm) → [stage_node] → /joint_N/position (도) → position_control_node → CAN

## 원점 기준

mm 는 **호밍 원점에서의 거리**다. 그래서 호밍이 선행 조건이다. `/homing_status` 의
`refs` 에서 축별 원점 각도를 받아 기준으로 삼는다. 원점을 모르면 이동을 거부한다 —
기준 없이 절대 위치로 보내면 어디로 갈지 알 수 없다.

## 토픽

  구독  /stage/goal      geometry_msgs/Point  목표 (x,y,z) mm. NaN 인 축은 안 움직인다
        /stage/stop      std_msgs/Empty       즉시 정지
        /homing_status   String(JSON)         원점 레퍼런스
        /safety/state    SafetyState          방향별 차단
        /control_mode    String(JSON)         제어 권한
  발행  /joint_N/position Float64MultiArray   [목표각도, 최대속도dps]
        /joint_N/speed    Float32             정지(0) 를 확실히 보낼 때
        /brake_cmd        String              축별 브레이크
        /stage/status     String(JSON)        현재 mm·목표·도달 여부

## yaw 자세를 어떻게 아는가 — **멀티턴 기준점**

⚠⚠ **단회전만으로는 못 가린다.** 단회전은 모터 1회전(건 28.8°)마다 접히고 yaw
구동범위는 1.23바퀴라, 별칭이 엉뚱한 자세 옆에 떨어지는 각도가 실제로 존재한다.
2026-10-03 에 건 +10.80° 를 **1번 자세로 읽었다**(별칭 −18.00° 가 1.26° 차이).
그 상태에서 회전을 시키면 27.6° 를 지나쳐 밀고, yaw 는 끝 리미트가 없다.
허용오차를 조여도 그런 구간이 남는다 — 원리적 한계다.

그래서 **각도를 아는 순간에 기준점을 잡고** 그 뒤로는 멀티턴 차이로 추적한다:

  · 호밍 완료 → 준비자세(`ready.yaw_pose`) 에 서 있다
  · 자세 회전 성공 → 그 자세에 서 있다

기준점이 없으면 **자세를 모르는 것으로 취급한다**(교집합 적용). 단회전 추정은
참고로만 상태에 싣는다 — 강제에는 쓰지 않는다. 전원이 꺼지면 멀티턴이 날아가
기준점도 무효이므로 재호밍이 필요하다.

## 자세별 가동 범위 (2026-10-03)

**yaw 자세에 따라 X·Y 가 갈 수 있는 거리가 다르다** — 결속건이 회전하며 간섭
방향이 바뀐다. 그래서 목표를 받으면 **지금 yaw 자세를 판별해서** 그 자세의
범위 밖이면 거부한다. 이게 없으면 검출 지점으로 보낼 때 프레임을 친다.
자세를 못 가리면(자세 사이) 네 자세의 **교집합**으로 본다 — 모르면 좁게 잡는다.

## 결속 자세 선택

결속 지점의 사분면으로 **어느 자세로 결속할지**도 같이 알려준다 (`/stage/status`
의 `pose_want`). X 가 xmax 쪽이면 1·4번, xmin 쪽이면 2·3번, Y 가 ymin 쪽이면
1·2번, ymax 쪽이면 3·4번 — 겹치면 하나로 떨어진다. 규칙은 `axes.yaml` 에 있다.
고르기만 한다 — "언제 돌릴지" 는 상위(결속 시퀀스)가 정한다.

## yaw 자세 이동 (`/stage/yaw_pose`)

**원점이 필요 없다.** 자세 판별이 현재 건 각도를 유일하게 정해 주므로 상대
이동으로 간다 (실측 도착 오차 건 +0.02°):

    목표 토픽각 = 현재 토픽각 + (현재 건각 − 목표 건각) × gear

`axis_config.ready_target()` 은 호밍이 **실제 적용한** 에지→12시 오프셋을 알아야
해서 `homing_node` 밖에서는 쓸 수 없다. 이 경로는 그 값이 필요 없다.

⚠⚠ **회전은 중간 자세를 지나간다.** 1번에서 3번으로 가면 12시와 2번을 지나므로,
XY 가 **지나가는 자세 전부의 교집합** 안에 있어야 한다. 아니면 거부한다 —
후퇴는 상위가 시킨다 (이 노드는 막는 쪽만 맡는다).

## 안전

  · `mm_per_deg` 가 `axes.yaml` 에 없으면(미측정) **이동을 거부한다.** 환산값을
    모르는 채로 움직이면 엉뚱한 거리를 간다.
  · 목표가 **현재 yaw 자세의 가동 범위 밖**이면 거부한다 (위 참조)
  · Z 는 브레이크를 풀면 떨어진다 → 이동 직전에 풀고 도달 즉시 잠근다
  · 안전 차단(`blocked_axes`)에 걸린 방향으로는 안 보낸다
  · 제어 권한이 없으면 명령하지 않는다 (mode_arbiter)
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, Float64MultiArray, Int32, String
from rebar_base_interfaces.msg import SafetyState

from .axis_config import (envelope_for, envelope_violation, identify_pose,
                          load_axis_motor_ids,
                          load_envelope, load_pose_id, load_pose_select,
                          load_ready_yaw_pose, load_stage_axes, gun_from_anchor,
                          pose_from_gun, pose_label, select_pose, transit_window)

AXES = ('x', 'y', 'z')          # yaw 는 mm 개념이 아니라 여기서 다루지 않는다


class StageNode(Node):
    def __init__(self):
        super().__init__('stage_node')

        self.declare_parameter('move_speed_dps', 30.0)
        # ⚠ **같은 dps 는 축마다 다른 mm/s 다** — 감속비가 다르다
        #   (X 0.2906 / Y 0.2227 / Z 0.0906 mm/도). 60dps 면 X 17.4 · Y 13.4 ·
        #   **Z 5.4** mm/s 가 되어 Z 가 사이클 시간을 지배했다 (2026-10-04 실측:
        #   한 점 41초 중 Z 하강·상승이 32초). `move_speed_mm_s` 를 0 보다 크게
        #   주면 축마다 환산해 **세 축을 같은 선속도**로 맞춘다.
        self.declare_parameter('move_speed_mm_s', 0.0)
        # 환산한 dps 의 상한. Z 는 mm/도가 가장 촘촘해서(0.0906) 같은 mm/s 에
        # 가장 큰 dps 를 요구한다 — 상한이 없으면 Z 만 과속한다.
        self.declare_parameter('max_speed_dps', 150.0)
        # ── 충돌 감지 ──────────────────────────────────────────────────────
        # 교차점이 아닌 곳에서 Z 를 내리면 건이 **철근을 찍는다.** 작업영역
        # 검사(X·Y)로는 막을 수 없다 — 작업영역 안이어도 그 자리에 철근이 있다.
        # 그래서 전류가 뛰면 이동을 **취소하고 반대 방향으로 물러난다.**
        # ⚠ 전류는 `0xA4` 응답에 실려 오므로 **명령을 보내는 동안만** 갱신된다.
        #   표본이 안 오면 판정하지 않는다 (없는 것을 0 으로 보면 안 된다).
        self.declare_parameter('collide_detect', True)
        # ⚠⚠ **문턱은 축마다 달라야 한다.** 2026-10-04 에 Z 기준 3.0A 를 세 축에
        #   같이 썼더니 X 가 정상 이동에서 헛걸렸다 — 실측 정상 이동 전류가
        #   X 최대 3.09A·평균 2.66A, Z 최대 1.87A·평균 0.72A 로 3배 넘게 다르다
        #   (X 는 행정이 길고 질량이 크다). 속도를 올리면 전류도 오르므로
        #   **최종 속도에서 다시 재서** 넣어야 한다.
        self.declare_parameter('collide_current_a', 3.0)        # 축별 값이 없을 때
        self.declare_parameter('collide_current_a_x', 0.0)      # 0 = 공통값 사용
        self.declare_parameter('collide_current_a_y', 0.0)
        self.declare_parameter('collide_current_a_z', 0.0)
        self.declare_parameter('collide_samples', 3)       # 연속 표본
        self.declare_parameter('collide_backoff_mm', 15.0)
        # 기동 직후에는 가속 전류가 뜬다 — 그 구간은 보지 않는다
        self.declare_parameter('collide_grace_sec', 0.7)
        self.declare_parameter('tolerance_mm', 1.0)
        self.declare_parameter('move_timeout_sec', 30.0)
        self.declare_parameter('arm_sec', 1.0)        # 브레이크 해제 후 대기
        # 중재기에서 권한을 받기까지 기다리는 시간. ⚠ 0 이면 **요청 직후 50ms 에**
        # "권한 없음" 으로 판단해 중단한다 — 리모콘을 쓴 뒤에는 모드가 manual 로
        # 돌아가 있어 항상 그렇게 된다 (2026-10-04 에 yaw 회전이 그래서 죽었다).
        self.declare_parameter('grant_wait_sec', 3.0)
        # ⚠ 끄면 프레임 충돌을 막을 것이 없다. 범위를 다시 재는 동안만 끈다.
        self.declare_parameter('enforce_envelope', True)
        self.declare_parameter('yaw_speed_dps', 60.0)     # 모터축. 실측 3.5s/208°
        self.declare_parameter('yaw_tol_deg', 3.0)        # 도달 판정 (모터축)

        g = self.get_parameter
        self.speed = float(g('move_speed_dps').value)
        self.speed_mm_s = float(g('move_speed_mm_s').value)
        self.max_dps = float(g('max_speed_dps').value)
        self.hit_on = bool(g('collide_detect').value)
        self.hit_a = float(g('collide_current_a').value)
        self.hit_a_ax = {ax: float(g(f'collide_current_a_{ax}').value)
                         for ax in ('x', 'y', 'z')}
        self.hit_n = int(g('collide_samples').value)
        self.hit_back = float(g('collide_backoff_mm').value)
        self.hit_grace = float(g('collide_grace_sec').value)
        self.cur_a = {}            # 축 → 최근 전류(A). 표본이 없으면 키가 없다
        self.hit_cnt = {}          # 축 → 문턱 초과 연속 횟수
        self.retreating = None     # 후퇴 중인 축 (그 동안 다시 판정하지 않는다)
        self.tol_mm = float(g('tolerance_mm').value)
        self.timeout = float(g('move_timeout_sec').value)
        self.arm_sec = float(g('arm_sec').value)
        self.grant_wait = float(g('grant_wait_sec').value)
        self.enforce = bool(g('enforce_envelope').value)
        self.yaw_speed = float(g('yaw_speed_dps').value)
        self.yaw_tol = float(g('yaw_tol_deg').value)

        # yaw 도 같이 싣는다 — 관절 발행·브레이크·위치 구독 경로를 공유한다.
        # mm 환산이 없는 축이라 `mm_of`/`deg_of` 는 AXES 에만 쓴다.
        self.ax = load_stage_axes(AXES + ('yaw',))
        missing = [n for n, c in self.ax.items()
                   if n in AXES and c['mm_per_deg'] is None]
        if missing:
            self.get_logger().warning(
                f"mm_per_deg 미측정: {', '.join(missing)} — 그 축은 이동을 거부합니다. "
                f"tools/test/stage_scale_measure.py 로 재서 axes.yaml 에 적으세요")

        self.pos_pubs = {n: self.create_publisher(
            Float64MultiArray, f"/joint_{c['joint']}/position", 10)
            for n, c in self.ax.items() if c['joint']}
        self.spd_pubs = {n: self.create_publisher(
            Float32, f"/joint_{c['joint']}/speed", 10)
            for n, c in self.ax.items() if c['joint']}
        self.brake_pub = self.create_publisher(String, '/brake_cmd', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode_request', 10)
        self.status_pub = self.create_publisher(String, '/stage/status', 10)

        self.deg = {n: None for n in self.ax}     # 현재 모터각(도). yaw 포함
        for n, c in self.ax.items():
            if c['motor']:
                self.create_subscription(
                    Float32, f"/motor_{c['motor']}_position",
                    (lambda k: (lambda m: self.deg.__setitem__(k, m.data)))(n), 10)

        self.refs = {}                            # 호밍 원점(도)
        self.create_subscription(String, '/homing_status', self._on_homing, 10)
        self.safety = None
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 1)
        self.mode = None
        self.create_subscription(String, '/control_mode', self._on_mode, 10)
        self.create_subscription(Point, '/stage/goal', self._on_goal, 10)
        # 각도 목표. 캘리브레이션이 카메라→**축 각도** 로 바로 맞춰지면 mm 환산이
        # 필요 없다 (변환이 환산까지 흡수한다). mm_per_deg 실측 전에도 쓸 수 있다.
        self.create_subscription(Point, '/stage/goal_deg', self._on_goal_deg, 10)
        self.create_subscription(Empty, '/stage/stop', lambda m: self._stop('정지 명령'), 10)

        # ── 자세별 가동 범위 ────────────────────────────────────────────
        self.env = load_envelope()
        self.sel = load_pose_select()
        self.pose_id = load_pose_id('yaw')
        self.yaw_single = None          # yaw 단회전값 (참고용 — 별칭이 있다)
        self.yaw_multi = None           # yaw 멀티턴 (기준점과의 차이로 각도를 낸다)
        self.yaw_anchor = None          # (멀티턴, 건각) — 각도를 아는 순간에 잡는다
        self.yaw_anchor_src = ''
        self._homing_state = None       # 전이 판정용 (None = 아직 못 봤다)
        self.ready_pose = load_ready_yaw_pose()
        self.yaw_brake = None           # 0x9A DATA[3] — 해제 확인 전엔 안 움직인다
        yaw_mid = load_axis_motor_ids(('yaw',)).get('yaw')
        if yaw_mid:
            self.create_subscription(
                Int32, f"/motor_{yaw_mid}/encoder_single",
                lambda m: setattr(self, 'yaw_single', m.data), 10)
            self.create_subscription(
                Int32, f"/motor_{yaw_mid}/encoder_multi",
                lambda m: setattr(self, 'yaw_multi', m.data), 10)
            self.create_subscription(
                Bool, f"/motor_{yaw_mid}/brake",
                lambda m: setattr(self, 'yaw_brake', m.data), 10)
        for _n, _c in self.ax.items():
            if _c.get('motor') and _c.get('mm_per_deg'):
                self.create_subscription(
                    # ⚠ `motor` 는 이미 "0x147" 꼴의 **문자열**이다 (axis_config
                    #   가 그렇게 만든다) — hex() 를 씌우면 TypeError 로 노드가
                    #   아예 못 뜬다. 위 위치 토픽도 같은 형식을 쓴다.
                    Float32, f"/motor_{_c['motor']}/current",
                    (lambda nm: (lambda m: self.cur_a.__setitem__(nm, float(m.data))))(_n),
                    10)
        self.create_subscription(Int32, '/stage/yaw_pose', self._on_yaw_pose, 10)
        # 사람이 "지금 눈으로 보니 N번 자세다" 를 알려주는 경로. 재시작으로 기준점을
        # 잃었을 때 재호밍(64초) 없이 복구한다. ⚠ 사람이 틀리면 그대로 틀린다 —
        # 자동으로는 절대 보내지 않는다 (ros2 topic pub 전용).
        self.create_subscription(Int32, '/stage/yaw_declare', self._on_declare, 10)
        self.yaw_moving = False
        self.yaw_tgt = None
        self.yaw_want = None
        self._yaw_sent = 0.0
        if self.env is None:
            self.get_logger().warning(
                "자세별 가동 범위가 없다 (axes.yaml 의 stage.envelope) — "
                "범위 검사를 하지 않는다. 자세에 따라 프레임을 칠 수 있다")
        elif not self.enforce:
            self.get_logger().warning(
                "enforce_envelope=false — 범위 밖 목표를 막지 않는다")

        self.target = {}          # 축 → 목표값
        self.target_unit = 'mm'   # 'mm' 또는 'deg' — 목표가 어느 단위인가
        self.t_start = 0.0
        self.t_arm = 0.0
        self.moving = False
        self.rejects = 0          # 거부 횟수. 상위가 "내 명령이 거부됐나" 를 가린다
        self.detail = '대기'

        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            "스테이지 노드 시작 — /stage/goal (mm) 로 목표를 준다. "
            "mm 는 호밍 원점 기준이다")

    # ---- 입력 --------------------------------------------------------------
    def _on_homing(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        refs = d.get('refs') or {}
        for k, v in refs.items():
            if v is not None:
                self.refs[k] = float(v)
        # 호밍이 끝나면 yaw 는 준비자세에 서 있다 — **각도를 아는 순간**이다.
        # ⚠ `done` 은 그 뒤로도 계속 발행된다(상태이지 사건이 아니다). 보이는
        #   대로 잡으면, 호밍 뒤 yaw 가 움직인 다음 재시작한 노드가 "지금 준비
        #   자세" 로 **틀리게** 고정한다. 그래서 **전이에서만** 잡는다. 첫 관측은
        #   이전 상태를 모르므로 잡지 않는다 — 그때는 기준점 없음(= 모름)이 맞다.
        st = d.get('state')
        prev, self._homing_state = self._homing_state, st
        if (st == 'done' and prev not in (None, 'done') and 'yaw' in refs
                and self.ready_pose is not None and self.yaw_multi is not None):
            self._anchor(self.ready_pose, '호밍 완료',
                         gun=self._measured_gun(self.ready_pose))

    def _on_safety(self, msg):
        self.safety = msg

    def _on_mode(self, msg):
        try:
            self.mode = (json.loads(msg.data) or {}).get('mode')
        except ValueError:
            pass

    # ---- 자세 --------------------------------------------------------------
    def _anchor(self, pose, why, gun=None):
        """그 자세에 서 있다고 **아는** 순간에 기준점을 잡는다.

        ⚠ `gun` 이 있으면 **그 실측 각도**를 쓴다. 자세의 공칭 각도를 박으면
        허용오차(±1.5°)만큼 틀어진 채로 고정되고, 그 뒤 모든 각도가 그만큼
        틀린다 — 2026-10-03 에 12시에서 0.84° 어긋났다. 단회전이 유일하게
        갈리면 각도를 **정확히** 알 수 있으므로 그 값을 쓰는 것이 맞다.
        (`yaw_single` 과 `yaw_multi` 는 같은 0x61 응답에서 나와 서로 맞는다 —
         움직이는 중에 잡아도 어긋나지 않는다.)
        """
        g = gun if gun is not None else (
            ((self.pose_id or {}).get('poses') or {}).get(pose, {}).get('gun'))
        if g is None or self.yaw_multi is None:
            return
        same = (self.yaw_anchor is not None
                and abs(self.yaw_anchor[0] - self.yaw_multi) < 50
                and abs(self.yaw_anchor[1] - g) < 0.01)
        self.yaw_anchor = (self.yaw_multi, float(g))
        self.yaw_anchor_src = why
        if not same:
            self.get_logger().info(
                f"yaw 기준점 — {pose_label(pose)} (건 {g:+.2f}°, "
                f"멀티턴 {self.yaw_multi}) · {why}")

    def _on_declare(self, msg):
        """사람이 현재 자세를 선언한다 → 기준점으로 잡는다."""
        want = int(msg.data)
        if ((self.pose_id or {}).get('poses') or {}).get(want) is None:
            return self._reject(f'모르는 자세 {want}')
        if self.yaw_multi is None:
            return self._reject('yaw 멀티턴을 못 받고 있다')
        self._anchor(want, '사람이 선언')
        self._publish_status()

    def _measured_gun(self, pose):
        """단회전이 유일하게 갈리고 그 자세와 맞으면 **실측 각도**, 아니면 None.

        공칭 각도보다 정확하다 (위 `_anchor` 참고). 자세가 다르게 나오면 쓰지
        않는다 — 엉뚱한 각도를 기준점으로 박는 것이 더 나쁘다.
        """
        got, info = identify_pose(self.pose_id, self.yaw_single)
        if got == pose and isinstance(info, dict):
            return info.get('gun_now')
        return None

    def gun_now(self):
        """지금 건 각도 (기준점 기준). 기준점이 없으면 None."""
        return gun_from_anchor(self.pose_id, self.yaw_anchor, self.yaw_multi)

    def cur_pose(self):
        """지금 yaw 자세. (번호|None, 설명).

        **기준점이 없으면 모르는 것으로 둔다** — 단회전 추정은 별칭 때문에
        틀릴 수 있어 강제에 쓰지 않는다 (모듈 문서 참고).
        """
        g = self.gun_now()
        if g is None:
            # 기준점이 없어도 **단회전이 유일하게 갈리면** 그걸로 잡는다.
            # `identify_pose` 가 후보를 구동범위로 걸러 모호하면 거부하므로,
            # 성공한 결과는 별칭이 없다 (12시·2번·3번). 1번·4번은 거부된다.
            got, det = identify_pose(self.pose_id, self.yaw_single)
            if got is None:
                return None, f"yaw 자세를 모른다 — {det}"
            # 공칭 각도가 아니라 **단회전에서 나온 실측 각도**로 잡는다
            self._anchor(got, '단회전 (별칭 없음)', gun=det.get('gun_now'))
            g = self.gun_now()
            if g is None:
                return None, 'yaw 멀티턴을 못 받고 있다'
            return got, f"{pose_label(got)} (건 {g:+.2f}°, 단회전으로 확정)"
        pose, detail = pose_from_gun(self.pose_id, g)
        if pose is None:
            return None, str(detail)
        return pose, f"{pose_label(pose)} (건 {g:+.2f}°, 오차 {detail['err_gun']:+.2f}°)"

    def _envelope_check(self, want_mm):
        """목표 mm 가 현재 자세의 가동 범위 안인가. 밖이면 사유, 안이면 None.

        자세를 못 가리면 교집합으로 본다 — 모르면 좁게 잡는 쪽이 안전하다.
        """
        if self.env is None or not self.enforce:
            return None
        pose, why = self.cur_pose()
        bad = envelope_violation(self.env, pose, want_mm)
        if bad is None:
            return None
        hint = ''
        if self.sel:
            w, _ = select_pose(self.sel, want_mm.get('x'), want_mm.get('y'), pose)
            if w is not None and w != pose:
                ok = envelope_violation(self.env, w, want_mm) is None
                hint = (f" → {pose_label(w)}로 돌리면 "
                        f"{'갈 수 있다' if ok else '역시 범위 밖이다'}")
        return f"{bad} [지금 {why}]{hint}"

    def _on_yaw_pose(self, msg):
        """yaw 를 1~4번 자세(또는 0=12시)로 돌린다. 상대 이동이라 원점이 필요 없다."""
        want = int(msg.data)
        info = self.pose_id
        if self.moving or self.yaw_moving:
            return self._reject('이미 이동 중이다 — /stage/stop 후 다시')
        if not info or want not in info['poses']:
            return self._reject(f'모르는 자세 {want} '
                                f'(axes.yaml 의 pose_offset_from_noon_gun_deg)')
        c = self.ax.get('yaw') or {}
        if not c.get('joint') or self.deg.get('yaw') is None:
            return self._reject('yaw: 축 정의나 현재 위치가 없다')
        cur, why = self.cur_pose()
        if cur is None:
            return self._reject(f'현재 yaw 자세를 못 가린다 — {why}')
        if cur == want:
            # 한도 때문에 지웠다가 되돌림 (2026-10-03) — 상태에도 남아야 상위가 본다
            self.detail = f'{pose_label(want)} — 이미 그 자세다'
            self.get_logger().info(self.detail)
            return self._publish_status()
        stop = self._safety_stop()
        if stop:
            return self._reject(stop)

        bad = self._transit_block(cur, want)
        if bad:
            return self._reject(bad)

        g = info['poses']
        # 기동 때 한 번만 읽으면 런타임 변경이 안 먹는다 — 명령마다 다시 읽는다
        self.yaw_speed = float(self.get_parameter('yaw_speed_dps').value)
        self.yaw_want = want
        # ⚠ 현재 건 각도는 **기준점**에서 낸다. 위치 토픽(0x92)은 멈추면 묵고,
        #   단회전은 별칭이 있다 — 둘 다 여기서 쓰면 안 된다.
        self.yaw_tgt = (self.deg['yaw']
                        + (self.gun_now() - g[want]['gun']) * info['gear'])
        self.yaw_moving = True
        self.t_start = self.t_arm = time.time()
        self._yaw_sent = 0.0
        self.detail = f'{pose_label(cur)} → {pose_label(want)} — 브레이크 해제 대기'
        self._brake('release', 'yaw')
        self._request_control('auto')
        self.get_logger().info(
            f"yaw 회전 — {pose_label(cur)} → {pose_label(want)} "
            f"(건 {g[cur]['gun']:+.2f}° → {g[want]['gun']:+.2f}°, "
            f"모터축 {self.yaw_tgt - self.deg['yaw']:+.1f}°)")

    def _transit_block(self, cur, want):
        """회전 중 지나가는 자세들의 교집합 밖이면 사유, 안이면 None.

        회전은 **중간 자세를 지나간다.** 양 끝만 보면 12시처럼 사이에 있는
        최악값을 놓친다 (2026-10-03: X 상한 22mm 과했다).
        """
        if self.env is None or not self.enforce:
            return None
        win, passed = transit_window(self.env, self.pose_id, cur, want)
        if not win:
            return None
        names = ', '.join(passed)        # transit_window 가 라벨로 돌려준다
        for ax in ('x', 'y'):
            v = self.mm_of(ax)
            if v is None:
                return (f'{ax}: 현재 mm 를 몰라 회전 안전을 확인할 수 없다 '
                        f'(호밍 원점과 mm_per_deg 가 있어야 한다)')
            if not (win[ax][0] <= v <= win[ax][1]):
                return (f'{pose_label(cur)}→{pose_label(want)} 회전은 [{names}] 를 '
                        f'지난다 — 지금 {ax}={v:.1f}mm 가 그 교집합 '
                        f'{win[ax][0]:.1f}~{win[ax][1]:.1f}mm 밖이다. '
                        f'XY 를 먼저 그 안으로 옮기세요')
        return None

    def _yaw_tick(self):
        stop = self._safety_stop()
        if stop:
            return self._yaw_done(f'안전 — {stop}')
        if not self._granted():
            if self._wait_grant():
                return                        # 승인 대기 — 요청은 계속 보낸다
            return self._yaw_done(f'제어 권한 없음 (모드 {self.mode})')
        if time.time() - self.t_start > self.timeout:
            return self._yaw_done(f'타임아웃 {self.timeout:.0f}s')
        # 해제가 명령보다 **먼저 도착해야** 한다. DATA[3] 로 확인한다 —
        # 안 풀린 채로 명령하면 전류만 오른다 (0.4초로는 부족한 적이 있다)
        if not self.yaw_brake or time.time() - self.t_arm < self.arm_sec:
            self._brake('release', 'yaw')
            return
        self._request_control('auto')
        err = self.yaw_tgt - self.deg['yaw']
        if abs(err) <= self.yaw_tol:
            return self._yaw_done(f'도달 (남은 모터축 {err:+.2f}°)')
        if time.time() - self._yaw_sent > 0.4:
            self._yaw_sent = time.time()
            self.pos_pubs['yaw'].publish(
                Float64MultiArray(data=[self.yaw_tgt, self.yaw_speed]))
            g = self.gun_now()
            self.detail = (f"{pose_label(self.yaw_want)}로 회전 중"
                           + (f" (건 {g:+.2f}°, 남은 모터축 {err:+.1f}°)"
                              if g is not None else ''))

    def _yaw_done(self, reason):
        """멈추고 **잠그고 0x80 까지** 보낸다 — 0x78 만으로는 전류가 계속 흐른다."""
        self.spd_pubs['yaw'].publish(Float32(data=0.0))
        self._brake('lock', 'yaw')
        self._brake('shutdown', 'yaw')
        # 도달했으면 거기가 곧 기준점이다 — 누적 오차를 끊는다
        if self.yaw_tgt is not None and self.deg.get('yaw') is not None \
                and abs(self.yaw_tgt - self.deg['yaw']) <= self.yaw_tol:
            self._anchor(self.yaw_want, '자세 회전 성공',
                         gun=self._measured_gun(self.yaw_want))
        got, why = self.cur_pose()
        ok = got is not None and got == self.yaw_want
        self.detail = (f'yaw {pose_label(self.yaw_want)} — {reason}'
                       + ('' if ok else f' ⚠ 도착 자세 {why}'))
        # ⚠ `(logger.info if ok else logger.warning)(msg)` 로 쓰면 안 된다 —
        #   rclpy 는 **호출 지점별로** 로깅 상태를 캐시해서, 한 줄에서 심각도를
        #   바꿔 부르면 `ValueError: Logger severity cannot be changed between
        #   calls.` 로 죽는다. 2026-10-03 에 회전이 끝날 때마다 노드가 죽었다.
        if ok:
            self.get_logger().info(self.detail)
        else:
            self.get_logger().warning(self.detail)
        self.yaw_moving = False
        self.yaw_tgt = None
        self._request_control('release')
        self._publish_status()

    # ---- 환산 --------------------------------------------------------------
    def mm_of(self, name):
        """현재 위치를 원점 기준 mm 로."""
        c = self.ax[name]
        d, ref, k = self.deg[name], self.refs.get(name), c['mm_per_deg']
        if d is None or ref is None or not k:
            return None
        return (d - ref) * float(k)

    def deg_of(self, name, mm):
        c = self.ax[name]
        return self.refs[name] + float(mm) / float(c['mm_per_deg'])

    # ---- 명령 --------------------------------------------------------------
    def _reject(self, why):
        """거부하고 **카운터를 올린다.**

        ⚠ `detail` 은 다음 일이 생길 때까지 남는다(끈적하다). 상위가 그 문자열만
        보고 "내 명령이 거부됐다" 고 판단하면 **한참 전의 거부**에 걸린다 —
        2026-10-03 실장비에서 시퀀스가 시작 즉시 중단됐다. 그래서 상위는 이
        카운터가 **늘었는지**로 본다.
        """
        self.rejects += 1
        self.get_logger().error(f"이동 거부 — {why}")
        self.detail = f"거부: {why}"
        self._publish_status()

    def _unpack(self, msg):
        want = {}
        for name, v in zip(AXES, (msg.x, msg.y, msg.z)):
            if v is None or math.isnan(v):
                continue
            want[name] = float(v)
        return want

    def _on_goal_deg(self, msg):
        """목표를 **모터 각도(절대)** 로 받는다.

        mm 과 달리 원점 레퍼런스도 mm_per_deg 도 필요 없다 — 모터의 절대 각도이기
        때문이다. 캘리브레이션이 카메라→축 각도로 바로 맞춰진 경우에 쓴다.
        ⚠ 다만 "지금 어디까지 갈 수 있는가" 는 리미트 센서와 안전계층이 본다.
        """
        want = self._unpack(msg)
        if not want:
            return self._reject("목표가 비어 있다 (움직일 축은 NaN 이 아니어야 한다)")
        for name in want:
            c = self.ax[name]
            if not c['joint'] or not c['motor']:
                return self._reject(f"{name}: axes.yaml 에 축 정의가 없다")
            if self.deg[name] is None:
                return self._reject(f"{name}: 현재 위치를 못 받고 있다")
        # 각도 목표도 mm 로 환산되면 같이 검사한다. 원점·환산값이 없으면 못 한다 —
        # 캘리브레이션 도구용 경로라 그때는 통과시킨다 (사람이 보며 쓰는 경로다).
        mm = {n: (want[n] - self.refs[n]) * self.ax[n]['mm_per_deg']
              for n in ('x', 'y')
              if n in want and n in self.refs and self.ax[n]['mm_per_deg']}
        bad = self._envelope_check(mm) if mm else None
        if bad:
            return self._reject(bad)
        self._begin(want, 'deg')

    def _on_goal(self, msg):
        want = self._unpack(msg)
        if not want:
            self._reject("목표가 비어 있다 (움직일 축은 NaN 이 아니어야 한다)")
            return

        for name in want:
            c = self.ax[name]
            if not c['joint'] or not c['motor']:
                return self._reject(f"{name}: axes.yaml 에 축 정의가 없다")
            # 원점을 먼저 본다 — 둘 다 없을 때 "환산값이 없다" 고만 알리면
            # 정작 호밍을 안 했다는 사실이 가려진다 (진단이 어려워진다)
            if name not in self.refs:
                return self._reject(
                    f"{name}: 호밍 원점이 없다 — 먼저 호밍하세요 (/homing_cmd)")
            if not c['mm_per_deg']:
                return self._reject(
                    f"{name}: mm_per_deg 가 미측정이다 — 환산값 없이는 움직일 수 없다. "
                    f"각도로 주려면 /stage/goal_deg 를 쓰세요")
            if self.deg[name] is None:
                return self._reject(f"{name}: 현재 위치를 못 받고 있다")

        # 자세별 가동 범위. X·Y 만 재어 두었다 (Z 는 [미측정])
        bad = self._envelope_check({k: v for k, v in want.items() if k in ('x', 'y')})
        if bad:
            return self._reject(bad)

        self._begin(want, 'mm')

    def _begin(self, want, unit):
        stop = self._safety_stop()
        if stop:
            return self._reject(stop)
        # 기동 때 한 번만 읽으면 런타임·런치 변경이 안 먹는다 (respawn 은 기동 당시
        # 인자를 다시 쓴다). 이동마다 다시 읽어 `ros2 param set` 으로 조정 가능하게.
        g = self.get_parameter
        self.speed = float(g('move_speed_dps').value)
        self.speed_mm_s = float(g('move_speed_mm_s').value)
        self.max_dps = float(g('max_speed_dps').value)
        self.timeout = float(g('move_timeout_sec').value)
        self.hit_on = bool(g('collide_detect').value)
        self.hit_a = float(g('collide_current_a').value)
        self.hit_a_ax = {ax: float(g(f'collide_current_a_{ax}').value)
                         for ax in ('x', 'y', 'z')}

        self.target = want
        self.target_unit = unit
        self.t_start = time.time()
        self.t_arm = time.time()
        self.moving = True
        self.detail = '브레이크 해제 대기'
        for name in want:
            self._brake('release', name)
        self._request_control('auto')
        self.get_logger().info(
            "이동 시작 — " + ", ".join(f"{k}={v:+.1f}{unit}" for k, v in want.items()))

    def _safety_stop(self):
        s = self.safety
        if s is None:
            return None
        if s.estop:
            return '비상정지'
        if s.stop_switch:
            return 'STOP 스위치'
        if s.inputs_stale:
            return '안전 입력 두절'
        return None

    def _hit(self, name):
        """그 축이 **무언가에 닿았는가.** 전류가 문턱을 연속으로 넘으면 참.

        ⚠ 전류 표본은 `0xA4` 응답으로만 온다 — 명령을 보내는 동안만 갱신된다.
          표본이 아예 없으면 **판정하지 않는다** (없는 것을 0 으로 보면 보호가
          조용히 꺼진 것과 같다).
        ⚠ 기동 직후 `collide_grace_sec` 은 보지 않는다 — 가속 전류가 뜬다.
        """
        if not self.hit_on or self.retreating is not None:
            return False
        if time.time() - self.t_arm < self.arm_sec + self.hit_grace:
            return False
        a = self.cur_a.get(name)
        if a is None:
            return False
        if abs(a) < self._hit_limit(name):
            self.hit_cnt[name] = 0
            return False
        self.hit_cnt[name] = self.hit_cnt.get(name, 0) + 1
        return self.hit_cnt[name] >= self.hit_n

    def _hit_limit(self, name):
        """그 축의 충돌 문턱(A). 축별 값이 0 이면 공통값을 쓴다."""
        return self.hit_a_ax.get(name) or self.hit_a

    def _retreat(self, name, err):
        """충돌 — 이동을 **전부 취소하고** 그 축만 반대 방향으로 물러난다.

        err = 목표 − 현재 이므로 가던 방향은 sign(err) 다. 반대로 `backoff` 만큼
        간다. 작업영역 밖으로 나가지 않게 자른다 (Z 는 작업영역 표가 없다).
        """
        a = self.cur_a.get(name)
        cur = self.mm_of(name)
        self.rejects += 1
        for n2 in list(self.target):
            self.spd_pubs[n2].publish(Float32(data=0.0))
        self.target.clear()
        if cur is None:
            self._brake('lock', name)
            self.detail = f'충돌: {name} 전류 {a:.2f}A — 현재 위치를 몰라 후퇴 못 함'
            self.get_logger().error(self.detail)
            return self._stop(self.detail)
        back = cur - math.copysign(self.hit_back, err)
        r, _why = envelope_for(self.env, self.cur_pose()[0])
        if r and name in r:
            lo, hi = r[name]
            back = min(max(back, lo), hi)
        self.hit_cnt[name] = 0
        self.retreating = name
        self.get_logger().error(
            f"충돌 감지 — {name} 전류 {a:.2f}A (문턱 {self._hit_limit(name):.1f}A, "
            f"{self.hit_n}회 연속). 이동 취소하고 {cur:.1f} → {back:.1f}mm 로 후퇴")
        self._begin({name: back}, 'mm')
        self.detail = f'충돌 후퇴 — {name} {back:.1f}mm (전류 {a:.2f}A)'

    def _speed_for(self, name):
        """그 축에 보낼 최대속도(모터축 dps).

        `move_speed_mm_s` 가 0 보다 크면 **선속도 기준**으로 환산한다 — 축마다
        mm/도가 달라서 같은 dps 로는 Z 가 X 의 1/3 속도가 된다. 환산값은
        `max_speed_dps` 로 자른다.
        """
        if self.speed_mm_s <= 0:
            return self.speed
        mmpd = (self.ax.get(name) or {}).get('mm_per_deg')
        if not mmpd:
            return self.speed            # 환산값이 없는 축(yaw)은 그대로
        return min(self.speed_mm_s / mmpd, self.max_dps)

    def _blocked(self, name, direction):
        s = self.safety
        if s is None:
            return False
        return f"{name}{'+' if direction > 0 else '-'}" in s.blocked_axes

    def _brake(self, action, name):
        arg = f"{action} {name}" + (' force' if action == 'release'
                                    and self.ax[name]['gravity'] else '')
        self.brake_pub.publish(String(data=arg))

    def _request_control(self, what):
        self.mode_pub.publish(String(data=f"stage_node {what}"))

    def _granted(self):
        return self.mode is None or self.mode == 'auto'

    def _wait_grant(self):
        """권한이 없을 때 **기다릴 것인가**. 기다릴 동안 요청을 계속 보낸다.

        중재기는 "아무도 안 잡고 있을 때만" 넘겨주고, 리모콘을 쓴 뒤에는 모드가
        `manual` 이다. 요청과 승인 사이에 왕복이 있으므로 그 틈을 기다려야 한다 —
        안 기다리면 명령이 **항상** 권한 없음으로 죽는다.
        """
        if self._granted():
            return False
        self._request_control('auto')
        return (time.time() - self.t_start) < self.grant_wait

    def _stop(self, reason):
        self.retreating = None
        self.hit_cnt.clear()
        if self.yaw_moving:
            return self._yaw_done(reason)
        for name in list(self.target):
            self.spd_pubs[name].publish(Float32(data=0.0))
        for name in list(self.target):
            self._brake('lock', name)
        if self.moving:
            self.get_logger().info(f"이동 종료 — {reason}")
        self.moving = False
        self.target = {}
        self.detail = reason
        self._request_control('release')
        self._publish_status()

    # ---- 진행 --------------------------------------------------------------
    def tick(self):
        if self.yaw_moving:
            return self._yaw_tick()
        if not self.moving:
            return
        stop = self._safety_stop()
        if stop:
            return self._stop(f"안전 — {stop}")
        if not self._granted():
            if self._wait_grant():
                return                        # 승인 대기 — 요청은 계속 보낸다
            return self._stop(f"제어 권한 없음 (모드 {self.mode})")
        if time.time() - self.t_start > self.timeout:
            return self._stop(f"타임아웃 {self.timeout:.0f}s")

        # 브레이크 해제가 명령보다 먼저 도착해야 한다 (homing_node 와 같은 이유)
        if time.time() - self.t_arm < self.arm_sec:
            for name in self.target:
                self._brake('release', name)
            return
        self._request_control('auto')          # 하트비트

        done = []
        for name, goal in self.target.items():
            if self.target_unit == 'deg':
                cur, goal_deg = self.deg[name], goal
                tol = self.tol_mm / (self.ax[name]['mm_per_deg'] or 0.05)
            else:
                cur, goal_deg = self.mm_of(name), self.deg_of(name, goal)
                tol = self.tol_mm
            if cur is None:
                continue
            err = goal - cur
            if abs(err) <= tol:
                done.append(name)
                continue
            if self._blocked(name, err):
                self.get_logger().warning(f"{name}: 안전 차단으로 그 방향 이동 불가")
                done.append(name)
                continue
            if self._hit(name):
                return self._retreat(name, err)
            self.pos_pubs[name].publish(
                Float64MultiArray(data=[goal_deg, self._speed_for(name)]))

        for name in done:
            self.spd_pubs[name].publish(Float32(data=0.0))
            self._brake('lock', name)
            self.target.pop(name, None)
        if not self.target:
            self._stop('목표 도달')

    def _publish_status(self):
        cur = {n: (round(v, 2) if v is not None else None)
               for n, v in ((k, self.mm_of(k)) for k in AXES)}
        pose, pose_detail = self.cur_pose()
        # 지금 위치에서 결속한다면 어느 자세여야 하는가. 상위가 이걸 보고 yaw 를 돌린다
        want, want_why = (select_pose(self.sel, cur['x'], cur['y'], pose)
                          if self.sel else (None, '선택 규칙 없음'))
        # ⚠ 강제(`envelope_violation`)와 **같은 함수**로 구한다. 따로 계산했다가
        # 12시에서 표시만 빈 값이 나왔다 (2026-10-03)
        r, lim_why = envelope_for(self.env, pose)
        # inf 는 JSON 으로 못 내보낸다(표준이 아니다) → 제약 없음은 null 로
        lim = ({k: [None if v[0] == float('-inf') else round(v[0], 1),
                    None if v[1] == float('inf') else round(v[1], 1)]
                for k, v in r.items()} if r else None)
        self.status_pub.publish(String(data=json.dumps({
            'moving': self.moving or self.yaw_moving,
            'yaw_moving': self.yaw_moving,
            'yaw_pose_target': self.yaw_want if self.yaw_moving else None,
            'current_mm': cur,
            'target': {k: round(v, 2) for k, v in self.target.items()},
            'target_unit': self.target_unit,
            'current_deg': {k: (round(v, 2) if v is not None else None)
                            for k, v in self.deg.items()},
            'homed': sorted(self.refs),
            'pose': pose,                 # 지금 yaw 자세 (None = 모름/자세 사이)
            'pose_detail': pose_detail,
            'gun_deg': (round(self.gun_now(), 2)
                        if self.gun_now() is not None else None),
            'yaw_anchor': self.yaw_anchor_src or None,
            'pose_want': want,            # 지금 위치에서 결속할 자세
            'pose_want_why': want_why,
            'limit_mm': lim,              # 지금 자세에서 갈 수 있는 X·Y 범위
            'limit_why': lim_why,         # 그 범위가 어디서 왔는가
            'enforce_envelope': self.enforce,
            'rejects': self.rejects,      # 늘었으면 **방금** 거부된 것이다
            'detail': self.detail,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StageNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node._stop('노드 종료')
                time.sleep(0.1)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
