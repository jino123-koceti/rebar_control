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

## 자세별 가동 범위 (2026-10-03)

**yaw 자세에 따라 X·Y 가 갈 수 있는 거리가 다르다** — 결속건이 회전하며 간섭
방향이 바뀐다. 그래서 목표를 받으면 **지금 yaw 자세를 판별해서** 그 자세의
범위 밖이면 거부한다. 이게 없으면 검출 지점으로 보낼 때 프레임을 친다.
자세를 못 가리면(자세 사이) 네 자세의 **교집합**으로 본다 — 모르면 좁게 잡는다.

## 결속 자세 선택

결속 지점의 사분면으로 **어느 자세로 결속할지**도 같이 알려준다 (`/stage/status`
의 `pose_want`). X 가 xmax 쪽이면 1·4번, xmin 쪽이면 2·3번, Y 가 ymin 쪽이면
1·2번, ymax 쪽이면 3·4번 — 겹치면 하나로 떨어진다. 규칙은 `axes.yaml` 에 있다.
⚠ **이 노드는 yaw 를 돌리지 않는다.** 고를 뿐이고, 회전은 상위(결속 시퀀스)가
`pose_want` 를 보고 시킨다.

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
from std_msgs.msg import Empty, Float32, Float64MultiArray, Int32, String
from rebar_base_interfaces.msg import SafetyState

from .axis_config import (envelope_violation, identify_pose, load_axis_motor_ids,
                          load_envelope, load_pose_id, load_pose_select,
                          load_stage_axes, pose_label, select_pose)

AXES = ('x', 'y', 'z')          # yaw 는 mm 개념이 아니라 여기서 다루지 않는다


class StageNode(Node):
    def __init__(self):
        super().__init__('stage_node')

        self.declare_parameter('move_speed_dps', 30.0)
        self.declare_parameter('tolerance_mm', 1.0)
        self.declare_parameter('move_timeout_sec', 30.0)
        self.declare_parameter('arm_sec', 1.0)        # 브레이크 해제 후 대기
        # ⚠ 끄면 프레임 충돌을 막을 것이 없다. 범위를 다시 재는 동안만 끈다.
        self.declare_parameter('enforce_envelope', True)

        g = self.get_parameter
        self.speed = float(g('move_speed_dps').value)
        self.tol_mm = float(g('tolerance_mm').value)
        self.timeout = float(g('move_timeout_sec').value)
        self.arm_sec = float(g('arm_sec').value)
        self.enforce = bool(g('enforce_envelope').value)

        self.ax = load_stage_axes(AXES)
        missing = [n for n, c in self.ax.items() if c['mm_per_deg'] is None]
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

        self.deg = {n: None for n in AXES}        # 현재 모터각(도)
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
        self.yaw_single = None          # yaw 단회전값 (자세 판별용)
        yaw_mid = load_axis_motor_ids(('yaw',)).get('yaw')
        if yaw_mid:
            self.create_subscription(
                Int32, f"/motor_{yaw_mid}/encoder_single",
                lambda m: setattr(self, 'yaw_single', m.data), 10)
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

    def _on_safety(self, msg):
        self.safety = msg

    def _on_mode(self, msg):
        try:
            self.mode = (json.loads(msg.data) or {}).get('mode')
        except ValueError:
            pass

    # ---- 자세 --------------------------------------------------------------
    def cur_pose(self):
        """지금 yaw 자세. (번호|None, 설명). None 이면 자세 사이거나 값이 없다."""
        pose, detail = identify_pose(self.pose_id, self.yaw_single)
        return pose, (pose_label(pose) if pose is not None else str(detail))

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
        mm = {}
        for name in ('x', 'y'):
            if name in want and name in self.refs and self.ax[name]['mm_per_deg']:
                mm[name] = (want[name] - self.refs[name]) * self.ax[name]['mm_per_deg']
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

    def _stop(self, reason):
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
        if not self.moving:
            return
        stop = self._safety_stop()
        if stop:
            return self._stop(f"안전 — {stop}")
        if not self._granted():
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
            self.pos_pubs[name].publish(Float64MultiArray(data=[goal_deg, self.speed]))

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
        lim = None
        if self.env is not None:
            r = self.env['poses'].get(pose) if pose is not None else self.env['any']
            lim = {k: [round(v[0], 1), round(v[1], 1)] for k, v in (r or {}).items()}
        self.status_pub.publish(String(data=json.dumps({
            'moving': self.moving,
            'current_mm': cur,
            'target': {k: round(v, 2) for k, v in self.target.items()},
            'target_unit': self.target_unit,
            'current_deg': {k: (round(v, 2) if v is not None else None)
                            for k, v in self.deg.items()},
            'homed': sorted(self.refs),
            'pose': pose,                 # 지금 yaw 자세 (None = 자세 사이)
            'pose_detail': pose_detail,
            'pose_want': want,            # 지금 위치에서 결속할 자세
            'pose_want_why': want_why,
            'limit_mm': lim,              # 지금 자세에서 갈 수 있는 X·Y 범위
            'enforce_envelope': self.enforce,
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
