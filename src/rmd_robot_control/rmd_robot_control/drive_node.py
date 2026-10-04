#!/usr/bin/env python3
"""L3 — 주행부를 **거리로** 움직인다.

    /drive/step (Float32 mm)  →  그만큼 전진·후진하고 정지한다

`position_control_node` 는 `/cmd_vel` 로 속도만 받는다. 거리를 가려면 바퀴
각도를 보며 속도를 끊어야 하는데, 그 판단을 이 노드가 한다.

## 왜 바퀴 각도를 쓰는가

시간으로 적분하면(속도×시간) 가감속 램프와 슬립이 전부 오차가 된다. 2026-10-04
실측에서 0.03m/s × 1.5s = 45mm 를 명령했는데 바퀴는 42.8mm 만 돌았다 — 램프만으로
5~7% 다. 바퀴 각도는 램프를 자동으로 포함한다.

⚠ **전진 시 두 바퀴의 부호가 반대다** (2026-10-04 실측: 우측 +84°, 좌측 -82°).
  그래서 전진거리는 `(Δ우 - Δ좌)/2 × mm_per_deg` 다. 합으로 쓰면 0 이 나와 안
  움직인 것처럼 보인다. CAN ID 는 `axes.yaml` 의 `drive` 에만 있다.

⚠ 바퀴 각도(멀티턴 각도 읽기)는 `position_control_node` 의 `wheel_angle_poll` 이 켜져 있어야
  온다. 2차년도는 위치 제어(S20) 응답에서만 발행해서 `/cmd_vel` 로 달릴 때는
  각도가 아예 없었다. 표본이 안 오면 **이동을 거부한다** — 없는 것을 0 으로 보면
  거리 제어가 열린 루프가 된다.

## 정밀도는 어느 정도여야 하는가

정지마다 카메라로 **다시 검출**하므로, 주행은 "다음 철근 띠가 스테이지 작업영역에
들어올 만큼" 이면 된다. ±수십 mm 로 충분하고 위치추정도 경로추종도 필요 없다.
그래서 이 노드는 한 축(전진)만 다루고 회전은 직진 유지용으로만 쓴다.

## 토픽

  구독  /drive/step       Float32   mm. + 전진, - 후진
        /drive/abort      Empty     즉시 정지
        /motor_<우>_position  Float32  우측 바퀴 출력축 각도(도).
        /motor_<좌>_position  Float32  ID 는 axes.yaml 의 drive 에서 읽는다
        /safety/state     SafetyState
        /control_mode     String
  발행  /cmd_vel          Twist     속도 명령
        /control_mode_request String  'auto' 요청 / 'release'
        /drive/status     String(JSON)  상태·남은거리·사유
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String
from rebar_base_interfaces.msg import SafetyState

from .axis_config import load_drive


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')

        self.cfg = load_drive()
        # 순항 속도. 정격 0.249m/s 의 1/3 쯤 — 철근 위를 가므로 보수적으로 둔다.
        self.declare_parameter('speed_mps', 0.08)
        # 목표 근처에서 기어가는 속도. 램프 때문에 순항으로 가면 넘어간다.
        self.declare_parameter('creep_mps', 0.025)
        self.declare_parameter('approach_mm', 80.0)    # 이 안에서 creep
        self.declare_parameter('tol_mm', 5.0)
        self.declare_parameter('timeout_sec', 90.0)
        # 직진 유지. 좌우 바퀴가 간 거리 차이에 비례해 각속도를 준다.
        # ⚠ 과하면 사행한다. 0 이면 보정하지 않는다 (드리프트가 그대로 남는다).
        self.declare_parameter('straight_gain', 0.004)  # rad/s per mm
        self.declare_parameter('grant_wait_sec', 3.0)
        # 바퀴 각도가 이보다 오래 묵으면 거부·중단한다. 10Hz 로 오므로 0.5초면 넉넉하다
        self.declare_parameter('angle_stale_sec', 0.5)

        g = self.get_parameter
        self.speed = float(g('speed_mps').value)
        self.creep = float(g('creep_mps').value)
        self.approach = float(g('approach_mm').value)
        self.tol = float(g('tol_mm').value)
        self.timeout = float(g('timeout_sec').value)
        self.kstraight = float(g('straight_gain').value)
        self.grant_wait = float(g('grant_wait_sec').value)
        self.stale = float(g('angle_stale_sec').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode_request', 10)
        self.status_pub = self.create_publisher(String, '/drive/status', 10)

        self.ang = {}            # 'right'/'left' → (각도, 받은 시각)
        if self.cfg:
            for side in ('right', 'left'):
                self.create_subscription(
                    Float32, f"/motor_{self.cfg[side]}_position",
                    (lambda s: (lambda m: self.ang.__setitem__(
                        s, (float(m.data), time.time()))))(side), 20)
        self.safety = None
        self.mode = None
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 1)
        # ⚠ `/control_mode` 는 **JSON** 이다 (평문 'auto' 가 아니다). 2026-10-04 에
        #   평문으로 비교해 "제어 권한 없음" 으로 매번 멈췄다.
        self.create_subscription(String, '/control_mode', self._on_mode, 10)
        self.create_subscription(Float32, '/drive/step', self._on_step, 10)
        self.create_subscription(Empty, '/drive/abort',
                                 lambda m: self._stop('중단 명령'), 10)

        self.moving = False
        self.goal = 0.0          # 목표 거리 mm (부호 있음)
        self.base = None         # 시작 시점의 (우, 좌) 각도
        self.detail = '대기'
        self.rejects = 0
        self.t_start = 0.0

        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self._publish)
        if not self.cfg:
            self.get_logger().error(
                "axes.yaml 의 drive 제원을 못 읽었다 — 거리 제어를 할 수 없다. "
                "모든 /drive/step 을 거부한다")
        else:
            self.get_logger().info(
                f"주행 노드 시작 — /drive/step 에 mm 를 준다 (+전진). "
                f"순항 {self.speed:.2f}m/s, 접근 {self.approach:.0f}mm 에서 "
                f"{self.creep:.3f}m/s, 허용오차 {self.tol:.0f}mm, "
                f"{self.cfg['mm_per_deg']:.5f}mm/도")

    # ---- 입력 --------------------------------------------------------------
    def _on_safety(self, msg):
        self.safety = msg

    def _on_step(self, msg):
        if self.moving:
            return self._reject(f'이미 주행 중이다 (남은 {self._left():+.0f}mm)')
        if not self.cfg:
            return self._reject('주행 제원이 없다')
        d = float(msg.data)
        if not math.isfinite(d) or abs(d) < self.tol:
            return self._reject(f'거리가 너무 작거나 값이 아니다 ({d})')
        a = self._angles()
        if a is None:
            return self._reject(
                '바퀴 각도가 없다 — position_control_node 의 wheel_angle_poll 을 확인하라')
        stop = self._safety_stop()
        if stop:
            return self._reject(stop)
        self.goal = d
        self.base = a
        self.moving = True
        self.t_start = time.time()
        self.detail = f'주행 시작 {d:+.0f}mm'
        self._request('auto')
        self.get_logger().info(f"주행 시작 — {d:+.0f}mm")

    # ---- 상태 --------------------------------------------------------------
    def _angles(self):
        """(우, 좌) 각도. 하나라도 없거나 묵었으면 None."""
        now = time.time()
        out = []
        for side in ('right', 'left'):
            v = self.ang.get(side)
            if v is None or now - v[1] > self.stale:
                return None
            out.append(v[0])
        return tuple(out)

    def _moved(self):
        """지금까지 간 거리 mm (부호 있음). 각도가 없으면 None.

        ⚠ 전진 시 우측 +, 좌측 - 다 — **차를 2로 나눈다.**
        """
        a = self._angles()
        if a is None or self.base is None:
            return None
        dr, dl = a[0] - self.base[0], a[1] - self.base[1]
        return (dr - dl) / 2.0 * self.cfg['mm_per_deg']

    def _drift(self):
        """좌우가 간 거리 차이 mm. + 면 우측이 더 갔다 (왼쪽으로 휜다)."""
        a = self._angles()
        if a is None or self.base is None:
            return 0.0
        dr, dl = a[0] - self.base[0], -(a[1] - self.base[1])
        return (dr - dl) * self.cfg['mm_per_deg']

    def _left(self):
        m = self._moved()
        return 0.0 if m is None else self.goal - m

    def _safety_stop(self):
        s = self.safety
        if s is None:
            return None               # 안전 노드가 없으면 막지 않는다 (stage_node 와 같다)
        if s.estop:
            return '비상정지'
        if s.stop_switch:
            return 'STOP 스위치'
        if s.inputs_stale:
            return '안전 입력이 묵었다'
        return None

    def _on_mode(self, msg):
        try:
            self.mode = (json.loads(msg.data) or {}).get('mode')
        except ValueError:
            pass

    def _request(self, what):
        # 요청 형식은 "<노드이름> <무엇>" 이다 (중재기가 소유자를 기록한다)
        self.mode_pub.publish(String(data=f"drive_node {what}"))

    def _granted(self):
        # 중재기가 없으면(모드를 못 받으면) 막지 않는다 — stage_node 와 같다
        return self.mode is None or self.mode == 'auto'

    # ---- 진행 --------------------------------------------------------------
    def _reject(self, why):
        self.rejects += 1
        self.detail = f'거부: {why}'
        self.get_logger().error(f"거부 — {why}")
        self._publish()

    def _stop(self, why):
        if self.moving:
            m = self._moved()
            self.get_logger().info(
                f"주행 종료 — {why}"
                + ('' if m is None else f" (간 거리 {m:+.1f}mm)"))
        self.moving = False
        self.base = None
        self.detail = why
        for _ in range(3):            # 정지는 확실하게 — 한 번은 놓칠 수 있다
            self.cmd_pub.publish(Twist())
        self._request('release')
        self._publish()

    def tick(self):
        if not self.moving:
            return
        stop = self._safety_stop()
        if stop:
            self.rejects += 1
            return self._stop(f'안전 — {stop}')
        if time.time() - self.t_start > self.timeout:
            self.rejects += 1
            return self._stop(f'타임아웃 {self.timeout:.0f}s')
        if not self._granted():
            self._request('auto')
            if time.time() - self.t_start < self.grant_wait:
                self.cmd_pub.publish(Twist())
                return
            self.rejects += 1
            return self._stop(f'제어 권한 없음 (모드 {self.mode})')
        # ⚠ **하트비트가 필요하다.** 중재기는 재요청이 끊기면 권한을 회수한다 —
        #   2026-10-04 에 시작할 때만 요청해서 200mm 중 167mm 에서 manual 로
        #   돌아가 멈췄다. `stage_node` 도 매 tick 재요청한다.
        self._request('auto')

        left = self._left()
        if self._moved() is None:
            self.rejects += 1
            return self._stop('바퀴 각도가 묵었다 — 거리를 알 수 없어 멈춘다')
        if abs(left) <= self.tol:
            return self._stop('목표 도달')

        v = self.speed if abs(left) > self.approach else self.creep
        v = math.copysign(v, left)
        # 직진 유지 — 많이 간 쪽을 늦춘다. 후진이면 보정 방향도 뒤집힌다.
        w = -self.kstraight * self._drift() * (1.0 if left > 0 else -1.0)
        w = max(-self.cfg['max_angular_radps'], min(self.cfg['max_angular_radps'], w))
        t = Twist()
        t.linear.x = max(-self.cfg['max_linear_mps'],
                         min(self.cfg['max_linear_mps'], v))
        t.angular.z = float(w)
        self.cmd_pub.publish(t)
        self.detail = (f'주행 중 — 남은 {left:+.0f}mm, '
                       f'{t.linear.x:+.3f}m/s, 좌우차 {self._drift():+.1f}mm')

    # ---- 발행 --------------------------------------------------------------
    def _publish(self):
        m = self._moved()
        self.status_pub.publish(String(data=json.dumps({
            'moving': self.moving,
            'goal_mm': round(self.goal, 1) if self.moving else None,
            'moved_mm': None if m is None else round(m, 1),
            'left_mm': round(self._left(), 1) if self.moving else None,
            'drift_mm': round(self._drift(), 1) if self.moving else None,
            'angles_ok': self._angles() is not None,
            'rejects': self.rejects,
            'detail': self.detail,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DriveNode()
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
