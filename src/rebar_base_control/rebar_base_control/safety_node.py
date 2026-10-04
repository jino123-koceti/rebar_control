#!/usr/bin/env python3
"""L2 — 안전 판정. 여기서 **판정만** 하고 차단은 L1(모터 직전)에서 한다.

왜 나누는가: 상위에서 막으면 "무엇이 명령했든" 을 보장할 수 없다. 텔레옵·호밍·자율주행이
각자 차단 로직을 들고 있으면 한 곳만 빠뜨려도 구멍이 난다. 판정은 한 곳, 차단은 모터 직전
(YEAR3_ARCHITECTURE.md §2 계층 규칙 4).

## 입력

  /remote_control        비상정지 (송신기 꺼짐·START 전도 비상정지로 들어온다)
  /switches/stop         하부 STOP 스위치
  /bumpers/{front,rear,left,right}   범퍼 (극성은 ezi_io_node 가 흡수 → True=눌림)
  /limit_sensors/*       상부 축 리미트
  /obstacle_pause        사람·장애물 (있으면)
  /deck_edge_block       데크 이탈 방향 (JSON, 있으면)

## 출력

  /safety/state (SafetyState)  20Hz + 상태 변화 시 즉시

## 방향별로 막는다

범퍼가 눌렸다는 건 **이미 부딪혔다**는 뜻이다. 전 방향을 막으면 빠져나올 수단까지
사라져 그 자리에 갇힌다. 그래서 부딪힌 방향만 막고 반대는 열어 둔다
(2차년도 bumper_node·deck_edge_node 가 같은 결론에 도달했다).

리미트도 같다. `x_min` 에 닿았으면 `x-` 방향만 막고 `x+` 는 열어 둔다. 안 그러면
리미트에 붙은 축을 뺄 수 없어 호밍도 못 한다.

## 입력이 끊기면

안전 입력이 `input_timeout` 이상 갱신되지 않으면 `inputs_stale` 을 세운다. L1 은 이를
전체 정지로 취급한다. **센서를 못 보는 상태로 움직이는 것이 가장 위험하다.**
단 시작 직후에는 아직 아무 입력도 안 왔을 수 있으므로, 한 번이라도 받은 뒤부터 판정한다.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rebar_base_interfaces.msg import RemoteControl, SafetyState

BUMPERS = ('front', 'rear', 'left', 'right')
LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')
# 리미트 → 막을 축 방향. yaw_home 은 원점 센서라 방향을 막지 않는다(호밍이 써야 한다).
# ⚠⚠ **Z 만 부호가 반대다.** 막을 방향은 "리미트 쪽으로 더 가는 mm 방향" 인데,
#   mm 는 **호밍 원점 기준**이고 Z 의 원점(z_min)은 **위쪽**이다:
#       X  x_min(원점) → 0 .. +453.7     | y 도 같다
#       Z  z_min(원점·위) → 0 .. **-139.2**   ← mm 가 아래로 갈수록 음수
#   그래서 z_min 은 **'z+'**(원점 쪽 = 위)를 막아야 한다. 'z-' 로 두면 **내려가는
#   방향을 막아** 결속 깊이로 갈 수 없다 — 2026-10-04 에 stage_node 가
#   "z: 안전 차단으로 그 방향 이동 불가" 로 Z 명령을 통째로 버렸다.
LIMIT_BLOCKS = {
    'x_min': 'x-', 'x_max': 'x+',
    'y_min': 'y-', 'y_max': 'y+',
    'z_min': 'z+', 'z_max': 'z-',
}


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('publish_rate', 20.0)
        # 안전 입력이 이 시간 이상 갱신되지 않으면 정지로 본다
        self.declare_parameter('input_timeout', 0.5)
        # 범퍼를 주행 차단으로 쓸지 (배선 확인 전에는 끌 수 있게 둔다)
        self.declare_parameter('use_bumpers', True)
        self.declare_parameter('use_limits', True)

        g = self.get_parameter
        self.input_timeout = float(g('input_timeout').value)
        self.use_bumpers = bool(g('use_bumpers').value)
        self.use_limits = bool(g('use_limits').value)

        self.pub = self.create_publisher(SafetyState, '/safety/state', 10)

        self.bumper = {n: False for n in BUMPERS}
        self.limit = {n: False for n in LIMITS}
        self.stop_switch = False
        self.estop = True          # 안전측 기본값: 입력이 오기 전엔 정지로 본다
        self.obstacle = False
        self.deck_block = []
        self.seen = {}             # 입력별 마지막 수신 시각

        self.create_subscription(RemoteControl, '/remote_control', self._on_remote, 10)
        self.create_subscription(Bool, '/switches/stop', self._on_stop, 10)
        for n in BUMPERS:
            self.create_subscription(Bool, f'/bumpers/{n}',
                                     lambda m, k=n: self._on_bumper(k, m), 10)
        for n in LIMITS:
            self.create_subscription(Bool, f'/limit_sensors/{n}',
                                     lambda m, k=n: self._on_limit(k, m), 10)
        self.create_subscription(Bool, '/obstacle_pause', self._on_obstacle, 10)
        self.create_subscription(String, '/deck_edge_block', self._on_deck, 10)

        self._last_state = None
        self.timer = self.create_timer(1.0 / float(g('publish_rate').value), self.tick)
        self.get_logger().info(
            f"안전 노드 시작 — 판정만 하고 차단은 L1 이 한다. "
            f"범퍼 {'사용' if self.use_bumpers else '미사용'}, "
            f"리미트 {'사용' if self.use_limits else '미사용'}, "
            f"입력 타임아웃 {self.input_timeout:.1f}s")

    # ---- 입력 --------------------------------------------------------------
    def _mark(self, key):
        self.seen[key] = time.time()

    def _on_remote(self, msg):
        self.estop = bool(msg.emergency_stop)
        self._mark('remote')

    def _on_stop(self, msg):
        self.stop_switch = bool(msg.data)
        self._mark('stop')

    def _on_bumper(self, name, msg):
        self.bumper[name] = bool(msg.data)
        self._mark('bumper')

    def _on_limit(self, name, msg):
        self.limit[name] = bool(msg.data)
        self._mark('limit')

    def _on_obstacle(self, msg):
        self.obstacle = bool(msg.data)
        self._mark('obstacle')

    def _on_deck(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        b = d.get('block') or d.get('blocked') or []
        self.deck_block = [str(x) for x in b] if isinstance(b, list) else []
        self._mark('deck')

    # ---- 판정 --------------------------------------------------------------
    def _stale(self):
        """한 번이라도 받은 입력 중 갱신이 끊긴 것들."""
        now = time.time()
        return [k for k, t in self.seen.items() if now - t > self.input_timeout]

    def tick(self):
        s = SafetyState()
        s.header.stamp = self.get_clock().now().to_msg()
        reasons = []

        stale = self._stale()
        if stale:
            s.inputs_stale = True
            reasons.append(f"입력 두절: {', '.join(stale)}")

        if self.estop:
            s.estop = True
            reasons.append("비상정지(또는 송신기 꺼짐·START 전)")
        if self.stop_switch:
            s.stop_switch = True
            reasons.append("STOP 스위치")

        if self.use_bumpers:
            # 부딪힌 방향만 막는다
            if self.bumper['front']:
                s.block_forward = True; reasons.append("전방 범퍼")
            if self.bumper['rear']:
                s.block_backward = True; reasons.append("후방 범퍼")
            if self.bumper['left']:
                s.block_left = True; reasons.append("좌측 범퍼")
            if self.bumper['right']:
                s.block_right = True; reasons.append("우측 범퍼")

        if self.obstacle:
            s.block_forward = True
            s.block_backward = True
            reasons.append("장애물 감지")

        for d in self.deck_block:
            if d in ('forward', 'front'):
                s.block_forward = True
            elif d in ('backward', 'rear'):
                s.block_backward = True
        if self.deck_block:
            reasons.append(f"데크끝: {', '.join(self.deck_block)}")

        blocked = []
        if self.use_limits:
            for name, axis_dir in LIMIT_BLOCKS.items():
                if self.limit.get(name):
                    blocked.append(axis_dir)
                    reasons.append(f"{name} 리미트")
        s.blocked_axes = blocked

        s.reason = " / ".join(reasons)
        self.pub.publish(s)

        key = (s.estop, s.stop_switch, s.inputs_stale, s.block_forward, s.block_backward,
               s.block_left, s.block_right, tuple(blocked))
        if key != self._last_state:
            self._last_state = key
            if reasons:
                self.get_logger().warning(f"안전 차단 — {s.reason}")
            else:
                self.get_logger().info("안전 정상 — 차단 없음")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyNode()
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
