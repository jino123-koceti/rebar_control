#!/usr/bin/env python3
"""안전 판정 무부하 검증 — 하드웨어 없이 돌린다.

가짜 입력(리모콘·범퍼·STOP·리미트)을 발행하고 `/safety/state` 판정이 맞는지 본다.
모터도 EZIO 도 필요 없다.

확인하는 것
  · 비상정지·STOP → 전체 정지 플래그
  · 범퍼 → **부딪힌 방향만** 차단 (전 방향을 막으면 빠져나올 수 없다)
  · 리미트 → 그 축의 **그 방향만** 차단 (반대 방향은 열려 있어야 호밍이 된다)
  · 입력이 끊기면 inputs_stale

**주의:** `ezi_io_node` 와 `remote_bridge` 가 떠 있으면 진짜 입력과 충돌한다. 끄고 돌릴 것.

사용:
    ros2 run rebar_base_control safety_node.py &
    python3 tools/test/safety_dryrun.py
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rebar_base_interfaces.msg import RemoteControl, SafetyState

BUMPERS = ('front', 'rear', 'left', 'right')
LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')


class Harness(Node):
    def __init__(self):
        super().__init__('safety_dryrun')
        self.rc_pub = self.create_publisher(RemoteControl, '/remote_control', 10)
        self.stop_pub = self.create_publisher(Bool, '/switches/stop', 10)
        self.bump_pub = {n: self.create_publisher(Bool, f'/bumpers/{n}', 10) for n in BUMPERS}
        self.limit_pub = {n: self.create_publisher(Bool, f'/limit_sensors/{n}', 10)
                          for n in LIMITS}
        self.state = None
        self.create_subscription(SafetyState, '/safety/state', self._on_state, 10)
        self.estop = False
        self.stop = False
        self.bump = {n: False for n in BUMPERS}
        self.limit = {n: False for n in LIMITS}
        self.feed = True

    def _on_state(self, msg):
        self.state = msg

    def pump(self):
        if not self.feed:
            return
        rc = RemoteControl()
        rc.emergency_stop = self.estop
        rc.switch_s10 = True
        rc.joysticks = [0.0, 0.0, 0.0, 0.0]
        rc.buttons = [0] * 8
        self.rc_pub.publish(rc)
        self.stop_pub.publish(Bool(data=self.stop))
        for n, p in self.bump_pub.items():
            p.publish(Bool(data=self.bump[n]))
        for n, p in self.limit_pub.items():
            p.publish(Bool(data=self.limit[n]))


def spin(h, sec):
    end = time.time() + sec
    while rclpy.ok() and time.time() < end:
        h.pump()
        rclpy.spin_once(h, timeout_sec=0.02)


def check(name, cond, detail=''):
    print(f"  {'통과' if cond else '실패'}  {name}" + (f"   {detail}" if detail else ''))
    return cond


def main():
    rclpy.init()
    h = Harness()
    ok = True
    spin(h, 1.5)
    if h.state is None:
        print("실패 — /safety/state 를 못 받았다. safety_node 가 떠 있나?")
        rclpy.shutdown()
        return 1

    print("■ 평상시 (모든 입력 정상)")
    s = h.state
    ok &= check("전체 정지 사유 없음", not (s.estop or s.stop_switch or s.inputs_stale),
                f"reason='{s.reason}'")
    ok &= check("방향 차단 없음", not any([s.block_forward, s.block_backward,
                                        s.block_left, s.block_right]))
    ok &= check("축 차단 없음", not s.blocked_axes)

    print("■ 비상정지")
    h.estop = True; spin(h, 0.6)
    ok &= check("estop 플래그", h.state.estop, f"reason='{h.state.reason}'")
    h.estop = False; spin(h, 0.6)
    ok &= check("해제되면 풀린다", not h.state.estop)

    print("■ STOP 스위치")
    h.stop = True; spin(h, 0.6)
    ok &= check("stop_switch 플래그", h.state.stop_switch)
    h.stop = False; spin(h, 0.6)

    print("■ 전방 범퍼 — 전진만 막혀야 한다")
    h.bump['front'] = True; spin(h, 0.6)
    s = h.state
    ok &= check("전진 차단", s.block_forward)
    ok &= check("후진은 열려 있다 (갇히지 않게)", not s.block_backward)
    h.bump['front'] = False; spin(h, 0.6)

    print("■ 좌측 범퍼 — 좌선회만 막혀야 한다")
    h.bump['left'] = True; spin(h, 0.6)
    s = h.state
    ok &= check("좌선회 차단", s.block_left)
    ok &= check("우선회는 열려 있다", not s.block_right)
    h.bump['left'] = False; spin(h, 0.6)

    print("■ x_min 리미트 — x- 만 막혀야 한다")
    h.limit['x_min'] = True; spin(h, 0.6)
    s = h.state
    ok &= check("x- 차단", 'x-' in s.blocked_axes, f"blocked={list(s.blocked_axes)}")
    ok &= check("x+ 는 열려 있다 (리미트에서 빠져나와야 한다)", 'x+' not in s.blocked_axes)
    h.limit['x_min'] = False; spin(h, 0.6)

    print("■ yaw_home — 원점 센서라 방향을 막지 않아야 한다")
    h.limit['yaw_home'] = True; spin(h, 0.6)
    ok &= check("yaw 방향 차단 없음", not [a for a in h.state.blocked_axes if a.startswith('yaw')],
                f"blocked={list(h.state.blocked_axes)}")
    h.limit['yaw_home'] = False; spin(h, 0.6)

    print("■ 입력 두절")
    h.feed = False; spin(h, 1.5)
    ok &= check("inputs_stale 플래그", h.state.inputs_stale, f"reason='{h.state.reason}'")

    print(f"\n■ 결과: {'통과' if ok else '실패'}")
    h.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
