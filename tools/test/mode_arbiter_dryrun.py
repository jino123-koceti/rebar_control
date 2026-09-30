#!/usr/bin/env python3
"""권한 중재 무부하 검증 — 하드웨어 없이 돌린다.

확인하는 것
  · 기본값이 manual 인가
  · 아무도 안 잡았을 때만 넘겨주는가 (이미 주인이 있으면 거절)
  · 주인이 반납하면 manual 로 돌아오는가
  · **주인이 죽으면 자동 회수되는가** (안 그러면 리모콘이 영영 막힌다)
  · **비상정지·STOP 이면 즉시 manual 로 되돌리는가** (조작자가 빠져나올 수 있어야 한다)
  · 안전하지 않은 상태에서의 권한 요청을 거절하는가

사용:
    ros2 run rebar_base_control mode_arbiter.py &
    python3 tools/test/mode_arbiter_dryrun.py
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rebar_base_interfaces.msg import SafetyState


class H(Node):
    def __init__(self):
        super().__init__('mode_arbiter_dryrun')
        self.req = self.create_publisher(String, '/control_mode_request', 10)
        self.saf = self.create_publisher(SafetyState, '/safety/state', 10)
        self.mode = None
        self.owner = None
        self.create_subscription(String, '/control_mode', self._on, 10)
        self.estop = False
        self.stop = False

    def _on(self, m):
        try:
            d = json.loads(m.data)
            self.mode, self.owner = d.get('mode'), d.get('owner')
        except ValueError:
            pass

    def pump(self):
        s = SafetyState()
        s.estop = self.estop
        s.stop_switch = self.stop
        self.saf.publish(s)

    def spin(self, sec, feed=True):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            if feed:
                self.pump()
            rclpy.spin_once(self, timeout_sec=0.02)

    def ask(self, who, what):
        self.req.publish(String(data=f"{who} {what}"))


def check(name, cond, detail=''):
    print(f"  {'통과' if cond else '실패'}  {name}" + (f"   {detail}" if detail else ''))
    return cond


def main():
    rclpy.init()
    h = H()
    ok = True
    h.spin(1.5)
    if h.mode is None:
        print("실패 — /control_mode 를 못 받았다. mode_arbiter 가 떠 있나?")
        rclpy.shutdown()
        return 1

    print("■ 기본값")
    ok &= check("manual 로 시작", h.mode == 'manual', f"mode={h.mode}")

    print("■ 권한 획득")
    h.ask('homing_node', 'homing'); h.spin(0.6)
    ok &= check("homing 으로 넘어간다", h.mode == 'homing', f"owner={h.owner}")

    print("■ 다른 노드의 요청은 거절")
    h.ask('other_node', 'auto'); h.spin(0.6)
    ok &= check("주인이 안 바뀐다", h.mode == 'homing' and h.owner == 'homing_node',
                f"mode={h.mode} owner={h.owner}")

    print("■ 반납")
    h.ask('homing_node', 'release'); h.spin(0.6)
    ok &= check("manual 로 돌아온다", h.mode == 'manual')

    print("■ 주인이 죽으면 자동 회수 (하트비트 끊김)")
    h.ask('homing_node', 'homing'); h.spin(0.6)
    got = h.mode == 'homing'
    h.spin(4.5)                      # hold_timeout 기본 3초보다 길게, 갱신 없이
    ok &= check("권한을 잡았다가", got)
    ok &= check("무응답이면 manual 로 회수", h.mode == 'manual', f"mode={h.mode}")

    print("■ 비상정지면 즉시 manual (조작자 탈출 경로)")
    h.ask('homing_node', 'homing'); h.spin(0.6)
    got = h.mode == 'homing'
    h.estop = True; h.spin(1.0)
    ok &= check("권한을 잡았다가", got)
    ok &= check("비상정지로 회수", h.mode == 'manual', f"mode={h.mode}")

    print("■ 안전하지 않으면 요청 거절")
    h.ask('homing_node', 'homing'); h.spin(0.8)
    ok &= check("비상정지 중에는 못 준다", h.mode == 'manual')
    h.estop = False; h.spin(0.6)

    print("■ STOP 스위치도 같은 효과")
    h.ask('homing_node', 'homing'); h.spin(0.6)
    got = h.mode == 'homing'
    h.stop = True; h.spin(1.0)
    ok &= check("권한을 잡았다가", got)
    ok &= check("STOP 으로 회수", h.mode == 'manual')
    h.stop = False
    h.ask('homing_node', 'release'); h.spin(0.4)

    print(f"\n■ 결과: {'통과' if ok else '실패'}")
    h.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
