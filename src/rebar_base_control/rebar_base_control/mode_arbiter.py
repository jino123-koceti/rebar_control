#!/usr/bin/env python3
"""L4 — 제어 권한 중재. **누가 축·주행에 명령을 쓸 수 있는지** 한 곳에서 정한다.

## 왜 필요한가 (2026-09-30 실측으로 드러난 문제)

호밍이 `/joint_3/speed` 에 30dps 를 20Hz 로 보내는 동안, `remote_teleop_node` 가
같은 토픽에 "정지(0)" 를 10Hz 로 계속 보내고 있었다. 리모콘 스틱이 중립이라 정지를
주기적으로 새로 보내는 정상 동작인데, 둘이 서로를 덮어써서 모터가 **30 받고 돌다 0 받고
서기를 초당 수 회 반복**했다 (로그에서 0.0 이 102회, 30.0 이 103회로 정확히 1:1).
축이 툭툭 끊기고, 결국 "안 움직인다"는 스톨 판정으로 호밍이 실패했다.

겉보기 증상이 모터 이상과 똑같아서 한참을 모터·브레이크·기구·CAN 을 의심했다.
**L3 노드 둘이 조정 없이 같은 L1 토픽에 쓴 것이 원인이었다.**

2차년도에도 같은 일이 있었다 — [A] 와 [B] 가 둘 다 `/cmd_vel` 에 쏘면서 중재가 없었고,
코드에 "섞이면 덜컥거린다" 고 적혀 있다 (YEAR3_ARCHITECTURE.md §4).

## 규칙

  · 기본값은 `manual` — 아무도 안 잡고 있으면 리모콘이 쓴다
  · 요청은 `/control_mode_request` 로 `"<요청자> <모드>"` 형식
      "homing_node homing"   권한 요청
      "homing_node release"  반납
  · **아무도 안 잡고 있을 때만** 넘겨준다. 이미 다른 주인이 있으면 거절한다
  · 주인이 죽어도 풀린다 — `hold_timeout` 동안 갱신 요청이 없으면 자동 반납.
    안 그러면 노드가 죽었을 때 리모콘이 영영 막힌다
  · **안전이 우선이다** — 비상정지·STOP·입력두절이면 즉시 `manual` 로 되돌린다.
    조작자가 언제나 빠져나올 수 있어야 한다

## 아래 노드들이 지켜야 할 것

`/control_mode` 를 구독해서 **자기 모드가 아닐 때는 명령을 발행하지 않는다.**
권한을 잃는 순간 **정지(0)를 한 번 보내고 그 뒤로는 조용히** 있어야 한다 —
계속 0 을 보내면 그게 바로 이 문제의 원인이다.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rebar_base_interfaces.msg import SafetyState

MANUAL = 'manual'
VALID = ('manual', 'homing', 'auto')


class ModeArbiter(Node):
    def __init__(self):
        super().__init__('mode_arbiter')

        self.declare_parameter('publish_rate', 10.0)
        # 주인이 이 시간 동안 갱신 요청을 안 하면 권한을 회수한다 (죽은 노드 대비)
        self.declare_parameter('hold_timeout', 3.0)
        # 안전 상태가 이 시간 이상 안 오면 "안전을 모른다" 로 보고 manual 로 되돌린다
        self.declare_parameter('safety_timeout', 1.0)

        g = self.get_parameter
        self.hold_timeout = float(g('hold_timeout').value)
        self.safety_timeout = float(g('safety_timeout').value)

        self.mode = MANUAL
        self.owner = ''
        self.since = time.time()
        self.last_hold = 0.0
        self.reason = '기본값'

        self.safety = None
        self.safety_time = 0.0

        self.pub = self.create_publisher(String, '/control_mode', 10)
        self.create_subscription(String, '/control_mode_request', self._on_request, 10)
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 1)

        self._last_published = None
        self.create_timer(1.0 / float(g('publish_rate').value), self.tick)
        self.get_logger().info(
            f"권한 중재 시작 — 기본 {MANUAL}, 주인 무응답 {self.hold_timeout:.0f}초면 회수")

    # ---- 입력 --------------------------------------------------------------
    def _on_safety(self, msg):
        self.safety = msg
        self.safety_time = time.time()

    def _unsafe(self):
        """조작자가 빠져나와야 하는 상황인가."""
        if self.safety is None:
            return None            # 아직 한 번도 안 왔다 — 판단하지 않는다
        if time.time() - self.safety_time > self.safety_timeout:
            return '안전 상태 두절'
        if self.safety.estop:
            return '비상정지'
        if self.safety.stop_switch:
            return 'STOP 스위치'
        if self.safety.inputs_stale:
            return '안전 입력 두절'
        return None

    def _on_request(self, msg):
        parts = msg.data.strip().split()
        if len(parts) != 2:
            self.get_logger().error(
                f"요청 형식이 아닙니다: '{msg.data}' — '<요청자> <모드|release>'")
            return
        who, what = parts

        if what == 'release':
            if self.owner == who:
                self._set(MANUAL, '', f"{who} 반납")
            return

        if what not in VALID:
            self.get_logger().error(f"모르는 모드: '{what}'")
            return

        if what == MANUAL:
            self._set(MANUAL, '', f"{who} 요청")
            return

        unsafe = self._unsafe()
        if unsafe:
            self.get_logger().warning(f"{who} 의 '{what}' 요청 거절 — {unsafe}")
            return

        if self.owner in ('', who):
            self.last_hold = time.time()
            if self.mode != what or self.owner != who:
                self._set(what, who, f"{who} 획득")
            return

        self.get_logger().warning(
            f"{who} 의 '{what}' 요청 거절 — 이미 {self.owner} 가 '{self.mode}' 를 잡고 있다")

    # ---- 판정 --------------------------------------------------------------
    def _set(self, mode, owner, reason):
        if (mode, owner) == (self.mode, self.owner):
            return
        self.get_logger().info(
            f"권한 {self.mode}({self.owner or '-'}) → {mode}({owner or '-'})  [{reason}]")
        self.mode, self.owner, self.reason = mode, owner, reason
        self.since = time.time()
        self._publish()

    def tick(self):
        if self.mode != MANUAL:
            unsafe = self._unsafe()
            if unsafe:
                self._set(MANUAL, '', f"안전 우선 — {unsafe}")
            elif time.time() - self.last_hold > self.hold_timeout:
                self._set(MANUAL, '', f"{self.owner} 무응답 {self.hold_timeout:.0f}초")
        self._publish()

    def _publish(self):
        payload = json.dumps({
            'mode': self.mode,
            'owner': self.owner,
            'reason': self.reason,
            'since': round(time.time() - self.since, 1),
        }, ensure_ascii=False)
        self.pub.publish(String(data=payload))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ModeArbiter()
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
