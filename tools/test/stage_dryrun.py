#!/usr/bin/env python3
"""스테이지 이동 무부하 검증 — 하드웨어 없이 돌린다.

가짜 엔코더·가짜 호밍 결과·가짜 안전 상태를 주고, `stage_node` 가 내는
`/joint_N/position` 과 `/brake_cmd` 를 관찰한다. 모터도 CAN 도 필요 없다.

확인하는 것
  · 호밍 원점이 없으면 **거부**하는가 (기준 없이 절대 위치로 보내면 안 된다)
  · mm_per_deg 미측정이면 mm 목표를 **거부**하는가
  · 각도 목표(/stage/goal_deg)는 환산 없이 받는가
  · **브레이크 해제가 축 명령보다 먼저** 나가는가 (오늘 호밍에서 문제였던 부분)
  · 목표에 도달하면 멈추고 **브레이크를 잠그는가**
  · 비상정지에 반응하는가

**주의:** 실장비의 position_control_node 가 떠 있으면 가짜 엔코더와 충돌한다. 끄고 돌릴 것.

사용:
    ros2 run rmd_robot_control stage_node &
    python3 tools/test/stage_dryrun.py
"""

import json
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, String
from rebar_base_interfaces.msg import SafetyState

NAN = float('nan')
MOTOR = {'x': '0x145', 'y': '0x146', 'z': '0x147'}
JOINT = {'x': 3, 'y': 4, 'z': 5}


class H(Node):
    def __init__(self):
        super().__init__('stage_dryrun')
        self.goal = self.create_publisher(Point, '/stage/goal', 10)
        self.goal_deg = self.create_publisher(Point, '/stage/goal_deg', 10)
        self.homing = self.create_publisher(String, '/homing_status', 10)
        self.saf = self.create_publisher(SafetyState, '/safety/state', 10)
        self.mode = self.create_publisher(String, '/control_mode', 10)
        self.enc = {a: self.create_publisher(Float32, f'/motor_{m}_position', 10)
                    for a, m in MOTOR.items()}
        self.deg = {a: 0.0 for a in MOTOR}          # 가짜 엔코더 값
        self.estop = False
        self.pos_cmd = {}                            # 축 → 마지막 위치 명령
        self.brakes = []                             # 브레이크 명령 이력
        self.status = None
        for a, j in JOINT.items():
            self.create_subscription(
                Float64MultiArray, f'/joint_{j}/position',
                (lambda k: (lambda m: self.pos_cmd.__setitem__(k, list(m.data))))(a), 10)
        self.create_subscription(String, '/brake_cmd',
                                 lambda m: self.brakes.append((time.time(), m.data)), 20)
        self.create_subscription(String, '/stage/status', self._on_status, 10)

    def _on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def pump(self, publish_refs=True):
        for a, p in self.enc.items():
            p.publish(Float32(data=self.deg[a]))
        s = SafetyState(); s.estop = self.estop
        self.saf.publish(s)
        self.mode.publish(String(data=json.dumps({'mode': 'auto', 'owner': 'stage_node'})))
        if publish_refs:
            self.homing.publish(String(data=json.dumps(
                {'state': 'done', 'refs': {'x': 0.0, 'y': 0.0, 'z': 0.0}})))

    def spin(self, sec, refs=True, drive=None):
        """drive 가 주어지면 위치 명령을 따라 가짜 엔코더를 움직인다 (모터 흉내)."""
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            self.pump(refs)
            if drive:
                for a, cmd in list(self.pos_cmd.items()):
                    tgt = cmd[0]
                    d = tgt - self.deg[a]
                    step = max(-drive, min(drive, d))
                    self.deg[a] += step
            rclpy.spin_once(self, timeout_sec=0.02)


def check(name, cond, detail=''):
    print(f"  {'통과' if cond else '실패'}  {name}" + (f"   {detail}" if detail else ''))
    return cond


def main():
    rclpy.init()
    h = H()
    ok = True
    h.spin(1.5, refs=False)
    if h.status is None:
        print("실패 — /stage/status 를 못 받았다. stage_node 가 떠 있나?")
        rclpy.shutdown(); return 1

    print("■ 호밍 원점이 없을 때")
    h.pos_cmd.clear()
    h.goal.publish(Point(x=10.0, y=NAN, z=NAN))
    h.spin(1.0, refs=False)
    ok &= check("mm 목표를 거부한다", not h.pos_cmd,
                f"detail={h.status.get('detail')}")

    print("■ 호밍 원점을 주고 다시 (mm_per_deg 는 여전히 미측정)")
    h.spin(1.0)                                   # refs 발행
    h.pos_cmd.clear()
    h.goal.publish(Point(x=10.0, y=NAN, z=NAN))
    h.spin(1.0)
    ok &= check("mm_per_deg 미측정이라 여전히 거부", not h.pos_cmd,
                f"detail={h.status.get('detail')}")

    print("■ 각도 목표는 환산 없이 받는다")
    h.pos_cmd.clear(); h.brakes.clear()
    t0 = time.time()
    h.goal_deg.publish(Point(x=50.0, y=NAN, z=NAN))
    h.spin(0.5)                                    # ARM 구간(1초) 안
    first_brake = next((t for t, d in h.brakes if d.startswith('release x')), None)
    ok &= check("브레이크 해제가 먼저 나간다", first_brake is not None)
    ok &= check("ARM 동안에는 축 명령이 없다", not h.pos_cmd,
                f"pos_cmd={h.pos_cmd}")

    h.spin(1.2)
    first_cmd = 'x' in h.pos_cmd
    ok &= check("ARM 이 끝나면 축 명령이 나간다", first_cmd,
                f"cmd={h.pos_cmd.get('x')}")
    if first_cmd:
        ok &= check("목표 각도가 그대로 실린다", abs(h.pos_cmd['x'][0] - 50.0) < 1e-6,
                    f"{h.pos_cmd['x'][0]}")

    print("■ 모터를 흉내 내서 목표까지 움직인다")
    h.spin(4.0, drive=2.0)                         # 틱마다 2도씩
    arrived = h.status and not h.status.get('moving')
    ok &= check("도달하면 멈춘다", bool(arrived),
                f"moving={h.status.get('moving')} deg={h.deg['x']:.1f}")
    locked = any(d == 'lock x' for _, d in h.brakes)
    ok &= check("도달 후 브레이크를 잠근다", locked)

    print("■ 비상정지")
    h.deg['x'] = 0.0
    h.pos_cmd.clear(); h.brakes.clear()
    h.goal_deg.publish(Point(x=200.0, y=NAN, z=NAN))
    h.spin(1.5)
    moving = h.status.get('moving')
    h.estop = True
    h.spin(1.0)
    ok &= check("이동 중이었다가", bool(moving))
    ok &= check("비상정지로 멈춘다", not h.status.get('moving'),
                f"detail={h.status.get('detail')}")
    ok &= check("멈추면서 브레이크를 잠근다",
                any(d == 'lock x' for _, d in h.brakes))

    print(f"\n■ 결과: {'통과' if ok else '실패'}")
    h.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
