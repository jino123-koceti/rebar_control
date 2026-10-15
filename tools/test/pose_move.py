#!/usr/bin/env python3
"""XY 를 중앙으로 옮기고 자세를 바꾼다. 거부/정지를 보고 바로 멈춘다.

⚠ Z 는 건드리지 않는다 (NaN). 회전 전 중앙 이동은 사용자 안전 규칙이다.
⚠ `rejects` 가 늘면 즉시 중단한다 — 보냈다고 믿고 넘어가면 안 된다.
"""
import argparse, json, math, sys, time
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32, String

NAN = float('nan')


class Mover(Node):
    def __init__(self):
        super().__init__('pose_move')
        self.st = None
        self.create_subscription(String, '/stage/status', self._s, 10)
        self.goal = self.create_publisher(Point, '/stage/goal', 10)
        self.yaw = self.create_publisher(Int32, '/stage/yaw_pose', 10)

    def _s(self, m):
        try:
            self.st = json.loads(m.data)
        except ValueError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--pose', type=int, required=True)
    a = ap.parse_args()

    rclpy.init()
    n = Mover()

    def pump(s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(n, timeout_sec=0.05)

    # 상태와 구독자를 먼저 확보한다 (디스커버리 경쟁 방지)
    t0 = time.time()
    while (n.st is None or n.goal.get_subscription_count() == 0) and time.time() - t0 < 8:
        pump(0.2)
    if n.st is None:
        print("  ⚠ /stage/status 가 안 온다"); return 1
    rej0 = n.st.get('rejects')
    mm = n.st.get('current_mm', {})
    print(f"  시작: X {mm.get('x')} Y {mm.get('y')} Z {mm.get('z')}  {n.st.get('pose_detail')}")

    def wait_done(what, limit):
        t = time.time()
        moved = False
        while time.time() - t < limit:
            pump(0.1)
            if n.st.get('rejects') != rej0:
                print(f"  ⚠ 거부 — {n.st.get('detail')}"); return False
            if n.st.get('moving'):
                moved = True
            elif moved:
                return True
        print(f"  ⚠ {what} 시간초과 ({limit:.0f}s) — detail: {n.st.get('detail')}")
        return False

    # ① 중앙으로 (Z 유지)
    print(f"  ① XY 중앙 이동 → X {a.x} Y {a.y}")
    n.goal.publish(Point(x=a.x, y=a.y, z=NAN))
    if not wait_done('이동', 40.0):
        return 1
    mm = n.st.get('current_mm', {})
    print(f"     도착 X {mm.get('x')} Y {mm.get('y')} Z {mm.get('z')}")

    # ② 자세 회전
    if n.st.get('pose') == a.pose:
        print(f"  ② 이미 {a.pose}번 자세다 — 회전 생략")
    else:
        print(f"  ② {a.pose}번 자세로 회전")
        n.yaw.publish(Int32(data=a.pose))
        if not wait_done('회전', 60.0):
            return 1
        # 정착(재보정)까지 기다린다 — yaw_moving 이 내려가야 끝이다
        t = time.time()
        while time.time() - t < 20 and n.st.get('yaw_moving'):
            pump(0.1)
    mm = n.st.get('current_mm', {})
    print(f"  끝: X {mm.get('x')} Y {mm.get('y')} Z {mm.get('z')}  {n.st.get('pose_detail')}"
          f"  거부 {n.st.get('rejects')}")
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
