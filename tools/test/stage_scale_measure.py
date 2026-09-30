#!/usr/bin/env python3
"""축의 **도 → mm 환산**을 실측한다. `stage_node` 가 움직이려면 이 값이 필요하다.

## 왜 필요한가

교차점 검출은 mm 를 내고 축은 도로 움직인다. 그 사이 환산값(`mm_per_deg`)이
`axes.yaml` 에 `null`(미측정)로 비어 있고, 그 상태에서는 `stage_node` 가 이동을
거부한다 — 환산값 없이 움직이면 엉뚱한 거리를 간다.

감속비로 계산하지 않고 **실측하는 이유**: 리드스크류 피치·커플링·백래시가 계산과
다르다. 2차년도 주행 바퀴 반지름도 계산값이 아니라 실측(183mm/회전)으로 잡았다.

## 방법

축을 알려진 각도만큼 움직이고 **자로 실제 이동거리를 잰다.**

    python3 tools/test/stage_scale_measure.py x --deg 360

  1. 시작 위치에 표시를 해 둔다 (테이프·연필)
  2. 이 도구가 지정 각도만큼 움직인다 (엔코더로 실제 이동각을 확인한다)
  3. 자로 이동거리(mm)를 재서 입력한다
  4. `mm_per_deg` 를 계산해 준다 → `axes.yaml` 에 적는다

⚠ 왕복으로 두 번 재면 백래시를 알 수 있다. 한쪽으로만 재면 놓친다.
⚠ 리미트 근처에서 하지 말 것. 중간에서 여유를 두고 한다.
⚠ Z 는 브레이크를 풀면 떨어진다 — 이 도구는 이동 직전에 풀고 끝나면 바로 잠근다.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

AX = {'x': dict(joint=3, motor='0x145', gravity=False),
      'y': dict(joint=4, motor='0x146', gravity=False),
      'z': dict(joint=5, motor='0x147', gravity=True)}
GEAR = 12.5


class Mover(Node):
    def __init__(self, axis):
        super().__init__('stage_scale_measure')
        c = AX[axis]
        self.axis, self.cfg = axis, c
        self.spd = self.create_publisher(Float32, f"/joint_{c['joint']}/speed", 10)
        self.brake = self.create_publisher(String, '/brake_cmd', 10)
        self.deg = None
        self.create_subscription(Float32, f"/motor_{c['motor']}_position",
                                 lambda m: setattr(self, 'deg', m.data), 20)

    def spin(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def release(self):
        arg = f"release {self.axis}" + (' force' if self.cfg['gravity'] else '')
        for _ in range(3):
            self.brake.publish(String(data=arg)); self.spin(0.15)

    def lock(self):
        for _ in range(3):
            self.spd.publish(Float32(data=0.0))
            self.brake.publish(String(data=f"lock {self.axis}")); self.spin(0.1)

    def move_deg(self, target_deg, dps):
        """지정 각도(모터축)만큼 움직인다. 실제 이동각을 돌려준다."""
        self.spin(1.0)
        if self.deg is None:
            return None
        start = self.deg
        sign = 1.0 if target_deg >= 0 else -1.0
        need = abs(target_deg)
        t0 = time.time()
        limit = need / max(dps, 1.0) * 3.0 + 5.0      # 넉넉한 안전 시한
        while rclpy.ok():
            moved = abs(self.deg - start)
            if moved >= need or time.time() - t0 > limit:
                break
            self.spd.publish(Float32(data=sign * dps))
            rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(10):
            self.spd.publish(Float32(data=0.0)); self.spin(0.03)
        self.spin(0.6)                                 # 감속 반영
        return self.deg - start


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('axis', choices=sorted(AX))
    ap.add_argument('--deg', type=float, default=360.0,
                    help='움직일 각도 (모터축, 기본 360 = 출력축 %.1f°)' % (360 / GEAR))
    ap.add_argument('--dps', type=float, default=30.0)
    ap.add_argument('--back', action='store_true', help='되돌아와서 백래시도 잰다')
    a = ap.parse_args()

    rclpy.init()
    n = Mover(a.axis)
    print(f"■ {a.axis} 축  {a.deg:+.0f}° (모터축) 이동  속도 {a.dps:.0f} dps")
    print("  ⚠ 시작 위치에 표시를 해 두세요. 준비되면 Enter.", end=' ')
    try:
        input()
    except EOFError:
        pass

    n.release()
    moved = n.move_deg(a.deg, a.dps)
    n.lock()
    if moved is None:
        print("  실패 — 위치 토픽을 못 받았습니다 (position_control_node 확인)")
        n.destroy_node(); rclpy.shutdown(); return 1
    print(f"  엔코더 실제 이동: {moved:+.2f}° (모터축) = 출력축 {moved/GEAR:+.2f}°")

    try:
        mm = float(input("  자로 잰 이동거리(mm, 부호 없이): ").strip())
    except (ValueError, EOFError):
        print("  숫자를 못 읽었습니다. 중단."); n.destroy_node(); rclpy.shutdown(); return 1

    k = mm / abs(moved)
    print(f"\n■ mm_per_deg = {k:.6f}   (모터축 1도 = {k:.4f} mm)")
    print(f"   참고: 출력축 1도 = {k*GEAR:.4f} mm,  1mm = 모터축 {1/k:.2f}°")
    print(f"\n   axes.yaml 의 stage.{a.axis} 에 이렇게 적으세요:")
    print(f"     mm_per_deg: {k:.6f}   # [확정] 실측 {mm}mm / {abs(moved):.2f}° ({time.strftime('%Y-%m-%d')})")

    if a.back:
        print("\n■ 되돌아갑니다 — 원래 표시까지의 거리를 재면 백래시를 알 수 있습니다.")
        input("  준비되면 Enter.")
        n.release()
        back = n.move_deg(-a.deg, a.dps)
        n.lock()
        print(f"  엔코더 복귀 이동: {back:+.2f}°  (왕복 합 {moved+back:+.2f}° — 0 에서 멀수록 백래시)")

    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
