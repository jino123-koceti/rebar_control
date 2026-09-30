#!/usr/bin/env python3
"""Yaw 구동범위가 **모터 1회전 안에 드는지** 잰다.

## 왜 재는가

`0x92` 멀티턴 각도는 전원을 내리면 사라진다 — 앱솔루트가 아니다. 반면 싱글턴 엔코더는
앱솔루트라 전원을 다시 넣어도 값이 살아 있다. 그래서 **구동 전 범위가 모터 1회전
(360°, 출력축으로는 360/감속비) 안에 들면**, 자세 4개를 싱글턴 값으로 저장해 두고
재부팅 후에도 호밍 없이 자세를 알 수 있다. 1회전을 넘으면 그 방법을 못 쓴다.

## 먼저 스케일을 가려야 한다

발행되는 값이 모터축인지 출력축인지가 코드만 봐서는 안 갈린다 (0x92 는 0.01°/LSB 인데
그 각도의 기준축이 문서마다 다르다). 그래서 **건을 눈에 보이는 만큼 돌려서** 값이 얼마나
변하는지로 정한다. 90° 돌렸을 때:
    값이 약 90  변함  → 출력축 기준  (1회전 한계 = 360/12.5 = 28.8°)
    값이 약 1125 변함 → 모터축 기준  (1회전 한계 = 360°, 건 기준 28.8°)
둘 다 아니면 감속비 가정이 틀린 것이다.

## 쓰는 법

    python3 tools/test/yaw_range_check.py

띄워 둔 채로 yaw 를 **한쪽 끝까지 → 반대쪽 끝까지** 천천히 돌린다. 기계적으로 더 안
가는 지점까지다. 중간에 자세(12시·3시·6시·9시)에서 잠깐 멈추면 그 값도 같이 찍힌다.
Ctrl-C 로 끝내면 범위와 판정을 낸다.

⚠ yaw 는 리미트가 `yaw_home` 하나뿐이라 **끝을 센서가 안 알려준다.** 기구 끝에 부딪히기
  전에 사람이 멈춰야 한다. 천천히.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

GEAR = 12.5          # axes.yaml stage.gear — RMD-X4-10


class YawRange(Node):
    def __init__(self, mark_gap):
        super().__init__('yaw_range_check')
        self.pos = None
        self.lo = None
        self.hi = None
        self.home_hits = []          # yaw_home 이 켜진 순간의 값들
        self.marks = []              # 멈춰 있던 지점 (자세 후보)
        self.home = None
        self._still_since = None
        self._last_pos = None
        self._mark_gap = mark_gap
        self._last_print = ''
        self.create_subscription(Float32, '/motor_0x148_position', self._on_pos, 20)
        self.create_subscription(Bool, '/limit_sensors/yaw_home', self._on_home, 10)
        self.create_timer(0.5, self._tick)
        print("■ yaw 를 한쪽 끝 → 반대쪽 끝까지 천천히 돌리세요. Ctrl-C 로 요약.")
        print("  (자세 지점에서 2초 이상 멈추면 그 값을 따로 적어 둡니다)\n")

    def _on_pos(self, msg):
        v = float(msg.data)
        self.pos = v
        self.lo = v if self.lo is None else min(self.lo, v)
        self.hi = v if self.hi is None else max(self.hi, v)
        now = time.time()
        if self._last_pos is None or abs(v - self._last_pos) > 0.5:
            self._last_pos = v
            self._still_since = now
        elif self._still_since and now - self._still_since > 2.0:
            if not self.marks or abs(self.marks[-1] - v) > self._mark_gap:
                self.marks.append(v)
                print(f"    · 정지 지점 기록: {v:+.2f}")
            self._still_since = None

    def _on_home(self, msg):
        was = self.home
        self.home = bool(msg.data)
        if was is False and self.home and self.pos is not None:
            self.home_hits.append(self.pos)
            print(f"    ★ yaw_home 켜짐 — 값 {self.pos:+.2f}  (건 12시)")

    def _tick(self):
        if self.pos is None:
            return
        span = (self.hi - self.lo) if self.lo is not None else 0.0
        line = (f"  현재 {self.pos:+8.2f}   지금까지 범위 {self.lo:+.2f} ~ {self.hi:+.2f}"
                f"  (폭 {span:.2f})   home={'ON' if self.home else 'off'}")
        if line != self._last_print:
            print(line)
            self._last_print = line


def summarize(n):
    print("\n■ 요약")
    if n.lo is None:
        print("  위치 값을 못 받았습니다. /motor_0x148_position 이 오는지 확인하세요.")
        return
    span = n.hi - n.lo
    print(f"  관측 범위 : {n.lo:+.2f} ~ {n.hi:+.2f}   폭 {span:.2f}")
    if n.home_hits:
        print(f"  yaw_home(12시) 값 : " + ", ".join(f"{v:+.2f}" for v in n.home_hits))
        if len(n.home_hits) > 1:
            spread = max(n.home_hits) - min(n.home_hits)
            print(f"    → 반복 편차 {spread:.2f}  (클수록 센서 위치 재현성이 나쁘다)")
    if n.marks:
        print("  정지 지점(자세 후보) : " + ", ".join(f"{v:+.2f}" for v in n.marks))

    print("\n■ 1회전 판정 — 값의 기준축을 모르므로 양쪽 다 낸다")
    print(f"  · 값이 **모터축** 도라면   : 한계 360°     → 폭 {span:.1f}° "
          f"{'안에 든다 ✅' if span < 360 else '넘는다 ❌'}")
    lim_out = 360.0 / GEAR
    print(f"  · 값이 **출력축** 도라면   : 한계 {lim_out:.1f}° (감속비 {GEAR}) → 폭 {span:.1f}° "
          f"{'안에 든다 ✅' if span < lim_out else '넘는다 ❌'}")
    print("\n  기준축을 가르려면: 건을 눈으로 90° 돌렸을 때 값이")
    print(f"    ~90 변함  → 출력축 기준 /  ~{90*GEAR:.0f} 변함 → 모터축 기준")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mark-gap', type=float, default=5.0,
                    help='이 값보다 가까운 정지 지점은 같은 자세로 본다')
    a = ap.parse_args()
    rclpy.init()
    n = YawRange(a.mark_gap)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    summarize(n)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
