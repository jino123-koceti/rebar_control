#!/usr/bin/env python3
"""카메라 좌표 → 스테이지 축 각도 변환을 실측으로 뜬다 (간이 캘리브레이션).

## 무엇을 푸는가

`crossing_detector` 는 교차점을 **카메라 좌표(mm)** 로 낸다. 축은 **각도**로 움직인다.
그 사이를 잇는 변환이 없으면 "검출했는데 어디로 보내야 할지 모른다".

    stage_deg = A · P_cam + b        A 는 2x3, b 는 2  → 미지수 8개

카메라가 **고정 프레임**에 있으므로(2026-09-30 확인) 스테이지가 움직여도 교차점의
카메라 좌표는 변하지 않는다. 그래서 절대 위치로 짝지으면 된다.

**mm 가 아니라 각도로 맞추는 이유:** 변환이 mm↔도 환산까지 흡수한다. `mm_per_deg`
실측(자로 재기)을 건너뛸 수 있다. mm 는 나중에 사람이 읽기 좋으라고 재면 된다.

## 절차

  1. 호밍으로 원점을 잡는다 (`/homing_cmd` → `all`)
  2. 이 도구를 띄우고 `s` — 지금 보이는 교차점들을 번호와 함께 고정한다
  3. 번호를 고른다
  4. **리모콘으로 건 끝을 그 교차점에 정확히 맞춘다**
  5. `r` — 그 순간의 축 각도를 읽어 짝으로 기록한다
  6. 2~5 를 **최소 4점, 권장 5~6점** 반복한다
  7. `f` — 최소제곱으로 A,b 를 구하고 yaml 로 저장한다

⚠ 점들이 **한 줄로 늘어서면 안 된다.** 시야 안에서 좌우·앞뒤로 흩어진 점을 고를 것.
⚠ 4점은 미지수와 같은 수라 오차 확인이 안 된다. 5점 이상이어야 잔차가 의미를 갖는다.
⚠ Z(결속 깊이)는 성격이 달라 여기서 다루지 않는다. X/Y 만 맞춘다.
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32
from rebar_base_interfaces.msg import RebarGrid

OUT = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/stage_camera.yaml')
MOTOR = {'x': '0x145', 'y': '0x146'}


class Calib(Node):
    def __init__(self):
        super().__init__('stage_camera_calib')
        self.deg = {'x': None, 'y': None}
        for a, m in MOTOR.items():
            self.create_subscription(
                Float32, f'/motor_{m}_position',
                (lambda k: (lambda msg: self.deg.__setitem__(k, msg.data)))(a), 20)
        self.grid = None
        self.create_subscription(RebarGrid, '/rebar/crossings', self._on_grid, 10)
        self.trig = self.create_publisher(Empty, '/rebar/detect', 10)
        self.frozen = []          # 고정한 검출점 [(x,y,z,conf), ...]
        self.pairs = []           # [(P_cam(3), stage_deg(2)), ...]

    def _on_grid(self, msg):
        self.grid = msg

    def spin(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---- 동작 --------------------------------------------------------------
    def snapshot(self):
        self.grid = None
        self.trig.publish(Empty())
        t = time.time() + 5.0
        while rclpy.ok() and self.grid is None and time.time() < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.grid is None:
            print("  검출 결과를 못 받았습니다 — crossing_detector 가 떠 있나요?")
            return
        frame = self.grid.error_message
        self.frozen = [(d.x, d.y, d.z, d.confidence) for d in self.grid.detections]
        print(f"  검출 {len(self.frozen)}개  ({frame})")
        if not self.frozen:
            print("  깊이가 확보된 검출이 없습니다. 카메라 시야·조명을 확인하세요.")
        for i, (x, y, z, c) in enumerate(self.frozen):
            print(f"    [{i}] 카메라 ({x:+8.1f}, {y:+8.1f}, {z:8.1f}) mm  신뢰도 {c:.2f}")

    def record(self, idx):
        if not (0 <= idx < len(self.frozen)):
            print(f"  번호가 범위를 벗어났습니다 (0~{len(self.frozen)-1})")
            return
        self.spin(0.6)
        if any(v is None for v in self.deg.values()):
            print("  축 위치를 못 받고 있습니다 (position_control_node 확인)")
            return
        x, y, z, _ = self.frozen[idx]
        pair = ([x, y, z], [self.deg['x'], self.deg['y']])
        self.pairs.append(pair)
        print(f"  짝 {len(self.pairs)} 기록 — 카메라 ({x:+.1f},{y:+.1f},{z:.1f}) "
              f"↔ 축 (x {self.deg['x']:+.2f}°, y {self.deg['y']:+.2f}°)")

    def fit(self, out_path):
        n = len(self.pairs)
        if n < 4:
            print(f"  짝이 {n}개뿐입니다 — 미지수 8개라 **최소 4점** 필요합니다")
            return
        P = np.array([p for p, _ in self.pairs], dtype=float)      # (n,3)
        S = np.array([s for _, s in self.pairs], dtype=float)      # (n,2)
        M = np.hstack([P, np.ones((n, 1))])                        # (n,4)
        sol, *_ = np.linalg.lstsq(M, S, rcond=None)                # (4,2)
        A = sol[:3, :].T                                           # (2,3)
        b = sol[3, :]                                              # (2,)

        pred = (A @ P.T).T + b
        res = pred - S
        rms = float(np.sqrt((res ** 2).sum(axis=1).mean()))
        worst = float(np.abs(res).max())

        print(f"\n■ 변환 산출 (짝 {n}개)")
        print(f"  A =\n{np.array2string(A, precision=5)}")
        print(f"  b = {np.array2string(b, precision=3)}")
        print(f"  잔차 RMS {rms:.3f}°   최대 {worst:.3f}°")
        if n == 4:
            print("  ⚠ 4점은 미지수와 같은 수라 잔차가 0 으로 나옵니다 — 정확도 확인이 "
                  "안 됩니다. 5점 이상을 권합니다")
        elif rms > 2.0:
            print("  ⚠ 잔차가 큽니다. 점을 잘못 맞췄거나 한 줄로 늘어서 있을 수 있습니다")

        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write("# 카메라 좌표(mm) → 스테이지 축 각도 변환\n")
            f.write("#   stage_deg = A @ P_cam + b     (A 2x3, b 2)\n")
            f.write("#   [0]=x축(0x145), [1]=y축(0x146) 모터 각도(절대)\n")
            f.write("# 카메라는 고정 프레임에 있으므로 스테이지가 움직여도 P_cam 은 변하지 않는다.\n")
            f.write(f"# 실측 {time.strftime('%Y-%m-%d %H:%M')}  짝 {n}개  "
                    f"잔차 RMS {rms:.3f}° 최대 {worst:.3f}°\n")
            f.write("# ⚠ 호밍 원점이 바뀌면 다시 떠야 한다 (축 각도가 절대값이므로).\n")
            f.write(f"A: {json.dumps(A.tolist())}\n")
            f.write(f"b: {json.dumps(b.tolist())}\n")
            f.write(f"rms_deg: {rms:.4f}\n")
            f.write(f"max_deg: {worst:.4f}\n")
            f.write("pairs:\n")
            for p, sv in self.pairs:
                f.write(f"  - cam: {json.dumps([round(v,2) for v in p])}\n")
                f.write(f"    deg: {json.dumps([round(v,3) for v in sv])}\n")
        print(f"  저장: {out_path}")

    def predict(self, idx, A, b):
        x, y, z, _ = self.frozen[idx]
        d = A @ np.array([x, y, z]) + b
        print(f"  [{idx}] → 축 목표 x {d[0]:+.2f}°, y {d[1]:+.2f}°")
        print(f"      ros2 topic pub --once /stage/goal_deg geometry_msgs/Point "
              f"\"{{x: {d[0]:.3f}, y: {d[1]:.3f}, z: .nan}}\"")


HELP = """
  s        지금 보이는 교차점을 고정하고 번호를 매긴다
  r <번호> 리모콘으로 건 끝을 그 점에 맞춘 뒤, 지금 축 각도를 짝으로 기록
  l        기록한 짝 보기
  d <n>    n 번째 짝 지우기
  f        변환 산출 + 저장
  p <번호> (저장된 변환으로) 그 점의 축 목표를 계산해 본다
  q        끝내기
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()

    rclpy.init()
    n = Calib()
    print("■ 스테이지-카메라 간이 캘리브레이션")
    print("  먼저 호밍이 끝나 있어야 합니다 (축 각도가 원점 기준이어야 하므로).")
    print(HELP)
    A = b = None
    try:
        while rclpy.ok():
            n.spin(0.15)
            try:
                cmd = input("calib> ").strip().split()
            except EOFError:
                break
            if not cmd:
                continue
            c = cmd[0].lower()
            if c == 'q':
                break
            elif c == 's':
                n.snapshot()
            elif c == 'r' and len(cmd) > 1:
                n.record(int(cmd[1]))
            elif c == 'l':
                for i, (p, s) in enumerate(n.pairs):
                    print(f"  [{i}] cam ({p[0]:+.1f},{p[1]:+.1f},{p[2]:.1f}) "
                          f"↔ deg ({s[0]:+.2f},{s[1]:+.2f})")
            elif c == 'd' and len(cmd) > 1:
                i = int(cmd[1])
                if 0 <= i < len(n.pairs):
                    n.pairs.pop(i); print(f"  {i} 삭제")
            elif c == 'f':
                n.fit(a.out)
                try:
                    import yaml
                    d = yaml.safe_load(open(a.out, encoding='utf-8'))
                    A = np.array(d['A']); b = np.array(d['b'])
                except Exception:
                    pass
            elif c == 'p' and len(cmd) > 1 and A is not None:
                n.predict(int(cmd[1]), A, b)
            else:
                print(HELP)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
