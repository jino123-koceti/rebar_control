#!/usr/bin/env python3
"""카메라 좌표 → **그 자세에서 보낼 스테이지 위치(mm)** 를 실측으로 뜬다.

## 무엇을 푸는가

`crossing_detector` 는 교차점을 **카메라 좌표(mm)** 로 낸다. 결속하려면 "그 점에
건 끝을 두려면 스테이지를 어디로 보내야 하나" 를 알아야 한다.

    stage_mm = A · P_cam + b        A 는 3x3, b 는 3  → 미지수 12개 (자세당)

⚠⚠ **자세마다 따로 뜬다.** 건이 yaw 축에 달려 있어 자세를 바꾸면 건 끝이 X·Y 로
움직인다. 자세 범위가 건 33.4° 이므로 건 끝이 축에서 100mm 떨어져 있으면 자세에
따라 약 58mm 차이가 난다 — 하나로 뭉치면 그만큼 틀린다.

⚠ 이것은 **좌표계 변환이 아니다.** 같은 세상의 점이라도 자세가 다르면 보낼
스테이지 위치가 다르다. 그래서 결과를 "stage 좌표" 라 부르지 않고 **"그 자세에서
보낼 위치"** 라 부른다.

## 왜 mm 인가 (각도가 아니라)

옛 버전은 **절대 모터 각도**로 떴고 "호밍 원점이 바뀌면 다시 떠야 한다" 는 경고가
붙어 있었다. 멀티턴은 전원에 날아가므로 절대 각도는 전원 세션 안에서만 산다.
mm 는 **원점 기준**이라 재호밍·전원 재투입을 견딘다 (원점 반복오차 ~0.4mm 만 남는다).
`/tying/goal` 과 자세별 가동범위·사분면 규칙도 전부 mm 라 그대로 맞물린다.

`/stage/status` 의 `current_mm` 을 그대로 쓴다 — `stage_node` 가 이미 원점과
`mm_per_deg` 로 환산해 발행한다. 여기서 또 환산하면 두 곳이 어긋난다.

## 절차

  1. 호밍 (`/homing_cmd` → `all`). 원점이 없으면 mm 가 안 나온다
  2. 이 도구를 띄운다. 현재 자세가 **판별되어 있어야** 한다
     (안 되면 `/stage/yaw_declare` 로 알려줄 것 — 1번·4번은 단회전이 모호하다)
  3. `s` — 지금 보이는 교차점을 번호와 함께 고정한다
  4. **리모콘으로 건 끝을 그 점에 맞춘다** (X·Y·Z 다)
  5. `r <번호>` — 그 순간의 스테이지 mm 를 짝으로 기록한다
  6. 3~5 를 **자세당 최소 4점, 권장 6점** 반복
  7. 리모콘 **S23/S24** 로 다음 자세로 옮기고 3~6 반복 (1~4번 전부)
  8. `f` — 자세별로 최소제곱을 풀고 yaml 로 저장한다

⚠ 점들이 **한 줄로 늘어서면 안 된다.** 시야 안에서 좌우·앞뒤로 흩어진 점을 고를 것.
⚠ **깊이(z)도 흩어져야 한다** — A 가 3x3 이라 z 가 거의 같으면 그 열이 결정되지
  않는다. 철근 배근이 기울어 있어 보통은 349~624mm 로 충분히 퍼진다.
⚠ 4점은 미지수와 같은 수라 잔차가 0 으로 나와 정확도 확인이 안 된다.

명령: s=고정  r <n>=기록  p <n>=예측검증  l=목록  u=마지막취소  f=산출·저장  q=종료
"""

import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from rebar_base_interfaces.msg import RebarGrid

OUT = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/stage_camera.yaml')
AXES = ('x', 'y', 'z')


class Calib(Node):
    def __init__(self):
        super().__init__('stage_camera_calib')
        self.stage = None
        self.grid = None
        self.create_subscription(String, '/stage/status', self._on_stage, 10)
        self.create_subscription(RebarGrid, '/rebar/crossings', self._on_grid, 10)
        self.trig = self.create_publisher(Empty, '/rebar/detect', 10)
        self.frozen = []          # 고정한 검출점 [(x,y,z,conf), ...]
        self.pairs = {}           # 자세 → [(P_cam(3), stage_mm(3)), ...]

    def _on_stage(self, msg):
        try:
            self.stage = json.loads(msg.data)
        except ValueError:
            pass

    def _on_grid(self, msg):
        self.grid = msg

    def spin(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---- 현재 상태 ---------------------------------------------------------
    def pose(self):
        return (self.stage or {}).get('pose')

    def mm(self):
        c = (self.stage or {}).get('current_mm') or {}
        v = [c.get(a) for a in AXES]
        return None if any(x is None for x in v) else v

    def show(self):
        s = self.stage or {}
        p, g = s.get('pose'), s.get('gun_deg')
        m = self.mm()
        print(f"  자세 {p if p is not None else '미확인'}"
              f"  건 {g if g is not None else '?'}°"
              f"  위치 " + (f"X {m[0]:.1f} Y {m[1]:.1f} Z {m[2]:.1f} mm" if m else '없음'))
        if p is None:
            print("  ⚠ 자세가 판별되지 않았습니다 — /stage/yaw_declare 로 알려주세요")
        if m is None:
            print("  ⚠ mm 를 못 받습니다 — 호밍이 필요합니다 (/homing_cmd all)")

    # ---- 동작 --------------------------------------------------------------
    def snapshot(self):
        self.grid = None
        self.trig.publish(Empty())
        t = time.time() + 6.0
        while rclpy.ok() and self.grid is None and time.time() < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.grid is None:
            print("  검출 결과를 못 받았습니다 — crossing_detector 가 떠 있나요?")
            return
        self.frozen = [(d.x, d.y, d.z, d.confidence) for d in self.grid.detections]
        print(f"  검출 {len(self.frozen)}개  ({self.grid.error_message})")
        if not self.frozen:
            print("  깊이가 확보된 검출이 없습니다. 시야·조명을 확인하세요.")
        for i, (x, y, z, c) in enumerate(self.frozen):
            print(f"    [{i}] 카메라 ({x:+8.1f}, {y:+8.1f}, {z:8.1f}) mm  신뢰도 {c:.2f}")

    def record(self, idx):
        if not (0 <= idx < len(self.frozen)):
            print(f"  번호가 범위를 벗어났습니다 (0~{len(self.frozen) - 1})")
            return
        self.spin(0.8)
        p, m = self.pose(), self.mm()
        if p is None:
            print("  자세를 모릅니다 — 기록하지 않습니다 "
                  "(/stage/yaw_declare 로 알려주세요)")
            return
        if m is None:
            print("  스테이지 mm 를 못 받습니다 — 기록하지 않습니다 (호밍 필요)")
            return
        x, y, z, _ = self.frozen[idx]
        self.pairs.setdefault(p, []).append(([x, y, z], m))
        print(f"  {p}번 자세 짝 {len(self.pairs[p])} 기록 — "
              f"카메라 ({x:+.1f},{y:+.1f},{z:.1f}) ↔ "
              f"스테이지 (X {m[0]:.1f}, Y {m[1]:.1f}, Z {m[2]:.1f}) mm")

    def undo(self):
        p = self.pose()
        if p is None or not self.pairs.get(p):
            print("  취소할 짝이 없습니다 (지금 자세 기준)")
            return
        self.pairs[p].pop()
        print(f"  {p}번 자세 마지막 짝 취소 — 남은 {len(self.pairs[p])}개")

    def listing(self):
        if not self.pairs:
            print("  기록된 짝이 없습니다")
            return
        for p in sorted(self.pairs):
            rows = self.pairs[p]
            zs = [q[0][2] for q in rows]
            print(f"  {p}번 자세: {len(rows)}개"
                  + (f"  (카메라 z {min(zs):.0f}~{max(zs):.0f}mm)" if rows else ''))
            for i, (c, m) in enumerate(rows):
                print(f"     {i} cam({c[0]:+7.1f},{c[1]:+7.1f},{c[2]:7.1f})"
                      f" → stage({m[0]:7.1f},{m[1]:7.1f},{m[2]:7.1f})")

    # ---- 산출 --------------------------------------------------------------
    def fit_one(self, rows):
        """(A 3x3, b 3, rms, worst) 또는 None."""
        if len(rows) < 4:
            return None
        P = np.array([c for c, _ in rows], dtype=float)        # (n,3)
        S = np.array([m for _, m in rows], dtype=float)        # (n,3)
        M = np.hstack([P, np.ones((len(rows), 1))])            # (n,4)
        sol, *_ = np.linalg.lstsq(M, S, rcond=None)            # (4,3)
        A, b = sol[:3, :].T, sol[3, :]
        res = (A @ P.T).T + b - S
        return (A, b,
                float(np.sqrt((res ** 2).sum(axis=1).mean())),
                float(np.abs(res).max()))

    def fit(self, out_path):
        done = {}
        print()
        for p in sorted(self.pairs):
            rows = self.pairs[p]
            r = self.fit_one(rows)
            if r is None:
                print(f"■ {p}번 자세 — 짝 {len(rows)}개뿐 (미지수 12개, 최소 4점)")
                continue
            A, b, rms, worst = r
            done[p] = (A, b, rms, worst, len(rows))
            print(f"■ {p}번 자세 — 짝 {len(rows)}개   잔차 RMS {rms:.2f}mm  "
                  f"최대 {worst:.2f}mm")
            zs = [q[0][2] for q in rows]
            if max(zs) - min(zs) < 30.0:
                print(f"   ⚠ 카메라 z 범위가 {max(zs)-min(zs):.0f}mm 뿐입니다 — "
                      "A 의 z 열이 결정되지 않아 깊이가 다른 점에서 틀립니다")
            if len(rows) == 4:
                print("   ⚠ 4점은 미지수와 같은 수라 잔차가 0 으로 나옵니다 — "
                      "정확도 확인이 안 됩니다")
            elif rms > 10.0:
                print("   ⚠ 잔차가 큽니다. 맞춘 점이 틀렸거나 점이 한 줄로 늘어서 "
                      "있을 수 있습니다")
        if not done:
            print("  저장할 것이 없습니다")
            return
        missing = [p for p in (1, 2, 3, 4) if p not in done]
        if missing:
            print(f"\n  ⚠ 아직 안 뜬 자세: {missing} — 그 자세에서는 결속할 수 "
                  "없습니다 (자세마다 따로 떠야 합니다)")

        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write("# 카메라 좌표(mm) → **그 자세에서 보낼 스테이지 위치(mm)**\n")
            f.write("#   stage_mm = A @ P_cam + b      (A 3x3, b 3)\n")
            f.write("#   [0]=X [1]=Y [2]=Z, 전부 **호밍 원점 기준 mm**\n")
            f.write("# ⚠ 좌표계 변환이 아니다 — 같은 점이라도 자세가 다르면 보낼\n")
            f.write("#   위치가 다르다 (건이 yaw 축에 달려 회전한다). 그래서 자세별이다.\n")
            f.write("# ⚠ mm 는 원점 기준이라 재호밍·전원 재투입을 견딘다. 다만 원점\n")
            f.write("#   반복오차(~0.4mm)는 남는다.\n")
            f.write(f"# 실측 {time.strftime('%Y-%m-%d %H:%M')}\n")
            f.write("poses:\n")
            for p in sorted(done):
                A, b, rms, worst, n = done[p]
                f.write(f"  {p}:\n")
                f.write(f"    A: {json.dumps([[round(v, 6) for v in r] for r in A.tolist()])}\n")
                f.write(f"    b: {json.dumps([round(v, 4) for v in b.tolist()])}\n")
                f.write(f"    pairs: {n}\n")
                f.write(f"    rms_mm: {rms:.3f}\n")
                f.write(f"    max_mm: {worst:.3f}\n")
            f.write("raw:\n")
            for p in sorted(self.pairs):
                f.write(f"  {p}:\n")
                for c, m in self.pairs[p]:
                    f.write(f"    - cam: {json.dumps([round(v, 2) for v in c])}\n")
                    f.write(f"      stage: {json.dumps([round(v, 2) for v in m])}\n")
        print(f"\n  저장: {out_path}")

    def predict(self, idx):
        p = self.pose()
        rows = self.pairs.get(p) or []
        r = self.fit_one(rows)
        if r is None:
            print(f"  {p}번 자세 짝이 모자랍니다 (최소 4점)")
            return
        if not (0 <= idx < len(self.frozen)):
            print(f"  번호가 범위를 벗어났습니다 (0~{len(self.frozen) - 1})")
            return
        A, b, rms, _ = r
        x, y, z, _ = self.frozen[idx]
        q = A @ np.array([x, y, z]) + b
        m = self.mm()
        print(f"  [{idx}] 예측 보낼 위치  X {q[0]:.1f}  Y {q[1]:.1f}  Z {q[2]:.1f} mm"
              f"   (자세 {p}, 잔차 RMS {rms:.2f}mm)")
        if m:
            d = q - np.array(m)
            print(f"       지금 위치와 차이  X {d[0]:+.1f}  Y {d[1]:+.1f}  Z {d[2]:+.1f} mm")


def main():
    rclpy.init()
    n = Calib()
    n.spin(2.5)
    print(__doc__.split('명령:')[0].rstrip())
    print("명령: s=고정  r <n>=기록  p <n>=예측  l=목록  u=취소  f=저장  q=종료\n")
    n.show()
    try:
        while rclpy.ok():
            n.spin(0.3)
            try:
                cmd = input("> ").strip().split()
            except EOFError:
                break
            if not cmd:
                n.show()
                continue
            c = cmd[0]
            if c == 'q':
                break
            elif c == 's':
                n.snapshot()
            elif c == 'r' and len(cmd) > 1:
                n.record(int(cmd[1]))
            elif c == 'p' and len(cmd) > 1:
                n.predict(int(cmd[1]))
            elif c == 'l':
                n.listing()
            elif c == 'u':
                n.undo()
            elif c == 'f':
                n.fit(OUT)
            else:
                print("  s / r <n> / p <n> / l / u / f / q")
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
