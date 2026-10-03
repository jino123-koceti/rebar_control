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
# 짝을 모으는 중간 상태. **한 명령씩 따로 띄워도 이어지도록** 파일에 남긴다
# (`--cmd` 비대화 모드용). 대화 모드에서도 같은 파일을 쓴다 — 중간에 끊겨도
# 다시 띄우면 이어서 할 수 있다. 자세당 6점을 손으로 맞추는 일이라 날리면 아깝다.
STATE = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/stage_camera_pairs.json')
# 고정할 때마다 **번호가 찍힌 그림**을 남긴다. 숫자만 보고는 어느 교차점인지
# 알 수 없어 사람이 건 끝을 맞출 수 없다. 그림과 번호가 **같은 스냅샷**에서
# 나와야 어긋나지 않으므로 도구가 직접 그린다 (따로 그리면 정렬 순서가 다르다).
SNAP = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/snapshot.png')
AXES = ('x', 'y', 'z')


class Calib(Node):
    def __init__(self):
        super().__init__('stage_camera_calib')
        self.stage = None
        self.grid = None
        self.create_subscription(String, '/stage/status', self._on_stage, 10)
        self.create_subscription(RebarGrid, '/rebar/crossings', self._on_grid, 10)
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self.color = None
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._on_color, qos_profile_sensor_data)
        self.trig = self.create_publisher(Empty, '/rebar/detect', 10)
        self.frozen = []          # 고정한 검출점 [(x,y,z,conf), ...]
        self.pairs = {}           # 자세 → [(P_cam(3), stage_mm(3)), ...]
        self._load_state()

    # ---- 중간 상태 ---------------------------------------------------------
    def _load_state(self):
        try:
            d = json.load(open(STATE, encoding='utf-8'))
            self.pairs = {int(k): [(c, m) for c, m in v]
                          for k, v in (d.get('pairs') or {}).items()}
            self.frozen = [tuple(f) for f in (d.get('frozen') or [])]
        except Exception:
            pass

    def _save_state(self):
        os.makedirs(os.path.dirname(STATE), exist_ok=True)
        json.dump({'pairs': {str(k): v for k, v in self.pairs.items()},
                   'frozen': [list(f) for f in self.frozen]},
                  open(STATE, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)

    def _on_stage(self, msg):
        try:
            self.stage = json.loads(msg.data)
        except ValueError:
            pass

    def _on_grid(self, msg):
        self.grid = msg

    def _on_color(self, msg):
        self.color = msg

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
        self.frozen = [(d.x, d.y, d.z, d.confidence, d.pixel_u, d.pixel_v)
                       for d in self.grid.detections]
        print(f"  검출 {len(self.frozen)}개  ({self.grid.error_message})")
        if not self.frozen:
            print("  깊이가 확보된 검출이 없습니다. 시야·조명을 확인하세요.")
        self._save_state()
        for i, (x, y, z, c, u, v) in enumerate(self.frozen):
            print(f"    [{i}] 카메라 ({x:+8.1f}, {y:+8.1f}, {z:8.1f}) mm  "
                  f"픽셀 ({u:4d},{v:4d})  신뢰도 {c:.2f}")
        self._draw()

    def _draw(self):
        """번호가 찍힌 그림을 남긴다. 번호는 위 목록과 **같은 순서**다."""
        if self.color is None or not self.frozen:
            print("  (그림을 못 그렸습니다 — 컬러 영상 없음)")
            return
        try:
            import cv2
            import numpy as np
            c = self.color
            img = np.frombuffer(c.data, dtype=np.uint8).reshape(
                c.height, c.width, 3)[:, :, ::-1].copy()
            # 깊이 층을 색으로 구분한다 — z 가 퍼진 점을 고르게 돕는다
            zs = [f[2] for f in self.frozen]
            lo, hi = min(zs), max(zs)
            for i, (x, y, z, cf, u, v) in enumerate(self.frozen):
                t = 0.0 if hi - lo < 1 else (z - lo) / (hi - lo)
                col = (int(60 + 195 * t), 220, int(255 - 195 * t))   # 가까움→멀음
                cv2.circle(img, (u, v), 18, col, 3)
                cv2.circle(img, (u, v), 2, (0, 0, 255), -1)
                cv2.putText(img, str(i), (u - 9, v + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
                cv2.putText(img, str(i), (u - 9, v + 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
                cv2.putText(img, f"{z:.0f}", (u + 22, v + 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
            s = self.stage or {}
            hdr = (f"pose {s.get('pose')}  gun {s.get('gun_deg')}deg   "
                   f"{len(self.frozen)} crossings   z {lo:.0f}-{hi:.0f}mm   "
                   f"(number = index, small = camera z)")
            cv2.rectangle(img, (0, 0), (c.width, 32), (0, 0, 0), -1)
            cv2.putText(img, hdr, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)
            os.makedirs(os.path.dirname(SNAP), exist_ok=True)
            cv2.imwrite(SNAP, img)
            print(f"  그림: {SNAP}")
        except Exception as e:
            print(f"  (그림 저장 실패: {e})")

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
        x, y, z = self.frozen[idx][:3]
        self.pairs.setdefault(p, []).append(([x, y, z], m))
        self._save_state()
        print(f"  {p}번 자세 짝 {len(self.pairs[p])} 기록 — "
              f"카메라 ({x:+.1f},{y:+.1f},{z:.1f}) ↔ "
              f"스테이지 (X {m[0]:.1f}, Y {m[1]:.1f}, Z {m[2]:.1f}) mm")

    def undo(self):
        p = self.pose()
        if p is None or not self.pairs.get(p):
            print("  취소할 짝이 없습니다 (지금 자세 기준)")
            return
        self.pairs[p].pop()
        self._save_state()
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
    def fit_shared(self):
        """**공통 변환 + 자세별 오프셋** 으로 한 번에 푼다.

        카메라→세상 변환은 **모든 자세에서 같다** — 카메라가 고정이고 교차점도
        움직이지 않는다. 자세마다 바뀌는 것은 **건 끝 오프셋 하나**뿐이다
        (캐리지는 회전하지 않고 평행이동만 한다):

            보낼 위치 = A·P_cam + b + offset(자세)      offset(첫 자세) = 0

        그래서 미지수가 자세별 독립(자세당 12개)보다 훨씬 적다 — 자세 4개면
        12 + 3x3 = **21개**이고, 점 하나가 식 3개를 주니 7점이면 풀린다.
        자세별 독립은 16점이 필요하다.

        ⚠ 오프셋을 **자유 벡터**로 둔다 (회전으로 모델링하지 않는다). 실측에서
        자세 1→2 는 거의 X 로만, 2→3 은 거의 Y 로만 움직였다 — 단순 회전이라면
        변위 방향이 각도만큼만 돌아야 하는데 80° 가까이 돌았다. 자유 벡터는
        그게 무엇이든 흡수한다.

        ⚠ Z 오프셋이 0 에 가까운지 보라 — yaw 회전은 높이를 바꾸지 않아야 한다.
        크게 나오면 정렬이 자세마다 어긋났다는 뜻이다 (실측 -0.9 / -3.0mm).
        """
        poses = sorted(self.pairs)
        n = sum(len(v) for v in self.pairs.values())
        if not poses or n < 1:
            return None
        base, extra = poses[0], poses[1:]
        U = 12 + 3 * len(extra)
        if n * 3 < U:
            return None
        M = np.zeros((n * 3, U))
        r = np.zeros(n * 3)
        row = 0
        for p in poses:
            for cam, st in self.pairs[p]:
                P = np.array(cam, dtype=float)
                for k in range(3):
                    M[row + k, k * 3:(k + 1) * 3] = P
                    M[row + k, 9 + k] = 1.0
                    if p in extra:
                        M[row + k, 12 + 3 * extra.index(p) + k] = 1.0
                    r[row + k] = st[k]
                row += 3
        sol, _, rank, _ = np.linalg.lstsq(M, r, rcond=None)
        if rank < U:
            return None                 # 점 배치가 겹쳐 결정되지 않는다
        e = ((M @ sol) - r).reshape(-1, 3)
        off = {base: np.zeros(3)}
        for i, p in enumerate(extra):
            off[p] = sol[12 + 3 * i:15 + 3 * i]
        return dict(A=sol[:9].reshape(3, 3), b=sol[9:12], off=off,
                    rms=float(np.sqrt((e ** 2).sum(axis=1).mean())),
                    worst=float(np.abs(e).max()), n=n, dof=n * 3 - U)

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
        """두 모델을 **다 풀어 비교**하고, 공통 모델을 기준으로 저장한다.

        자세별 독립은 자세당 4점이면 **잔차가 0** 으로 나와 정확도 확인이 안 되고
        과적합하기 쉽다. 공통 모델은 모든 점을 함께 쓰므로 검증 여유가 남는다.
        그래서 **공통 모델을 기준**으로 삼고 독립은 교차검증으로만 쓴다.
        독립이 확실히 더 좋으면(점이 넉넉하고 잔차가 뚜렷이 작으면) 그게 기구에
        공통 모델로 설명 안 되는 것이 있다는 신호다 — 그때 사람이 판단한다.
        """
        sh = self.fit_shared()
        print()
        if sh:
            print(f"■ 공통 변환 + 자세별 오프셋 — 짝 {sh['n']}개, 검증 여유 {sh['dof']}")
            print(f"   잔차 RMS {sh['rms']:.2f}mm  최대 {sh['worst']:.2f}mm"
                  + ("   ⚠ 미지수=식 이라 0 이 당연 (검증 안 됨)"
                     if sh['dof'] == 0 else ""))
            for p in sorted(sh['off']):
                o = sh['off'][p]
                tag = ' ← 기준' if not o.any() else ''
                print(f"     {p}번 오프셋 ({o[0]:+7.1f}, {o[1]:+7.1f}, {o[2]:+6.1f}) mm{tag}")
            zmax = max(abs(o[2]) for o in sh['off'].values())
            if zmax > 10.0:
                print(f"   ⚠ Z 오프셋이 {zmax:.1f}mm 입니다 — yaw 회전은 높이를 바꾸지 "
                      "않아야 합니다. 자세마다 맞추는 기준이 달랐을 수 있습니다")
        else:
            need = 12 + 3 * max(0, len(self.pairs) - 1)
            got = sum(len(v) for v in self.pairs.values()) * 3
            print(f"■ 공통 변환 — 아직 못 풉니다 (식 {got} / 미지수 {need}, "
                  f"또는 점 배치가 겹칩니다)")
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
        if not done and not sh:
            print("  저장할 것이 없습니다")
            return
        usable = set(sh['off']) if sh else set(done)
        missing = [p for p in (1, 2, 3, 4) if p not in usable]
        if missing:
            print(f"\n  ⚠ 아직 쓸 수 없는 자세: {missing} — 공통 모델이라도 그 자세 "
                  "짝이 **1점** 은 있어야 오프셋이 나옵니다")

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
            # 소비자가 쓰는 값. 모델이 무엇이든 자세별 (A, b) 한 쌍으로 낸다
            f.write("# model: shared = 공통 변환 + 자세별 오프셋 (기준).\n")
            f.write("#   independent = 자세별 독립 (교차검증용, 자세당 4점 이상).\n")
            f.write("poses:\n")
            if sh:
                for p in sorted(sh['off']):
                    A = sh['A']
                    b = sh['b'] + sh['off'][p]
                    f.write(f"  {p}:\n")
                    f.write(f"    A: {json.dumps([[round(v, 6) for v in r] for r in A.tolist()])}\n")
                    f.write(f"    b: {json.dumps([round(v, 4) for v in b.tolist()])}\n")
                    f.write(f"    pairs: {len(self.pairs.get(p, []))}\n")
                    f.write(f"    source: shared\n")
                f.write(f"shared:\n")
                f.write(f"  pairs: {sh['n']}\n")
                f.write(f"  dof: {sh['dof']}\n")
                f.write(f"  rms_mm: {sh['rms']:.3f}\n")
                f.write(f"  max_mm: {sh['worst']:.3f}\n")
                f.write("  offsets:\n")
                for p in sorted(sh['off']):
                    o = sh['off'][p]
                    f.write(f"    {p}: {json.dumps([round(v, 3) for v in o.tolist()])}\n")
            f.write("independent:   # 교차검증용. 기준이 아니다\n")
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
        x, y, z = self.frozen[idx][:3]
        q = A @ np.array([x, y, z]) + b
        m = self.mm()
        print(f"  [{idx}] 예측 보낼 위치  X {q[0]:.1f}  Y {q[1]:.1f}  Z {q[2]:.1f} mm"
              f"   (자세 {p}, 잔차 RMS {rms:.2f}mm)")
        if m:
            d = q - np.array(m)
            print(f"       지금 위치와 차이  X {d[0]:+.1f}  Y {d[1]:+.1f}  Z {d[2]:+.1f} mm")


def run_one(n, cmd):
    c = cmd[0] if cmd else ''
    if c == 's':
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
    elif c == '':
        n.show()
    else:
        print("  s / r <n> / p <n> / l / u / f / q")
        return False
    return True


def main():
    import argparse
    ap = argparse.ArgumentParser()
    # 한 명령만 돌리고 끝낸다. 짝은 STATE 파일에 남아 다음 호출로 이어진다 —
    # 사람이 리모콘으로 맞추는 동안 터미널을 붙잡고 있을 필요가 없다
    ap.add_argument('--cmd', help='비대화 모드: "s" | "r 3" | "l" | "u" | "f" | "p 3"')
    a = ap.parse_args()

    rclpy.init()
    n = Calib()
    n.spin(2.5)
    if a.cmd is not None:
        n.show()
        run_one(n, a.cmd.strip().split())
        n.destroy_node()
        rclpy.shutdown()
        return 0
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
            if cmd[0] == 'q':
                break
            run_one(n, cmd)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
