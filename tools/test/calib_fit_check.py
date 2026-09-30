#!/usr/bin/env python3
"""캘리브레이션 최소제곱 산출을 검증한다 (ROS·하드웨어 불필요).

알려진 변환으로 가짜 짝을 만들고, `stage_camera_calib.py` 와 **같은 식**으로 풀어서
원래 변환이 복원되는지 본다. 잡음을 넣어 잔차가 의미 있게 커지는지도 본다.

확인하는 것
  · 잡음 없는 짝이면 변환이 정확히 복원되는가
  · 4점(미지수와 같은 수)에서 잔차가 0 으로 나오는가 — **정확도 확인이 안 된다는 증거**
  · 잡음을 넣으면 잔차가 그만큼 커지는가
  · 점이 한 줄로 늘어서면(퇴화) 결과가 무너지는가
"""

import sys

import numpy as np


def fit(P, S):
    """stage_camera_calib.py 의 fit 과 같은 식."""
    n = len(P)
    M = np.hstack([P, np.ones((n, 1))])
    sol, *_ = np.linalg.lstsq(M, S, rcond=None)
    A, b = sol[:3, :].T, sol[3, :]
    res = (A @ P.T).T + b - S
    rms = float(np.sqrt((res ** 2).sum(axis=1).mean()))
    return A, b, rms


def check(name, cond, detail=''):
    print(f"  {'통과' if cond else '실패'}  {name}" + (f"   {detail}" if detail else ''))
    return cond


def main():
    rng = np.random.default_rng(0)
    A_true = np.array([[0.20, -0.02, 0.005],
                       [0.01,  0.18, -0.003]])
    b_true = np.array([-30.0, 12.0])
    ok = True

    print("■ 잡음 없는 6점")
    P = rng.uniform([-200, -150, 400], [200, 150, 600], size=(6, 3))
    S = (A_true @ P.T).T + b_true
    A, b, rms = fit(P, S)
    ok &= check("A 복원", np.allclose(A, A_true, atol=1e-6),
                f"최대오차 {np.abs(A-A_true).max():.2e}")
    ok &= check("b 복원", np.allclose(b, b_true, atol=1e-6))
    ok &= check("잔차 ~0", rms < 1e-6, f"rms={rms:.2e}")

    print("■ 4점 — 미지수와 같은 수라 잔차가 0 으로 나온다 (정확도 확인 불가)")
    P4 = rng.uniform([-200, -150, 400], [200, 150, 600], size=(4, 3))
    S4 = (A_true @ P4.T).T + b_true + rng.normal(0, 0.5, (4, 2))   # 잡음을 넣어도
    _, _, rms4 = fit(P4, S4)
    ok &= check("잔차가 0 에 가깝다 (착시)", rms4 < 1e-6,
                f"rms={rms4:.2e} — 잡음을 넣었는데도 0 이다")

    print("■ 6점 + 잡음 — 잔차가 잡음 크기를 반영한다")
    noise = 0.5
    S6 = (A_true @ P.T).T + b_true + rng.normal(0, noise, (6, 2))
    _, _, rms6 = fit(P, S6)
    ok &= check("잔차가 잡음 수준", 0.1 < rms6 < 3 * noise * 2,
                f"rms={rms6:.3f}° (잡음 σ={noise})")

    print("■ 퇴화 — 점이 한 줄로 늘어선 경우")
    t = np.linspace(-1, 1, 6).reshape(-1, 1)
    Pl = np.array([0, 0, 500]) + t * np.array([100, 60, 20])
    Sl = (A_true @ Pl.T).T + b_true + rng.normal(0, 0.2, (6, 2))
    Al, bl, rmsl = fit(Pl, Sl)
    off = float(np.abs(Al - A_true).max())
    ok &= check("한 줄이면 A 가 크게 어긋난다", off > 0.01,
                f"최대오차 {off:.4f} — 잔차 {rmsl:.3f}° 는 작아 보여도 변환은 틀렸다")
    print("    → 점을 시야 안에서 **흩뜨려** 고르라는 경고가 근거 있다")

    print(f"\n■ 결과: {'통과' if ok else '실패'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
