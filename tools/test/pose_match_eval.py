#!/usr/bin/env python3
"""수집한 격자를 **템플릿**으로 삼아 (X, Y, yaw) 를 맞히고 성능을 잰다.

왜 이 방식인가 (2026-10-07):
단일 특징(건 덩어리의 주축각)으로 yaw 부호를 가리려 했더니, **한 점(x≈0, y≈174)
에서는 4/4 완벽**인데 X·Y 를 전 범위로 넓히자 45장 중 **3장을 틀렸다.** X 가
커지면 건이 카메라에 가까워져 주축각이 뒤집힌다 — 1번인데 −86°, 4번인데 +77.6°.
거부가 아니라 **틀린 답을 자신 있게 내는** 가장 나쁜 실패였다.

원인은 특징이 약해서가 아니라 **숨은 전제**였다. 주축각은 X·Y 가 고정일 때만
yaw 의 함수다. 그래서 단일 특징을 버리고 **이산 후보 매칭**으로 간다:

    단회전 → X 5후보 × Y 5후보 × yaw 2후보 = 50조합
       ↓   각 조합의 템플릿과 지금 영상을 비교
    가장 잘 맞는 하나. 2등과 차이가 작으면 **거부**.

이러면 "X 를 모른다" 가 약점이 아니라 **후보 중 하나**가 된다. 그리고 비전은
템플릿 밖의 답을 만들어 낼 수 없다 — 엔코더가 준 후보 안에서만 고른다.

**성능은 leave-one-out 으로 잰다.** 각 장을 템플릿에서 빼고 나머지로 맞힌다.
자기 자신이 템플릿에 있으면 100% 가 나와 아무것도 알 수 없다.

사용
  python3 pose_match_eval.py --dirs ~/vmap/grid1:1 ~/vmap/grid4:4
"""

import argparse
import csv
import os
import sys


def load(dirs):
    """[(자세, x, y, 특징벡터)] 로 읽는다."""
    import cv2
    out = []
    for spec in dirs:
        d, _, pose = spec.partition(':')
        pose = int(pose)
        idx = os.path.join(d, 'index.csv')
        for r in csv.DictReader(open(idx)):
            img = cv2.imread(os.path.join(d, r['file']))
            if img is None:
                continue
            out.append((pose, float(r['x_mm']), float(r['y_mm']),
                        descriptor(img), r['file']))
    return out


def descriptor(bgr):
    """영상을 작은 벡터로 줄인다.

    ⚠ 배근(철근)은 현장마다 바뀌므로 **기계가 만드는 밝기 구조**만 남기고 싶다.
      그래서 원본을 크게 줄여(저해상도) 세부 질감을 버리고, 큰 덩어리의 배치만
      남긴다. 건은 어둡고 크므로 이 수준에서 충분히 드러난다.
    """
    import cv2
    import numpy as np
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(g, (32, 20), interpolation=cv2.INTER_AREA)
    v = small.astype(np.float32).ravel()
    v -= v.mean()                      # 전체 밝기 변화(조명)에 둔감하게
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else v


def match(query, templates, exclude_idx=None):
    """가장 가까운 템플릿과 그 거리, 그리고 **다른 자세** 중 최선의 거리."""
    import numpy as np
    best = (1e9, None)
    best_other = 1e9
    for i, (pose, x, y, vec, _f) in enumerate(templates):
        if i == exclude_idx:
            continue
        dist = float(1.0 - np.dot(query[3], vec))     # 코사인 거리
        if dist < best[0]:
            best = (dist, (pose, x, y))
        # 정답 자세와 **다른** 자세 중 최선 — 여유(margin)를 재려고
    if best[1] is not None:
        for i, (pose, x, y, vec, _f) in enumerate(templates):
            if i == exclude_idx or pose == best[1][0]:
                continue
            dist = float(1.0 - np.dot(query[3], vec))
            best_other = min(best_other, dist)
    return best[0], best[1], best_other


# 단회전이 접히는 간격 — 이 간격의 배수만 후보가 된다
PITCH = {'x': 104.6, 'y': 80.2}
STROKE = {'x': (0.0, 453.7), 'y': (0.0, 349.1)}


def candidates(val, axis):
    """그 축의 단회전 후보들. 참값에서 ±PITCH 배수로 스트로크 안에 드는 것."""
    lo, hi = STROKE[axis]
    pit = PITCH[axis]
    out, k = [], -8
    while k <= 8:
        v = val + k * pit
        if lo - 1 <= v <= hi + 1:
            out.append(v)
        k += 1
    return out


def eval_constrained(data, margin):
    """⚠ **후보를 실제대로 제한해서** 평가한다.

    전에는 모든 템플릿과 비교했는데, 그러면 실제로는 **동시에 후보가 될 수 없는**
    조합끼리 혼동한 것까지 오답으로 센다 (예: y=210 과 y=110 은 100mm 차이인데
    Y 후보 간격은 80.2mm 라 둘 다 후보가 되는 일이 없다). 엔코더가 후보를 격자로
    묶어 주는 것이 이 방식의 핵심이므로, 평가도 그 조건에서 해야 한다.
    """
    import numpy as np
    ok = bad = ref = 0
    wrong = []
    for i, q in enumerate(data):
        tp, tx, ty, qv, _f = q
        xs = candidates(tx, 'x')
        ys = candidates(ty, 'y')
        # 후보 격자에 가장 가까운 템플릿들만 비교 대상으로 삼는다
        pool = []
        for j, (pose, x, y, vec, _g) in enumerate(data):
            if j == i:
                continue
            near_x = min(abs(x - c) for c in xs)
            near_y = min(abs(y - c) for c in ys)
            if near_x <= 55 and near_y <= 45:        # 격자 간격의 절반 안
                pool.append((pose, x, y, vec))
        if len(pool) < 2:
            ref += 1
            continue
        d = [(float(1.0 - np.dot(qv, v)), p, x, y) for p, x, y, v in pool]
        d.sort()
        best = d[0]
        other = next((e for e in d if e[1] != best[1]), None)
        if other is None:
            ref += 1
            continue
        if other[0] - best[0] < margin:
            ref += 1
            continue
        if best[1] == tp:
            ok += 1
        else:
            bad += 1
            wrong.append((tp, tx, ty, best, other[0] - best[0]))
    return ok, bad, ref, wrong


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dirs', nargs='+', required=True,
                   help='경로:자세 형식. 예 ~/vmap/grid1:1 ~/vmap/grid4:4')
    p.add_argument('--margin', type=float, default=0.0,
                   help='1등과 다른자세 최선의 거리차가 이보다 작으면 거부')
    a = p.parse_args()

    data = load(a.dirs)
    if not data:
        print("데이터가 없습니다.")
        return 1
    print(f"템플릿 {len(data)}장\n")

    ok, bad, ref, wrong = eval_constrained(data, a.margin)
    print(f"=== 후보 제한 평가 (실제 조건)")
    print(f"자세 판정 — 정답 {ok} / 틀림 {bad} / 거부 {ref}  (전체 {len(data)})")
    for tp, tx, ty, best, gap in wrong:
        print(f"  자세{tp} x={tx:.0f} y={ty:.0f} → 자세{best[1]} "
              f"x={best[2]:.0f} y={best[3]:.0f}  여유 {gap:+.4f}")
    print()

    ok = bad = ref = 0
    xerr = []
    yerr = []
    wrong = []
    for i, q in enumerate(data):
        dist, got, other = match(q, data, exclude_idx=i)
        if got is None:
            ref += 1
            continue
        gap = other - dist
        if gap < a.margin:
            ref += 1
            continue
        if got[0] == q[0]:
            ok += 1
            xerr.append(abs(got[1] - q[1]))
            yerr.append(abs(got[2] - q[2]))
        else:
            bad += 1
            wrong.append((q[0], q[1], q[2], got, dist, gap))

    n = len(data)
    print(f"=== 참고: 전 템플릿 비교 (실제보다 어려운 조건)")
    print(f"자세 판정 — 정답 {ok} / 틀림 {bad} / 거부 {ref}  (전체 {n})")
    if wrong:
        print("\n틀린 것:")
        for truth, x, y, got, dist, gap in wrong:
            print(f"  자세{truth} x={x:.0f} y={y:.0f} → 자세{got[0]} "
                  f"x={got[1]:.0f} y={got[2]:.0f}  거리 {dist:.4f} 여유 {gap:+.4f}")
    if xerr:
        import numpy as np
        print(f"\n자세가 맞은 것들의 위치 오차 — "
              f"X 평균 {np.mean(xerr):.0f}mm 최대 {max(xerr):.0f}mm / "
              f"Y 평균 {np.mean(yerr):.0f}mm 최대 {max(yerr):.0f}mm")
        print(f"  (참고: 후보 간격은 X 104.6mm, Y 80.2mm — "
              f"그 절반보다 작아야 후보를 고를 수 있다)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
