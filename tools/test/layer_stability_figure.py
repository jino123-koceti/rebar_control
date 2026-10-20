#!/usr/bin/env python3
"""층 분리의 **흔들림**을 그린다 — 군집 방법보다 기준 평면이 문제다.

    python3 tools/test/layer_stability_figure.py <depth0.npy> … -K <K.npy> -o <out.png>

같은 자세에서 뜬 프레임 여러 장을 주면, 각 장에 대해
  ① 평면거리 히스토그램을 겹쳐 그려 **평면이 얼마나 움직이는지**
  ② k-means 와 골 기반이 낸 층 높이를 나란히
보여 준다.

## 이 그림이 말하는 것 (2026-10-20 실측)

원본(한국공대 구현)은 평면거리를 **k-means** 로 나눈다. 그것을 **골 기반**으로
바꾸면 "점이 많은 층을 쪼개는" 문제는 사라진다. 그런데 같은 자세에서 6회를
떠 보니 **두 방법 모두 흔들렸다** — 층 높이가 회차마다 한꺼번에 25~70mm 씩
밀린다. 한꺼번에 밀린다는 것이 핵심 단서다. 층을 **나누는** 문제가 아니라
층을 재는 **자(기준 평면)** 가 흔들린다는 뜻이다.

따라서 절대 높이로 층을 고르는 방식(`pick_rank(120~300mm)`)은 그대로 쓸 수
없다. 평면을 안정시키거나, 평면과 무관한 기준(예: 가장 큰 띠인 **바닥을
기준점으로** 삼아 상대 높이로 고르기)이 필요하다.
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                      # noqa: E402

for _f in ('Noto Sans CJK JP', 'Noto Sans CJK KR', 'NanumGothic', 'DejaVu Sans'):
    try:
        matplotlib.font_manager.findfont(_f, fallback_to_default=False)
        plt.rcParams['font.family'] = _f
        break
    except Exception:
        continue
plt.rcParams['axes.unicode_minus'] = False

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src', 'rebar_vision'))
from rebar_vision.layer_rank import (_bands, _floor_plane,   # noqa: E402
                                     DEPTH_MIN, DEPTH_MAX, MAX_POINTS)


def kmeans_1d(dist, k=4):
    """원본이 쓰던 방식 — 비교용으로 여기에만 남긴다."""
    import cv2
    d = np.asarray(dist, np.float32).reshape(-1, 1)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
    _, _, cen = cv2.kmeans(d, k, None, crit, 10, cv2.KMEANS_PP_CENTERS)
    return np.sort(cen.reshape(-1))


def plane_dist(depth, K):
    fx, fy, cx, cy = K
    good = np.isfinite(depth) & (depth >= DEPTH_MIN) & (depth <= DEPTH_MAX)
    vv, uu = np.nonzero(good)
    zz = depth[vv, uu]
    pts = np.stack([(uu - cx) * zz / fx, (vv - cy) * zz / fy, zz], axis=1)
    sel = pts if len(pts) <= MAX_POINTS else pts[
        np.random.default_rng(42).choice(len(pts), MAX_POINTS, replace=False)]
    normal, d = _floor_plane(sel)
    return pts @ normal + d


def main():
    args = sys.argv[1:]
    if '-K' not in args:
        sys.exit(__doc__)
    ki = args.index('-K')
    files = args[:ki]
    K = tuple(np.load(args[ki + 1]))
    out = args[args.index('-o') + 1] if '-o' in args else 'layer_stability.png'

    dists, kms, bds = [], [], []
    for f in files:
        d = plane_dist(np.load(f).astype(np.float32), K)
        dists.append(d)
        kms.append([-c for c in kmeans_1d(d, 4)])
        bds.append(sorted([-c for c in (_bands(d)[1] or [])], reverse=True))

    fig, axes = plt.subplots(2, 1, figsize=(11.5, 9.4),
                             gridspec_kw=dict(height_ratios=[1.15, 1.0]))

    # ① 평면거리 분포를 회차별로 겹쳐 — 평면이 움직이면 통째로 밀린다
    ax = axes[0]
    cmap = plt.get_cmap('viridis')
    for i, d in enumerate(dists):
        ax.hist(d, bins=np.arange(-420, 160, 10), histtype='step', lw=1.7,
                color=cmap(i / max(1, len(dists) - 1)), label=f'{i + 1}회')
    ax.set_title('① 같은 자세에서 6회 — 평면거리 분포가 통째로 밀린다',
                 fontsize=13, pad=8)
    ax.set_xlabel('바닥 평면까지의 거리 (mm · 음수가 위쪽)', fontsize=10)
    ax.set_ylabel('화소 수', fontsize=10)
    ax.legend(fontsize=9, ncol=6, loc='upper left')
    ax.text(0.985, 0.93,
            '대부분 회차는 겹치는데 일부(2회)만 통째로 밀렸다.\n'
            '한 층이 아니라 모든 봉우리가 같이 밀린다 = 층을 나누는\n'
            '문제가 아니라 층을 재는 자(기준 평면)가 다른 면에 붙은 것',
            transform=ax.transAxes, va='top', ha='right', fontsize=9.5,
            bbox=dict(boxstyle='round,pad=0.45', fc='#FFEBEE', ec='#E53935'))

    # ② 두 방법이 낸 층 높이
    ax = axes[1]
    for i, (km, bd) in enumerate(zip(kms, bds)):
        ax.scatter([i + 1 - 0.13] * len(km), km, s=64, marker='o',
                   color='#E53935', zorder=3,
                   label='k-means (원본)' if i == 0 else None)
        ax.scatter([i + 1 + 0.13] * len(bd), bd, s=64, marker='s',
                   color='#1E88E5', zorder=3,
                   label='골 기반 (바꾼 것)' if i == 0 else None)
    ax.axhspan(120, 300, color='#FFC107', alpha=0.16, zorder=0)
    ax.text(len(dists) + 0.42, 210, 'pick_rank 가\n상부근을 찾는\n창 120~300mm',
            fontsize=9, va='center', ha='center', color='#F57F17')
    ax.set_title('② 층 높이 — 두 방법 모두 회차마다 달라진다',
                 fontsize=13, pad=8)
    ax.set_xlabel('촬영 회차 (각 9장 중앙값)', fontsize=10)
    ax.set_ylabel('바닥 위 높이 (mm)', fontsize=10)
    ax.set_xticks(range(1, len(dists) + 1))
    ax.set_xlim(0.5, len(dists) + 0.95)
    ax.grid(axis='y', alpha=0.3)
    ax.legend(fontsize=10, loc='lower right')

    fig.suptitle('층 분리 — k-means 를 버린 이유와, 아직 남은 문제',
                 fontsize=15, y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.962))
    fig.savefig(out, dpi=130)
    print(f'저장: {out}')
    for i, (km, bd) in enumerate(zip(kms, bds)):
        print(f'  {i + 1}회  k-means ' + ' · '.join(f'{v:5.0f}' for v in km)
              + '   |  골 ' + ' · '.join(f'{v:5.0f}' for v in bd))


main()
