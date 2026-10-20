#!/usr/bin/env python3
"""층 분리: k-means 와 골(gap) 기반을 **같은 장면에서** 비교해 그린다.

왜 이 그림이 필요한가 — 원본(한국공대 학생 구현)은 평면거리를 k-means 로
나눈다. 1·2차 장면에서는 맞았는데 **3차 장면에서 통째로 틀렸다.** 원인을
말로만 적으면 다음 사람이 또 k-means 로 돌아간다.

    python3 tools/test/layer_kmeans_vs_bands.py <depth.npy> <K.npy> [out.png]

`depth.npy` 는 mm 단위 2차원 배열, `K.npy` 는 (fx, fy, cx, cy) 4원소.
`tools/test/grab_depth.py` 류로 떠 둔 프레임을 쓴다.
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

BAND_C = ['#D32F2F', '#F9A825', '#1976D2', '#43A047', '#8E24AA', '#00838F']


def kmeans_1d(dist, k=4, seed=42):
    """원본이 쓰던 방식 — 비교를 위해 여기에만 남겨 둔다."""
    import cv2
    d = np.asarray(dist, np.float32).reshape(-1, 1)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
    _, labels, centers = cv2.kmeans(d, k, None, crit, 10, cv2.KMEANS_PP_CENTERS)
    c = centers.reshape(-1)
    order = np.argsort(c)
    remap = np.empty(k, int)
    remap[order] = np.arange(k)
    return remap[labels.reshape(-1)], c[order]


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


def draw(ax, dist, centers, labels, title, note):
    """히스토그램 위에 층 경계를 칠한다. 층이 **골에 맞는지** 가 한눈에 보이게."""
    ax.hist(dist, bins=np.arange(dist.min(), dist.max() + 10, 10),
            color='#90A4AE', edgecolor='none')
    for r, c in enumerate(centers):
        m = labels == r
        if not m.any():
            continue
        lo, hi = np.percentile(dist[m], [0.5, 99.5])
        col = BAND_C[r % len(BAND_C)]
        ax.axvspan(lo, hi, color=col, alpha=0.17, zorder=0)
        ax.axvline(c, color=col, lw=2.0, zorder=3)
        ax.text(c, ax.get_ylim()[1] * 0.97, f'{-c:.0f}mm',
                rotation=90, va='top', ha='right', fontsize=9, color=col)
    ax.set_title(title, fontsize=13, pad=8)
    ax.set_xlabel('바닥 평면까지의 거리 (mm · 음수가 위쪽)', fontsize=10)
    ax.set_ylabel('화소 수', fontsize=10)
    ax.text(0.015, 0.975, note, transform=ax.transAxes, va='top', fontsize=10,
            bbox=dict(boxstyle='round,pad=0.45', fc='#FFF8E1', ec='#FFB300'))


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    depth = np.load(sys.argv[1]).astype(np.float32)
    K = tuple(np.load(sys.argv[2]))
    out = sys.argv[3] if len(sys.argv) > 3 else 'layer_kmeans_vs_bands.png'

    dist = plane_dist(depth, K)
    kl, kc = kmeans_1d(dist, 4)
    bl, bc = _bands(dist)

    fig, axes = plt.subplots(2, 1, figsize=(11.5, 9.0))
    draw(axes[0], dist, kc, kl, 'k-means — 층 수를 4 로 강제 (원본 방식)',
         '경계가 봉우리가 아니라 분산으로 정해진다.\n'
         '점이 많은 층을 쪼개고 적은 층을 뭉개는 경향이 있어,\n'
         '층별 점 개수 비율이 바뀌면 결과가 달라진다.')
    draw(axes[1], dist, bc, bl, '골(gap) 기반 — 빈 구간으로 가른다 (바꾼 것)',
         '경계가 빈 틈에 놓인다. 층 수를 미리 정하지 않아\n'
         '개수 편차에 안 끌리는 대신, 이 프레임처럼\n'
         '한 층이 더 쪼개지기도 한다 (위 띠 2개).')
    lo, hi = np.percentile(dist, [0.2, 99.9])
    for ax in axes:
        ax.set_xlim(lo, hi + 20)
    fig.suptitle('층 분리 — 같은 히스토그램을 두 방법이 어떻게 가르나',
                 fontsize=15, y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    fig.savefig(out, dpi=130)
    print(f'저장: {out}')
    print(f'  k-means 층 높이: ' + ' · '.join(f'{-c:.0f}' for c in kc))
    print(f'  골 기반 층 높이: ' + ' · '.join(f'{-c:.0f}' for c in bc))


main()
