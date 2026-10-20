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
from rebar_vision.layer_rank import (_bands, _floor_plane, ROI_DECK,  # noqa: E402
                                     DEPTH_MIN, DEPTH_MAX, MAX_POINTS)


def kmeans_1d(dist, k=4):
    """원본이 쓰던 방식 — 비교용으로 여기에만 남긴다."""
    import cv2
    d = np.asarray(dist, np.float32).reshape(-1, 1)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
    _, _, cen = cv2.kmeans(d, k, None, crit, 10, cv2.KMEANS_PP_CENTERS)
    return np.sort(cen.reshape(-1))


def plane_dist(depth, K, roi=None):
    fx, fy, cx, cy = K
    good = np.isfinite(depth) & (depth >= DEPTH_MIN) & (depth <= DEPTH_MAX)
    if roi is not None:
        x, y, w, h = roi
        m = np.zeros_like(good)
        m[y:y + h, x:x + w] = True
        good &= m
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

    def layers(roi):
        ds, hs = [], []
        for f in files:
            d = plane_dist(np.load(f).astype(np.float32), K, roi)
            ds.append(d)
            hs.append(sorted([-c for c in (_bands(d)[1] or [])], reverse=True))
        return ds, hs

    d_full, h_full = layers(None)
    d_roi, h_roi = layers(ROI_DECK)

    fig, axes = plt.subplots(3, 1, figsize=(11.5, 12.4),
                             gridspec_kw=dict(height_ratios=[1, 1, 1.05]))
    cmap = plt.get_cmap('viridis')

    def hists(ax, ds, title, note, fc, ec):
        for i, d in enumerate(ds):
            ax.hist(d, bins=np.arange(-420, 160, 10), histtype='step', lw=1.7,
                    color=cmap(i / max(1, len(ds) - 1)), label=f'{i + 1}회')
        ax.set_title(title, fontsize=13, pad=8)
        ax.set_xlabel('바닥 평면까지의 거리 (mm · 음수가 위쪽)', fontsize=10)
        ax.set_ylabel('화소 수', fontsize=10)
        ax.set_xlim(-420, 120)
        ax.legend(fontsize=9, ncol=6, loc='upper left')
        ax.text(0.985, 0.93, note, transform=ax.transAxes, va='top', ha='right',
                fontsize=9.5,
                bbox=dict(boxstyle='round,pad=0.45', fc=fc, ec=ec))

    hists(axes[0], d_full, '① 전체 영상 — 회차마다 봉우리가 통째로 밀린다',
          '데크 밖 배경(먼 벽)과 결속건이 평면 탐색에 섞여\n'
          '평면을 끌어당긴다. 모든 봉우리가 같이 밀리는 것이\n'
          '"층이 아니라 자가 흔들린다" 는 신호다.',
          '#FFEBEE', '#E53935')
    hists(axes[1], d_roi, '② 데크만 남기고(ROI) — 여섯 회차가 거의 포개진다',
          'ROI = (280, 160, 720, 630). 좌우 프레임 레일까지 뺀다 —\n'
          '레일이 상부근과 비슷한 높이(≈198mm)라 안 빼면 섞인다.',
          '#E8F5E9', '#43A047')

    ax = axes[2]
    for i, (a, b) in enumerate(zip(h_full, h_roi)):
        ax.scatter([i + 1 - 0.14] * len(a), a, s=66, marker='o',
                   color='#E53935', zorder=3,
                   label='전체 영상' if i == 0 else None)
        ax.scatter([i + 1 + 0.14] * len(b), b, s=66, marker='s',
                   color='#1E88E5', zorder=3,
                   label='ROI 적용' if i == 0 else None)
    ax.axhspan(120, 300, color='#FFC107', alpha=0.16, zorder=0)
    ax.text(len(files) + 0.42, 210, 'pick_rank 가\n상부근을 찾는\n창 120~300mm',
            fontsize=9, va='center', ha='center', color='#F57F17')
    fu = [max([v for v in a if 120 <= v <= 300], default=float('nan')) for a in h_full]
    ru = [max([v for v in b if 120 <= v <= 300], default=float('nan')) for b in h_roi]
    okf = [v for v in fu if v == v]
    okr = [v for v in ru if v == v]
    ax.set_title(f'③ 층 높이 — 상부근으로 고른 값의 폭: '
                 f'전체 {max(okf)-min(okf):.0f}mm → ROI {max(okr)-min(okr):.0f}mm',
                 fontsize=13, pad=8)
    ax.set_xlabel('촬영 회차 (각 9장 중앙값)', fontsize=10)
    ax.set_ylabel('바닥 위 높이 (mm)', fontsize=10)
    ax.set_xticks(range(1, len(files) + 1))
    ax.set_xlim(0.5, len(files) + 0.95)
    ax.grid(axis='y', alpha=0.3)
    ax.legend(fontsize=10, loc='lower right')

    fig.suptitle('층 분리 — 흔들림의 원인은 군집 방법이 아니라 ROI 였다',
                 fontsize=15, y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.972))
    fig.savefig(out, dpi=130)
    print(f'저장: {out}')
    for i, (a, b) in enumerate(zip(h_full, h_roi)):
        print(f'  {i + 1}회  전체 ' + ' · '.join(f'{v:5.0f}' for v in a)
              + '   |  ROI ' + ' · '.join(f'{v:5.0f}' for v in b))


main()
