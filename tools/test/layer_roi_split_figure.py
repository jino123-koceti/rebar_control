#!/usr/bin/env python3
"""ROI 를 **평면 맞춤용**과 **층 매김용**으로 나눠야 하는 이유를 그린다.

    python3 tools/test/layer_roi_split_figure.py <depth.npy> <K.npy> <color.png> \
            <u> <v> [out.png]

`u v` 는 ROI 경계 가까이에 있는 검출 하나의 화소 좌표.

## 무엇을 보여주나 (2026-10-20 실측)

층 분리는 ROI 가 있어야 안정된다 — 데크 밖 배경이 평면을 끌어당기기 때문이다
(`layer_stability.png`). 그런데 **같은 ROI 로 층까지 매기면** 경계 가까운
검출의 둘레 원이 잘린다. 비율의 분모는 원 전체 넓이라, 잘린 만큼 비율이
그대로 깎인다 — 진짜 상부근 교차점이 **허위처럼 보인다.**

    같은 검출 (517, 157):  ROI 로 층까지 매김 0.156  →  나눠서 매김 0.416

그래서 **좁게 맞추고 넓게 매긴다.** 평면만 정확하면 랭크는 어디든 매길 수
있으므로, 둘을 묶어 둘 이유가 없다.
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.patches import Circle, Rectangle     # noqa: E402

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
from rebar_vision.layer_rank import (rank_map, layer_ratio, pick_rank,  # noqa: E402
                                     ROI_DECK, INVALID)

RADIUS = 50
BAND_RGB = [(0.90, 0.20, 0.18), (0.98, 0.68, 0.14),
            (0.20, 0.55, 0.85), (0.30, 0.70, 0.35),
            (0.60, 0.35, 0.72), (0.40, 0.40, 0.40)]


def paint(rank, ncen):
    """랭크 지도를 색으로. 깊이 없음은 어둡게."""
    h, w = rank.shape
    img = np.full((h, w, 3), 0.11)
    for r in range(ncen):
        img[rank == r] = BAND_RGB[r % len(BAND_RGB)]
    return img


def panel(ax, depth, K, color, u, v, roi_label, title, note):
    """`roi_label` 이 None 이면 전체에 매긴다 (나눈 방식)."""
    rank, info = rank_map(depth, K, roi=roi_label, fit_roi=ROI_DECK,
                          depth_max=1000.0)
    up = pick_rank(info, 120, 300, 'high')
    ratio = layer_ratio(rank, u, v, RADIUS, up) if up is not None else 0.0

    vis = 0.45 * (color[:, :, ::-1] / 255.0) + 0.55 * paint(rank, len(info['centers']))
    ax.imshow(np.clip(vis, 0, 1))
    rx, ry, rw, rh = ROI_DECK
    ax.add_patch(Rectangle((rx, ry), rw, rh, fill=False, ec='#E91E63', lw=2.2))
    ax.add_patch(Circle((u, v), RADIUS, fill=False, ec='white', lw=2.6))
    ax.plot([u], [v], 'w+', ms=12, mew=2.4)
    ax.set_title(title, fontsize=12.5, pad=7)
    ax.set_xticks([]); ax.set_yticks([])
    ax.text(0.5, -0.045, note, transform=ax.transAxes, ha='center', va='top',
            fontsize=10.5,
            bbox=dict(boxstyle='round,pad=0.45', fc='#FFF8E1', ec='#FFB300'))
    ax.text(u + RADIUS + 12, v, f'상부근 비율 {ratio:.3f}', color='white',
            fontsize=12, va='center',
            bbox=dict(boxstyle='round,pad=0.3', fc='#263238', ec='none'))
    return ratio


def main():
    if len(sys.argv) < 6:
        sys.exit(__doc__)
    depth = np.load(sys.argv[1]).astype(np.float32)
    K = tuple(np.load(sys.argv[2]))
    import cv2
    color = cv2.imread(sys.argv[3])
    u, v = int(sys.argv[4]), int(sys.argv[5])
    out = sys.argv[6] if len(sys.argv) > 6 else 'layer_roi_split.png'

    fig, axes = plt.subplots(1, 2, figsize=(13.6, 5.6))
    a = panel(axes[0], depth, K, color, u, v, ROI_DECK,
              '① 같은 ROI 로 평면도 맞추고 층도 매김',
              'ROI 밖은 "모름" 이라 둘레 원의 위 절반이 빈다.\n'
              '분모는 원 전체 넓이 → 비율이 그대로 깎인다.')
    b = panel(axes[1], depth, K, color, u, v, None,
              '② 평면만 ROI 로 맞추고, 층은 전체에 매김',
              '평면이 정확하면 랭크는 어디든 매길 수 있다.\n'
              '원이 온전해져 진짜 값이 나온다.')
    fig.suptitle(f'ROI 는 평면 맞춤에만 — 경계 검출 ({u}, {v}) 의 비율 '
                 f'{a:.3f} → {b:.3f}', fontsize=14.5, y=0.985)
    fig.tight_layout(rect=(0, 0.07, 1, 0.945))
    fig.savefig(out, dpi=130)
    print(f'저장: {out}   비율 {a:.3f} → {b:.3f}')


main()
