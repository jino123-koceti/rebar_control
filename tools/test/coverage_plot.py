#!/usr/bin/env python3
"""커버리지 시뮬 결과를 그림으로. `coverage_sim.py --plot` 이 부른다.

발표·설명 자료에 바로 쓸 수 있는 수준으로 그린다 — 그래서 한글 글꼴을 쓰고,
**주행과 횡이동을 다른 굵기로** 그린다 (비용 비 7:1 이 이 알고리즘의 핵심이라
그림에서 바로 보여야 한다).
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.patches import Rectangle             # noqa: E402

for _f in ('Noto Sans CJK JP', 'Noto Sans CJK KR', 'NanumGothic', 'DejaVu Sans'):
    try:
        matplotlib.font_manager.findfont(_f, fallback_to_default=False)
        plt.rcParams['font.family'] = _f
        break
    except Exception:
        continue
plt.rcParams['axes.unicode_minus'] = False

C_TIED = '#2a7f3f'
C_FAIL = '#c62828'
C_LEFT = '#9e9e9e'
C_DRIVE = '#1565c0'
C_LAT = '#ef6c00'
C_REACH = '#1565c0'


def _draw(ax, world, trace, reach_bbox, title, note=''):
    us, vs = world.us, world.vs
    d = world.deck

    ax.add_patch(Rectangle((d[0], d[2]), d[1] - d[0], d[3] - d[2],
                           fc='#f5f5f5', ec='#bdbdbd', lw=1.2, zorder=0))
    for u in us:
        ax.plot([u, u], [d[2], d[3]], color='#d7ccc8', lw=1.0, zorder=1)
    for v in vs:
        ax.plot([d[0], d[1]], [v, v], color='#d7ccc8', lw=1.0, zorder=1)

    # 기둥 — 크기 있는 장애물. 주행도 막는다
    for cu, cv, su, sv in getattr(world, 'columns', ()):
        ax.add_patch(Rectangle((cu - su / 2, cv - sv / 2), su, sv,
                               fc='#8d6e63', ec='#4e342e', lw=1.4, zorder=6))

    # ⚠ **정차지는 결속영역의 중심에 찍는다.** 내부 포즈(ru, rv)는 영역의
    #   모서리 기준이라 그대로 찍으면 묶은 교차점과 어긋나 보인다(사용자 지적).
    #   계획은 포즈로 하고, **그림만** 중심으로 옮긴다.
    b = reach_bbox
    rmid = (0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]))
    # 경로는 결속영역 **중심**을 잇는다. 정차지 표시를 따로 찍지 않고
    # 화살표만 남기므로(사용자 요청), 선이 묶은 교차점 위를 지나가는 것이
    # 오히려 "여기서 여기로 갔다" 로 읽힌다. 순서는 화살표에 적는다.
    def cen(pose):
        return (pose[0] + rmid[0], pose[1] + rmid[1])

    if trace['stops']:
        ru, rv = trace['stops'][0]['pose']
        ax.add_patch(Rectangle((ru + b[0], rv + b[2]), b[1] - b[0], b[3] - b[2],
                               fc=C_REACH, alpha=0.10, ec=C_REACH, ls='--',
                               lw=1.2, zorder=2))
        # 사각 **안쪽 위**에 붙인다 — 가운데는 이제 정차지 번호가 쓴다
        ax.text(ru + b[0] + 14, rv + b[3] - 14, '결속 가능 영역\n(한 정차지)',
                color=C_REACH, fontsize=7.5, va='top', ha='left',
                linespacing=1.3, zorder=6)

    # 이동 경로 — **ㄱ자로 꺾어 그린다.** 로봇은 주행과 횡이동을 동시에 하지
    # 않는다 (주행은 궤도, 횡이동은 2축 리프팅). 대각선으로 그리면 거짓말이다.
    # 순서 번호는 **화살표 위**에 올린다.
    P = [cen(s_['pose']) for s_ in trace['stops']]
    for i, (a, c) in enumerate(zip(P, P[1:])):
        corner = (c[0], a[1])
        legs = []
        if abs(c[0] - a[0]) > 1.0:
            ax.annotate('', xy=corner, xytext=a, zorder=4,
                        arrowprops=dict(arrowstyle='-|>', shrinkA=4,
                                        shrinkB=0 if abs(c[1] - a[1]) > 1 else 4,
                                        color=C_DRIVE, lw=2.0, alpha=0.95))
            legs.append((a, corner, 'u'))
        if abs(c[1] - a[1]) > 1.0:
            ax.annotate('', xy=c, xytext=corner, zorder=4,
                        arrowprops=dict(arrowstyle='-|>', shrinkA=0, shrinkB=4,
                                        color=C_LAT, lw=3.4, alpha=0.95))
            legs.append((corner, c, 'v'))
        if not legs:
            continue
        # 긴 쪽 구간의 가운데 위에 번호를 둔다
        p0, p1, kind = max(legs, key=lambda L: abs(L[1][0] - L[0][0])
                           + abs(L[1][1] - L[0][1]))
        # ⚠ 가운데에 일률적으로 두면 **ㄱ자의 두 구간 라벨이 겹친다**.
        #   주행은 40% 지점 위, 횡이동은 35% 지점 옆 — 떨어뜨려 놓는다.
        f = 0.40 if kind == 'u' else 0.35
        mx, my = p0[0] + (p1[0] - p0[0]) * f, p0[1] + (p1[1] - p0[1]) * f
        off = 0.14 * (b[3] - b[2])
        tx, ty = (mx, my + off) if kind == 'u' else (mx + off * 2.1, my)
        ax.text(tx, ty, str(i + 1), ha='center', va='center', fontsize=9.5,
                color=C_DRIVE if kind == 'u' else C_LAT, zorder=8,
                bbox=dict(fc='#FFFFFF', ec='#CFD8DC', lw=0.8, pad=1.6,
                          boxstyle='round,pad=0.25'))

    # 교차점 상태
    for (u, v), st in trace['cells'].items():
        col = {'tied': C_TIED, 'failed': C_FAIL, 'blocked': C_FAIL}.get(st, C_LEFT)
        mk = 'o' if st == 'tied' else ('X' if st in ('failed', 'blocked') else 's')
        ax.plot(u, v, mk, ms=11 if mk == 'X' else 9, color=col,
                mec='white', mew=1.2, zorder=5)

    # 출발 표시 — 순서가 어디서 시작하는지는 있어야 읽힌다
    if P:
        ax.plot(P[0][0], P[0][1], 'o', ms=13, color='#FFFFFF',
                mec='#37474F', mew=1.6, zorder=7)
        ax.text(P[0][0], P[0][1], 'S', ha='center', va='center', fontsize=8.5,
                color='#37474F', zorder=8)

    # 축 범위 — 내용에 딱 맞춘다. 요약 상자 자리를 만들려고 축을 늘렸더니
    # 그림이 위로 밀려 보였다 (사용자 지적). 요약은 제목 아래로 뺀다.
    xs = [p[0] for p in P] + [d[0], d[1]]
    ys = [p[1] for p in P] + [d[2], d[3]]
    for cu, cv, su, sv in getattr(world, 'columns', ()):   # 기둥이 잘리면 벽으로 안 읽힌다
        xs += [cu - su / 2, cu + su / 2]
        ys += [cv - sv / 2, cv + sv / 2]
    pad = 170.0
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)

    # ⚠ 요약을 제목 위 여백에 띄우지 않는다. `aspect='equal'` 이면 축 상자가
    #   줄면서 그 글씨가 그림 밖으로 밀려 **제목이 잘린다**(실제로 그랬다).
    #   요약은 `save_*` 가 범례 위 고정 자리에 그린다.
    ax.set_title(title, fontsize=11.5, pad=9)
    ax.set_xlabel('U — 주행방향 (mm)', fontsize=9)
    ax.set_ylabel('V — 횡방향 (mm)', fontsize=9)
    ax.set_aspect('equal')
    ax.tick_params(labelsize=8)
    ax.grid(alpha=0.15)


def _legend(fig, y):
    """범례. 그림 폭에 맞춰 줄 수를 나눈다 (한 줄에 6개면 오른쪽이 잘렸다)."""
    h = [
        plt.Line2D([], [], marker='o', ls='', color=C_TIED, mec='w', label='결속 완료'),
        plt.Line2D([], [], marker='X', ls='', color=C_FAIL, mec='w', label='장애물 (감지됨)'),
        plt.Line2D([], [], marker='s', ls='', color='#8d6e63', mec='#4e342e',
                   label='기둥 (주행 차단)'),
        plt.Line2D([], [], marker='s', ls='', color=C_LEFT, mec='w', label='미결속으로 남음'),
        plt.Line2D([], [], color=C_DRIVE, lw=1.6, label='주행 (싸다)'),
        plt.Line2D([], [], color=C_LAT, lw=3.2, label='횡이동 (비싸다 — 7배)'),
        plt.Line2D([], [], marker='o', ls='', mfc='white', mec='#37474f',
                   ms=9, label='출발 지점 (S) · 숫자 = 이동 순서'),
    ]
    ncol = 7 if fig.get_figwidth() > 10 else 4
    fig.legend(handles=h, loc='lower center', ncol=ncol, fontsize=8.5,
               frameon=False, bbox_to_anchor=(0.5, 0.004))


def _note_lines(notes):
    return max((str(n).count('\n') + 1) for n in notes if n) if any(notes) else 0


def _finish(fig, axes, notes, path):
    """고정 배치로 마무리한다.

    ⚠ `tight_layout` 을 쓰지 않는다. `aspect='equal'` 이면 축 상자가 그려질 때
      다시 줄어드는데, tight_layout 은 줄기 전 크기로 계산한다 — 그래서 요약을
      넣을 자리를 늘릴 때마다 **제목이 그림 밖으로 잘렸다**(두 번 겪었다).
      고정 비율로 잡고, **실제로 그려진 축 위치를 읽어** 글씨를 놓는다.
    """
    nl = _note_lines(notes)
    legend_h = 0.055 if fig.get_figwidth() > 10 else 0.095
    note_h = (0.050 + 0.024 * nl) * (6.6 / fig.get_figheight())
    one = len(axes) == 1
    fig.subplots_adjust(top=0.935, bottom=legend_h + note_h + 0.065,
                        left=0.095 if one else 0.055,
                        right=0.975 if one else 0.985, wspace=0.16)
    _legend(fig, legend_h)
    fig.canvas.draw()                      # 축 비율이 반영된 **실제** 위치를 얻는다
    for ax, nt in zip(axes, notes):
        if not nt:
            continue
        pos = ax.get_position()
        # 축 아래에 **매단다**(va='top'). 그림 아래에서 재면 x축 라벨과 겹친다 —
        # 축 높이가 비율 때문에 그릴 때 바뀌기 때문이다.
        fig.text(pos.x0 + pos.width / 2, pos.y0 - 0.072, nt, ha='center',
                 va='top', fontsize=8.5, color='#37474f', linespacing=1.45)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def save_one(world, trace, reach_bbox, title, path, note=''):
    fig, ax = plt.subplots(figsize=(7.8, 6.8))
    _draw(ax, world, trace, reach_bbox, title, note)
    return _finish(fig, [ax], [note], path)


def save_compare(world, left, right, reach_bbox, titles, path, notes=('', '')):
    fig, axs = plt.subplots(1, 2, figsize=(13.6, 6.8))
    for ax, tr, t, nt in zip(axs, (left, right), titles, notes):
        _draw(ax, world, tr, reach_bbox, t, nt)
    return _finish(fig, list(axs), list(notes), path)
