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
        # 결속영역은 **묶은 점이 많고, 그 묶음이 영역 한가운데 들어오는**
        # 정차지에 그린다.
        # ⚠ 첫 정차지는 판 모서리라 교차점이 덜 들어와 "한 번에 몇 점" 이
        #   안 보이고, 가장 많이 묶은 곳만 고르면 묶음이 영역 안에서 한쪽으로
        #   쏠려 보인다 (사용자 지적 2026-10-17). 가운데 정렬까지 보면
        #   사각·교차점·정차지 원이 저절로 같이 맞는다 — 위치를 꾸밀 필요가 없다.
        def _pick(i):
            ru, rv = trace['stops'][i]['pose']
            ins = [(u, v) for (u, v) in trace['cells']
                   if ru + b[0] <= u <= ru + b[1] and rv + b[2] <= v <= rv + b[3]]
            if not ins:
                return (0, 1e9)
            cu = sum(u for u, _v in ins) / len(ins)
            cv = sum(v for _u, v in ins) / len(ins)
            err = abs(cu - (ru + rmid[0])) + abs(cv - (rv + rmid[1]))
            return (len(ins), -err)
        k = max(range(len(trace['stops'])), key=_pick)
        ru, rv = trace['stops'][k]['pose']
        ax.add_patch(Rectangle((ru + b[0], rv + b[2]), b[1] - b[0], b[3] - b[2],
                               fc=C_REACH, alpha=0.10, ec=C_REACH, ls='--',
                               lw=1.6, zorder=2))
        inside = [(u, v) for (u, v) in trace['cells']
                  if ru + b[0] <= u <= ru + b[1] and rv + b[2] <= v <= rv + b[3]]
        for u, v in inside:
            ax.plot(u, v, 'o', ms=23, mfc='none', mec=C_REACH, mew=2.4, zorder=6)
        ax.text(ru + b[0] - 10, rv + b[2] - 14,
                f'한 정차지의 결속 가능 영역 — 교차점 {len(inside)}개',
                color=C_REACH, fontsize=11, va='top', ha='left', zorder=6)

    # 이동 경로 — **계획이 실제로 간 경로**를 정차지끼리 잇는다.
    # ⚠ ㄱ자를 다시 만들어 그리면 기둥을 돌아가는 경로가 **관통하는 선**이
    #   된다 (사용자 지적). 저장된 경로를 그대로 쓴다.
    P = [cen(s_['pose']) for s_ in trace['stops']]
    for i, s_ in enumerate(trace['stops'][:-1]):
        raw = s_.get('out') or [s_['pose'], trace['stops'][i + 1]['pose']]
        legs = [(cen(a), cen(b)) for a, b in zip(raw, raw[1:])
                if abs(b[0] - a[0]) > 1.0 or abs(b[1] - a[1]) > 1.0]
        for k, (p0, p1) in enumerate(legs):
            lat = abs(p1[1] - p0[1]) > 1.0
            ax.annotate('', xy=p1, xytext=p0, zorder=4,
                        arrowprops=dict(arrowstyle='-|>',
                                        shrinkA=11 if k == 0 else 0,
                                        shrinkB=11 if k == len(legs) - 1 else 0,
                                        color=C_LAT if lat else C_DRIVE,
                                        lw=3.4 if lat else 2.0, alpha=0.95))

    # 교차점 상태
    for (u, v), st in trace['cells'].items():
        col = {'tied': C_TIED, 'failed': C_FAIL, 'blocked': C_FAIL}.get(st, C_LEFT)
        mk = 'o' if st == 'tied' else ('X' if st in ('failed', 'blocked') else 's')
        ax.plot(u, v, mk, ms=15 if mk == 'X' else 12, color=col,
                mec='white', mew=1.2, zorder=5)

    # 정차지 — 번호가 곧 순서다. 같은 자리를 다시 들르면 한 동그라미에 모은다.
    # ⚠ 교차점끼리 화살표로 잇는 방식도 써 봤는데, 결속영역 중심이 교차점
    #   줄과 어긋나면(모서리 시작) 선이 줄 사이로 지나가 더 헷갈린다.
    order = {}
    for i, s_ in enumerate(trace['stops']):
        c_ = cen(s_['pose'])
        order.setdefault((round(c_[0]), round(c_[1])), []).append(i + 1)
    for (ru, rv), ns in order.items():
        lab = ','.join(str(n) for n in ns)
        w = 20 + 6 * (len(lab) - 1)
        ax.plot(ru, rv, 'o', ms=w, color='white', mec='#37474F', mew=1.8, zorder=7)
        ax.text(ru, rv, lab, ha='center', va='center',
                fontsize=11 if len(lab) < 3 else 9, color='#37474F', zorder=8)

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
    ax.set_title(title, fontsize=14, pad=11)
    ax.set_xlabel('U — 주행방향 (mm)', fontsize=11)
    ax.set_ylabel('V — 횡방향 (mm)', fontsize=11)
    ax.set_aspect('equal')
    ax.tick_params(labelsize=10)
    ax.grid(alpha=0.15)


def _legend(fig, y):
    """범례. 그림 폭에 맞춰 줄 수를 나눈다 (한 줄에 6개면 오른쪽이 잘렸다)."""
    h = [
        plt.Line2D([], [], marker='o', ls='', color=C_TIED, mec='w', label='결속 완료'),
        plt.Line2D([], [], marker='X', ls='', color=C_FAIL, mec='w',
                   label='결속 불가 지점 (주행은 가능)'),
        plt.Line2D([], [], marker='s', ls='', color='#8d6e63', mec='#4e342e',
                   label='기둥 (주행도 차단)'),
        plt.Line2D([], [], marker='s', ls='', color=C_LEFT, mec='w', label='미결속으로 남음'),
        plt.Line2D([], [], color=C_DRIVE, lw=1.6, label='주행 (싸다)'),
        plt.Line2D([], [], color=C_LAT, lw=3.2, label='횡이동 (비싸다 — 7배)'),
        plt.Line2D([], [], marker='o', ls='', mfc='white', mec='#37474f',
                   ms=11, label='정차지 (번호 = 순서)'),
    ]
    ncol = 7 if fig.get_figwidth() > 14 else 4
    fig.legend(handles=h, loc='lower center', ncol=ncol, fontsize=11,
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
    legend_h = 0.050 if fig.get_figwidth() > 14 else 0.075
    note_h = (0.050 + 0.024 * nl) * (9.2 / fig.get_figheight())
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
                 va='top', fontsize=11, color='#37474f', linespacing=1.45)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def save_one(world, trace, reach_bbox, title, path, note=''):
    fig, ax = plt.subplots(figsize=(11.0, 9.2))
    _draw(ax, world, trace, reach_bbox, title, note)
    return _finish(fig, [ax], [note], path)


def save_compare(world, left, right, reach_bbox, titles, path, notes=('', '')):
    fig, axs = plt.subplots(1, 2, figsize=(19.0, 9.2))
    for ax, tr, t, nt in zip(axs, (left, right), titles, notes):
        _draw(ax, world, tr, reach_bbox, t, nt)
    return _finish(fig, list(axs), list(notes), path)
