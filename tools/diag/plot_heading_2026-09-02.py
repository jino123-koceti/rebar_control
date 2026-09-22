#!/usr/bin/env python3
"""헤딩 제어 응답 — 오프셋 수정 후 (2026-09-02 18:27 주행).

계측: tools 스크래치 heading_watch.py
  /deck_edge_status 의 heading_deg(raw) 를 스텝 시작/끝에 기록하고,
  /cmd_vel 의 angular.z 를 스텝 구간 평균·최대로 낸다.

★ 목표는 0이 아니라 **오프셋**이다 — raw heading이 오프셋과 같으면 물리적 직진.
  이날 12시 정렬 실측: front -0.86° / back -0.69°
  (직전까지 쓰던 back +1.52°는 2.21° 틀린 값이었고, 그래서 후진마다 좌측으로
   틀어져 1시 출발 → 11시 도착이 났다.)
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch

for cand in ('NanumGothic', 'NanumSquareRound', 'Noto Sans CJK KR', 'Noto Sans KR'):
    if any(cand in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = cand
        break
plt.rcParams['axes.unicode_minus'] = False

BLUE, ORANGE = '#2a78d6', '#eb6834'      # dataviz 검증 통과
INK, INK2, MUTED, SURFACE = '#0b0b0b', '#52514e', '#b8b7b0', '#fcfcfb'

# (구간, 목표, raw시작, raw끝, angular평균, angular최대)
STEPS = [
    ('FWD', -0.86, +2.09, -1.05, -0.074, 0.180),
    ('FWD', -0.86, -1.10, -0.29, -0.005, 0.061),
    ('FWD', -0.86, -0.49, -0.81, -0.029, 0.036),
    ('FWD', -0.86, -0.87, -0.92, +0.000, 0.000),
    ('REV', -0.69, -0.92, -0.93, +0.008, 0.028),
    ('REV', -0.69, -0.76, -0.92, -0.009, 0.034),
    ('REV', -0.69, -0.91, -0.69, +0.007, 0.029),
    ('REV', -0.69, -0.92, -0.77, +0.003, 0.024),
]
DEADBAND, AMAX = 0.2, 0.18

fig, (ax, ax2) = plt.subplots(
    2, 1, figsize=(11, 8), sharex=True, facecolor=SURFACE,
    gridspec_kw={'height_ratios': [2.3, 1]})
fig.subplots_adjust(hspace=0.16, top=0.855, bottom=0.13, left=0.095, right=0.97)

# ── 상단: 목표 대비 오차 궤적 ────────────────────────────────────────
ax.axhspan(-DEADBAND, DEADBAND, color=MUTED, alpha=0.25, zorder=0)
ax.axhline(0, color=INK, lw=1.4, zorder=2)
ax.text(0.06, 0.09, '목표 = 물리적 직진',
        fontsize=9, color=INK, va='bottom')
ax.text(0.06, -DEADBAND - 0.14, f'데드밴드 ±{DEADBAND}° (조향 안 함)',
        fontsize=8.5, color=INK2, va='top')

for i, (kind, tg, r0, r1, _, _) in enumerate(STEPS):
    c = BLUE if kind == 'FWD' else ORANGE
    e0, e1 = r0 - tg, r1 - tg
    ax.plot([i, i + 0.72], [e0, e1], '-', color=c, lw=2.4, zorder=4)
    ax.plot([i], [e0], 'o', color=c, ms=8, zorder=5)
    ax.plot([i + 0.72], [e1], 'o', color=c, ms=8, mfc=SURFACE, mew=2.2, zorder=5)
    if i + 1 < len(STEPS):                     # 스텝 사이(검출·결속 구간)
        nk, ntg = STEPS[i + 1][0], STEPS[i + 1][1]
        ax.plot([i + 0.72, i + 1], [e1, STEPS[i + 1][2] - ntg],
                ':', color=MUTED, lw=1.4, zorder=3)
    lx, ly = (i + 0.5, e0 - 0.30) if i == 0 else (i + 0.36, max(e0, e1) + 0.16)
    ax.text(lx, ly, f'{e1 - e0:+.2f}°', ha='center', fontsize=8.5,
            color=BLUE if i == 0 else INK2,
            fontweight='bold' if i == 0 else 'normal')

ax.set_ylabel('목표 대비 오차 (°)', fontsize=10.5, color=INK2)
ax.set_ylim(-1.1, 3.25)
ax.set_title('헤딩 제어 응답 — 오프셋 수정 후', fontsize=14.5,
             color=INK, pad=30, loc='left', fontweight='bold')
ax.text(0, 1.055,
        '초기 2.95도 오차를 1스텝에 잡고 이후 ±0.6° 유지 · 방향 전환 튐 없음   '
        '— 2026-09-02 18:27 주행, kp 0.08 / max 0.18 / 데드밴드 0.2',
        transform=ax.transAxes, fontsize=9.5, color=INK2, va='bottom')
ax.legend(handles=[
    Patch(facecolor=BLUE,  label='전진 (목표 -0.86°)'),
    Patch(facecolor=ORANGE, label='후진 (목표 -0.69°)')],
    frameon=False, fontsize=10, loc='upper right', ncol=2)
ax.grid(axis='y', color=MUTED, alpha=0.3, lw=0.7, zorder=0)
for s in ('top', 'right'):
    ax.spines[s].set_visible(False)
for s in ('left', 'bottom'):
    ax.spines[s].set_color(MUTED)
ax.set_facecolor(SURFACE)
ax.text(7.85, -0.95, '● 스텝 시작   ○ 스텝 끝   ⋯ 스텝 사이(검출·결속)',
        ha='right', fontsize=8.5, color=MUTED)

# ── 하단: 조향 지령 ─────────────────────────────────────────────────
ax2.axhline(AMAX, color=INK, lw=1.3, ls='--', zorder=4)
ax2.text(7.85, AMAX + 0.006, f'상한 {AMAX} rad/s', ha='right',
         fontsize=8.5, color=INK, fontweight='bold')
for i, (kind, _, _, _, wm, wx) in enumerate(STEPS):
    c = BLUE if kind == 'FWD' else ORANGE
    ax2.bar(i + 0.36, wx, 0.62, color=c, alpha=0.35, zorder=3)
    ax2.bar(i + 0.36, abs(wm), 0.62, color=c, zorder=4)
    if wx > 0.005:
        ax2.text(i + 0.36, wx + 0.006, f'{wx:.3f}', ha='center',
                 fontsize=8, color=INK2)

ax2.set_ylabel('|angular.z| (rad/s)', fontsize=10.5, color=INK2)
ax2.text(3.4, 0.185, '진한색 = 구간 평균,  옅은색 = 구간 최대',
         fontsize=8.5, color=MUTED, va='top')
ax2.set_ylim(0, 0.215)
ax2.set_xlim(-0.45, 7.95)
ax2.set_xticks([i + 0.36 for i in range(len(STEPS))])
ax2.set_xticklabels([f'#{i+1}\n{k}' for i, (k, *_ ) in enumerate(STEPS)],
                    fontsize=9, color=INK2)
ax2.grid(axis='y', color=MUTED, alpha=0.3, lw=0.7, zorder=0)
for s in ('top', 'right'):
    ax2.spines[s].set_visible(False)
for s in ('left', 'bottom'):
    ax2.spines[s].set_color(MUTED)
ax2.set_facecolor(SURFACE)

fig.text(0.095, 0.052,
         '오프셋 실측(12시 정렬): front -0.86° / back -0.69°  '
         '— 직전 back +1.52°는 2.21° 오차였고 후진마다 좌측 편향을 만들었다\n'
         '전·후진 목표 차이가 2.65° → 0.17°로 줄어 방향 전환 시 자세가 유지된다',
         fontsize=8.5, color=MUTED, va='top')

OUT = '/home/koceti/ros2_ws/docs/reports/figures/heading_response_2026-09-02'
fig.savefig(OUT + '.png', dpi=170, facecolor=SURFACE)
fig.savefig(OUT + '.jpg', dpi=170, facecolor=SURFACE)
print('저장:', OUT + '.png / .jpg')
