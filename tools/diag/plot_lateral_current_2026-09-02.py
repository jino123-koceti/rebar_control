#!/usr/bin/env python3
"""횡이동(0x143) 전류 실측 — 무부하 vs 배근 위 (2026-09-02).

계측: tools 스크래치의 lat_current.py (0x9C Read Motor Status 2를 20Hz 폴링).
      0x143은 위치명령(0xA4) 응답이 1회뿐이라 폴링하지 않으면 이동 중 전류를 못 본다.

⚠ 원시 파형은 저장하지 않았고 회전 구간별 요약값만 있다. 그래서 파형이 아니라
  **구간 통계 비교**로 그린다. 파형이 필요하면 재계측해야 한다.

데이터시트(myactuator, X4-36): 정격 상전류 6.1 A(rms) / 최대 상전류 21.5 A(rms).
0x9C가 주는 값은 **진폭(iq)** 이라 rms로 보려면 ÷√2 — 그래서 기준선은 ×√2 해서 그린다.
"""
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

# 한글 폰트
for cand in ('NanumGothic', 'NanumSquareRound', 'Noto Sans CJK KR', 'Noto Sans KR'):
    if any(cand in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = cand
        break
plt.rcParams['axes.unicode_minus'] = False

BLUE, ORANGE = '#2a78d6', '#eb6834'      # dataviz 검증 통과 (ΔE 24.7 protan)
INK, INK2, MUTED = '#0b0b0b', '#52514e', '#b8b7b0'
SURFACE = '#fcfcfb'

# ── 실측 (2026-09-02) ────────────────────────────────────────────────
# 무부하 = 장비를 들어올린 상태, 배근 위 = 실제 작업 자세
LABELS = ['무부하\n90dps\n(11회)', '무부하\n70dps\n(4회)',
          '배근 위\n70dps #1', '배근 위\n70dps #2', '배근 위\n70dps #3']
AVG  = [3.30, 2.97,  7.84, 14.98, 30.01]
PEAK = [4.80, 4.20, 30.47, 30.16, 30.01]
DUR  = [5.16, 5.20,  2.40,  1.20,  0.40]      # 회전 소요(초) — 배근 위는 전부 미완료
TEMP = [40, 40, 36, 55, 63]                    # 구간 종료 시점 온도(°C)
OK   = [True, True, False, False, False]       # 회전 완료 여부

RATED_RMS, MAX_RMS = 6.1, 21.5                 # 데이터시트 상전류
RATED, MAXC = RATED_RMS * math.sqrt(2), MAX_RMS * math.sqrt(2)   # 진폭 환산

fig, (ax, ax2) = plt.subplots(
    2, 1, figsize=(10.5, 8.4), sharex=True, facecolor=SURFACE,
    gridspec_kw={'height_ratios': [2.5, 1]})   # mpl 3.5는 height_ratios 인자 미지원
fig.subplots_adjust(hspace=0.30, top=0.86, bottom=0.155, left=0.115, right=0.97)

# ── 상단: 전류 ──────────────────────────────────────────────────────
x = range(len(LABELS))
w = 0.36
for i in x:
    ax.bar(i - w/2 - 0.01, AVG[i],  w, color=BLUE,
           label='구간 평균' if i == 0 else None, zorder=3)
    ax.bar(i + w/2 + 0.01, PEAK[i], w, color=ORANGE,
           label='구간 피크' if i == 0 else None, zorder=3)
    ax.text(i - w/2 - 0.01, AVG[i] + 0.6, f'{AVG[i]:.1f}',
            ha='center', va='bottom', fontsize=9, color=INK2)
    ax.text(i + w/2 + 0.01, PEAK[i] + 0.6, f'{PEAK[i]:.1f}',
            ha='center', va='bottom', fontsize=9,
            color=INK if PEAK[i] > 20 else INK2,
            fontweight='bold' if PEAK[i] > 20 else 'normal')

# 데이터시트 기준선
ax.axhline(MAXC, color=INK, lw=1.6, ls='--', zorder=4)
ax.text(-0.42, MAXC + 0.55,
        f'최대 상전류 {MAX_RMS} A(rms) = {MAXC:.1f} A(진폭)',
        ha='left', va='bottom', fontsize=9, color=INK, fontweight='bold')
ax.axhline(RATED, color=MUTED, lw=1.3, ls=':', zorder=4)
ax.text(-0.42, RATED + 0.45,
        f'정격 상전류 {RATED_RMS} A(rms) = {RATED:.1f} A(진폭)',
        ha='left', va='bottom', fontsize=8.5, color=INK2)

ax.set_ylabel('모터 상전류 iq (A, 진폭)', fontsize=10.5, color=INK2)
ax.set_ylim(0, 37.5)
ax.set_title('횡이동 모터(0x143, RMD X4-36) 전류 — 무부하 vs 배근 위',
             fontsize=14, color=INK, pad=26, loc='left', fontweight='bold')
ax.text(0, 1.035,
        '배근 위 3회 시도 모두 최대 상전류에 포화(100%)하고도 목표 미도달 — 2026-09-02 실측, 0x9C 20Hz 폴링',
        transform=ax.transAxes, fontsize=10, color=INK2, va='bottom')
ax.legend(frameon=False, fontsize=10, loc='upper left', ncol=2)
ax.grid(axis='y', color=MUTED, alpha=0.35, lw=0.7, zorder=0)
for s in ('top', 'right'):
    ax.spines[s].set_visible(False)
for s in ('left', 'bottom'):
    ax.spines[s].set_color(MUTED)
ax.set_facecolor(SURFACE)

# 미완료 표시
for i in x:
    if not OK[i]:
        ax.text(i, 34.2, '스톨', ha='center', fontsize=9.5,
                color=ORANGE, fontweight='bold')

# ── 하단: 회전 소요시간 (완료 여부) ──────────────────────────────────
for i in x:
    ax2.bar(i, DUR[i], 0.5, color=(BLUE if OK[i] else ORANGE), zorder=3)
    ax2.text(i, DUR[i] + 0.12,
             f'{DUR[i]:.1f}s' + ('' if OK[i] else '  미완료'),
             ha='center', va='bottom', fontsize=9,
             color=INK2 if OK[i] else ORANGE)
    # 막대가 짧으면 안에 못 넣는다 → 위로 뺀다
    if DUR[i] >= 1.0:
        ax2.text(i, 0.18, f'{TEMP[i]}°C', ha='center', fontsize=8.5,
                 color='white', fontweight='bold')
    else:
        ax2.text(i, DUR[i] + 0.55, f'{TEMP[i]}°C', ha='center', fontsize=8.5,
                 color=ORANGE, fontweight='bold')

ax2.set_ylabel('1회전 소요 (초)\n막대 안 = 종료 온도', fontsize=10, color=INK2)
ax2.set_ylim(0, 6.6)
ax2.set_xticks(list(x))
ax2.set_xticklabels(LABELS, fontsize=9.5, color=INK2)
ax2.grid(axis='y', color=MUTED, alpha=0.35, lw=0.7, zorder=0)
for s in ('top', 'right'):
    ax2.spines[s].set_visible(False)
for s in ('left', 'bottom'):
    ax2.spines[s].set_color(MUTED)
ax2.set_facecolor(SURFACE)

fig.text(0.115, 0.045,
         '무부하 = 장비를 들어올린 상태 · 배근 위 = 실제 작업 자세 (승강 부하 포함)\n'
         '0x9C가 주는 전류는 진폭(iq) — 데이터시트 rms 값에 √2를 곱해 같은 축에 두었다',
         fontsize=8.5, color=MUTED, va='top')

OUT = '/home/koceti/ros2_ws/docs/reports/figures/lateral_current_2026-09-02'
fig.savefig(OUT + '.png', dpi=170, facecolor=SURFACE)
fig.savefig(OUT + '.jpg', dpi=170, facecolor=SURFACE)
print('저장:', OUT + '.png / .jpg')
