#!/usr/bin/env python3
"""고부하 구간 전류 파형을 보고서용 JPG 피겨로 저장한다.

## 왜
문서(HWP/Word/PPT)에 바로 붙일 그림이 필요하다. 아티팩트 페이지는 공유·열람용이고,
보고서 본문에는 정지 이미지가 있어야 한다.

## 읽는 법 / 함정
- `ma_peak`은 프로브가 다운샘플하며 남긴 **구간 최대**다. 다만 **초기 버전에서는 이 필드가
  0으로 기록**된 구간이 있다(2026-08-14 오전). 그래서 `max(|ma|, |ma_peak|)`를 쓴다.
- can_parser는 엔코더 응답(0x90/0x92/0x94)도 `current_current=0`으로 발행하므로,
  0만 있는 구간을 그대로 그리면 무부하처럼 보인다. 초 단위 최대값으로 집계해 피한다.
- 값은 진폭(iq) 단위다. 실효값은 ÷√2.

사용:
    python3 tools/diag/plot_load_figure.py \
        --from "2026-08-14 13:08:50" --to "2026-08-14 13:09:13" \
        --title "궤도 이탈·복귀 구간" --limit 16.03 --out fig1.jpg
"""
import argparse
import csv
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

CSV = '/var/log/robot_control/motor_current.csv'
ALL_SERIES = {
    '0x141': ('좌 모터 (0x141)', '#2a78d6'),
    '0x142': ('우 모터 (0x142)', '#eb6834'),
    '0x143': ('횡이동 모터 (0x143)', '#2a78d6'),
}
SERIES = []   # main()에서 --motors 로 채운다


def load(csv_path, t0, t1):
    out = {k: {} for k, _, _ in SERIES}
    base = datetime.strptime(t0, '%Y-%m-%d %H:%M:%S')
    with open(csv_path) as f:
        rd = csv.reader(f)
        hdr = next(rd)
        ima, ipk, isp = hdr.index('ma'), hdr.index('ma_peak'), hdr.index('speed')
        for r in rd:
            if len(r) != len(hdr) or r[2] not in out:
                continue
            if not (t0 <= r[0] <= t1):
                continue
            try:
                ma = abs(int(r[ima] or 0))
                pk = abs(int(r[ipk] or 0))
                sp = abs(float(r[isp] or 0))
            except ValueError:
                continue
            sec = int((datetime.strptime(r[0], '%Y-%m-%d %H:%M:%S') - base).total_seconds())
            e = out[r[2]].setdefault(sec, [0.0, 0.0])
            e[0] = max(e[0], max(ma, pk) / 1000.0)     # ma_peak 고장 구간 대비
            e[1] = max(e[1], sp)
    return {k: sorted(v.items()) for k, v in out.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=CSV)
    ap.add_argument('--from', dest='t0', required=True)
    ap.add_argument('--to', dest='t1', required=True)
    ap.add_argument('--title', required=True)
    ap.add_argument('--subtitle', default='')
    ap.add_argument('--limit', type=float, default=0.0, help='토크 상한 기준선 (A)')
    ap.add_argument('--limit-label', default='')
    ap.add_argument('--base', type=float, default=0.0, help='평상시 전류 기준선 (A)')
    ap.add_argument('--rated', type=float, default=0.0,
                    help='연속 정격 전류 기준선 (A, 진폭). 위쪽은 단시간만 허용 영역으로 음영')
    ap.add_argument('--rated-label', default='')
    ap.add_argument('--avg', type=float, default=0.0, help='이동 중 평균 주석 (A)')
    ap.add_argument('--ymax', type=float, default=0.0)
    ap.add_argument('--motors', default='0x141,0x142',
                    help='그릴 모터 (쉼표 구분)')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()

    global SERIES
    SERIES = [(m, ALL_SERIES[m][0], ALL_SERIES[m][1])
              for m in a.motors.split(',') if m in ALL_SERIES]

    for cand in ('NanumGothic', 'NanumBarunGothic', 'Noto Sans CJK JP'):
        if any(f.name == cand for f in font_manager.fontManager.ttflist):
            plt.rcParams['font.family'] = cand
            break
    plt.rcParams['axes.unicode_minus'] = False

    data = load(a.csv, a.t0, a.t1)
    peak = max((v for s in data.values() for _, (v, _) in s), default=0.0)
    ymax = a.ymax or (peak * 1.28)

    fig, ax = plt.subplots(figsize=(9.6, 4.5), dpi=160)
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    if a.limit:
        ax.axhline(a.limit, ls='--', lw=1.3, color='#78766f', alpha=.85, zorder=1)
        ax.text(0.995, a.limit + ymax * .018,
                a.limit_label or f'토크 상한 {a.limit:.1f} A',
                transform=ax.get_yaxis_transform(), ha='right',
                fontsize=8.5, color='#52514e')
    # 연속 정격 — 위쪽은 '단시간만 허용' 영역
    if a.rated:
        ax.axhspan(a.rated, ymax, color='#eb6834', alpha=.055, zorder=0)
        ax.axhline(a.rated, ls=(0, (7, 3)), lw=1.4, color='#c05a2c', alpha=.9, zorder=2)
        ax.text(0.012, a.rated + ymax * .028,
                a.rated_label or f'연속 정격 {a.rated:.1f} A',
                transform=ax.get_yaxis_transform(), ha='left',
                fontsize=8.5, color='#a8481f')
        ax.text(0.012, 0.955, '↑ 연속 정격 초과 — 단시간만 허용',
                transform=ax.transAxes, ha='left', fontsize=8.5, color='#a8481f')
    if a.base:
        ax.axhline(a.base, ls=':', lw=1.1, color='#9a9890', alpha=.9, zorder=1)
        ax.text(0.995, a.base + ymax * .018, f'평상시 {a.base:.1f} A',
                transform=ax.get_yaxis_transform(), ha='right',
                fontsize=8.5, color='#78766f')

    for key, label, color in SERIES:
        pts = data.get(key) or []
        if not pts:
            continue
        xs = [t for t, _ in pts]
        ys = [v for _, (v, _) in pts]
        ax.plot(xs, ys, lw=2, color=color, label=label,
                solid_joinstyle='round', solid_capstyle='round', zorder=3)

    # 최대점 표시
    top_key, top_t, top_v = None, 0, 0.0
    for key, _, _ in SERIES:
        for t, (v, _) in data.get(key) or []:
            if v > top_v:
                top_key, top_t, top_v = key, t, v
    if top_key:
        color = dict((k, c) for k, _, c in SERIES)[top_key]
        ax.plot([top_t], [top_v], 'o', ms=7, color=color,
                mec='white', mew=1.8, zorder=4)
        ax.annotate(f'{top_v:.1f} A', (top_t, top_v),
                    textcoords='offset points', xytext=(0, 11),
                    ha='center', fontsize=10, fontweight='bold', color='#0b0b0b')

    if a.avg:
        ax.axhline(a.avg, ls='-', lw=1.2, color='#52514e', alpha=.55, zorder=2)
        ax.text(0.988, a.avg - ymax * .045, f'이동 중 평균 {a.avg:.1f} A',
                transform=ax.get_yaxis_transform(), ha='right',
                fontsize=8.5, color='#52514e')

    ax.set_xlabel('경과 시간 (초)', fontsize=9.5, color='#52514e')
    ax.set_ylabel('모터 전류 (A)', fontsize=9.5, color='#52514e')
    ax.set_ylim(0, ymax)
    ax.set_xlim(0, max((t for s in data.values() for t, _ in s), default=1))
    ax.grid(axis='y', color='#e6e5e1', lw=1)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color('#dedddaff')
    ax.tick_params(colors='#78766f', labelsize=9)

    # 제목 / 부제 / 범례를 축 위에 층으로 쌓는다 (겹침·데이터 가림 방지)
    multi = len(SERIES) > 1
    ax.set_title(a.title, fontsize=13, fontweight='bold',
                 color='#0b0b0b', loc='left', pad=46 if multi else 30)
    if a.subtitle:
        ax.text(0, 1.075 if multi else 1.03, a.subtitle, transform=ax.transAxes,
                fontsize=9, color='#78766f', va='bottom')
    if multi:
        ax.legend(frameon=False, fontsize=9.5, loc='lower left',
                  bbox_to_anchor=(0, 1.005), ncol=len(SERIES),
                  labelcolor='#52514e', handlelength=1.6, columnspacing=1.6)

    fig.tight_layout()
    fig.savefig(a.out, format='jpg', facecolor='white',
                pil_kwargs={'quality': 95})
    print(f'저장: {a.out}   구간 최대 {peak:.2f} A')


if __name__ == '__main__':
    main()
