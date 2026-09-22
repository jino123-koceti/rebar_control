#!/usr/bin/env python3
"""주행모터 전류 소모 분석 — `motor_current.csv` 를 구간별로 뜯어본다.

## 왜
"주행 중 전류를 얼마나 쓰는가"는 세 가지를 결정한다:
  1. 배터리 운용시간   2. 모터 열여유(연속정격 대비)   3. 토크 상한을 얼마로 둘지

## 읽는 법 / 함정
- `ma_peak`은 프로브가 다운샘플하며 남긴 **구간 최대**다. `ma`(마지막값)만 보면
  스파이크를 통째로 놓친다.
- can_parser는 **엔코더 응답(0x90/0x92/0x94)도 `current_current=0`으로 발행**한다.
  0을 그대로 평균에 넣으면 실제보다 훨씬 낮게 나온다 → **주행 구간만** 골라야 한다.
- 전류값은 **진폭(iq) 단위**로 보인다. rms는 ÷√2.
  (X4-36에서 실측 30.5A ÷ √2 = 21.6A ≈ 데이터시트 최대 21.5A(rms)로 확인됨)

사용:
    python3 tools/diag/drive_current_analyze.py                # 오늘 전체
    python3 tools/diag/drive_current_analyze.py --since "2026-08-18 13:41"
"""
import argparse
import math
from collections import defaultdict

CSV = '/var/log/robot_control/motor_current.csv'
NAME = {'0x141': '좌 (0x141)', '0x142': '우 (0x142)', '0x143': '횡이동 (0x143)'}


def pct(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    i = min(len(sorted_vals) - 1, int(len(sorted_vals) * p))
    return sorted_vals[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=CSV)
    ap.add_argument('--since', default='', help='"YYYY-MM-DD HH:MM" 이후만')
    ap.add_argument('--moving-dps', type=float, default=20.0,
                    help='이 속도 초과를 "주행 중"으로 본다')
    a = ap.parse_args()

    rows = []
    with open(a.csv) as f:
        hdr = f.readline().rstrip('\n').split(',')
        for line in f:
            r = line.rstrip('\n').split(',')
            if len(r) != len(hdr) or not r[2].startswith('0x'):
                continue
            if a.since and r[0] < a.since:
                continue
            rows.append(r)
    if not rows:
        print('데이터 없음'); return

    i_ma, i_pk, i_sp = hdr.index('ma'), hdr.index('ma_peak'), hdr.index('speed')

    drive = defaultdict(list)   # 주행 중 peak 전류
    idle = defaultdict(list)    # 정지 중
    spd = defaultdict(list)
    for r in rows:
        mid = r[2]
        if mid not in NAME:
            continue
        try:
            pk = abs(int(r[i_pk] or 0))
            sp = abs(float(r[i_sp] or 0))
        except ValueError:
            continue
        if pk == 0 and sp == 0:
            continue                       # 엔코더 응답 (전류 정보 없음)
        if sp > a.moving_dps:
            drive[mid].append(pk)
            spd[mid].append(sp)
        else:
            idle[mid].append(pk)

    print('=' * 72)
    print('구간: %s ~ %s   (총 %d 레코드)' % (rows[0][0], rows[-1][0], len(rows)))
    print('=' * 72)

    for mid in sorted(NAME):
        d = sorted(drive[mid])
        if not d:
            continue
        n = len(d)
        mean = sum(d) / n
        s = sorted(spd[mid])
        print('\n■ %s   주행 샘플 %d' % (NAME[mid], n))
        print('  전류(진폭)  평균 %5.2f A   중앙 %5.2f   p90 %5.2f   p99 %5.2f   최대 %5.2f A'
              % (mean / 1000, pct(d, .5) / 1000, pct(d, .9) / 1000,
                 pct(d, .99) / 1000, d[-1] / 1000))
        print('  전류(rms)   평균 %5.2f A                                        최대 %5.2f A'
              % (mean / 1000 / math.sqrt(2), d[-1] / 1000 / math.sqrt(2)))
        print('  속도        평균 %5.0f dps  중앙 %5.0f  최대 %5.0f dps'
              % (sum(s) / len(s), pct(s, .5), s[-1]))
        # 부하 구간 분포
        bands = [(0, 3000), (3000, 6000), (6000, 10000), (10000, 13000), (13000, 99000)]
        print('  부하 분포:', end='')
        for lo, hi in bands:
            c = sum(1 for v in d if lo <= v < hi)
            print('  %2d~%2dA %4.1f%%' % (lo / 1000, hi / 1000 if hi < 99000 else 99,
                                          100.0 * c / n), end='')
        print()
        if idle[mid]:
            it = sorted(idle[mid])
            print('  (정지 중 평균 %.2f A, 최대 %.2f A — 홀딩/마찰)'
                  % (sum(it) / len(it) / 1000, it[-1] / 1000))

    # 좌우 불균형
    L, R = drive.get('0x141'), drive.get('0x142')
    if L and R:
        ml, mr = sum(L) / len(L), sum(R) / len(R)
        print('\n' + '=' * 72)
        print('■ 좌우 균형')
        print('  주행 평균   좌 %.2f A  vs  우 %.2f A   → 우측이 %+.0f%% '
              % (ml / 1000, mr / 1000, 100.0 * (mr - ml) / ml))
        print('  주행 최대   좌 %.2f A  vs  우 %.2f A'
              % (max(L) / 1000, max(R) / 1000))
        # 전기 출력 추정 (버스 28V 가정, rms 기준)
        p = (ml + mr) / 1000 / math.sqrt(2) * 28.0
        print('  주행 중 두 모터 합산 소비(개산, 28V·rms) 약 %.0f W' % p)


if __name__ == '__main__':
    main()
