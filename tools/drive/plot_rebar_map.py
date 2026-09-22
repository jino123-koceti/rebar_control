#!/usr/bin/env python3
"""주행 기록(JSONL) → **철근 배근 작업도** 플롯.

## 무엇을 그리나
자율주행 중 매 검출마다 기록된 교차점을 **전역 좌표로 합쳐** 한 장에 그린다.
  · 교차점 위치 (격자)
  · 결속됨(tie) / 미결속(untie) / 미분류(crossing)
  · 이번 작업에서 **실제로 결속 요청한 점**
  · 로봇 주행 경로

## 좌표 복원
검출 좌표는 **로봇 기준 stage mm**이고, 로봇은 x축으로만 이동한다(횡이동 제외).
    global_x = odom_x_mm + local_x        global_y = local_y + 레인오프셋
전진하면 odom_x가 늘고 같은 교차점의 local_x는 그만큼 줄어 상쇄된다 → 고정점은 고정.

⚠ **엔코더 odom은 직진 슬립이 거의 없다**(2026-08-19 실측: 명령 대비 100%).
   회전만 크게 미끄러지는데, 직진 스텝 주행이라 여기선 문제가 안 된다.

## 중복 병합
같은 교차점이 여러 스텝에서 반복 검출된다. `--merge` (기본 60mm) 안이면 한 점으로
합치고, 클래스는 **다수결 + tie 우선**으로 정한다
(한 번이라도 tie로 보였으면 결속된 것으로 본다 — 결속 흔적은 사라지지 않는다).

사용:
    python3 tools/drive/plot_rebar_map.py                      # 가장 최근 기록
    python3 tools/drive/plot_rebar_map.py --file data/rebar_map/run_*.jsonl
    python3 tools/drive/plot_rebar_map.py --csv                # 좌표 CSV도 저장
"""
import argparse
import glob
import json
import os
from collections import defaultdict

REC_DIR = '/home/koceti/ros2_ws/data/rebar_map'
COLOR = {'tie': '#2e9b3f', 'untie': '#d43b2f', 'crossing': '#8a8a8a', None: '#8a8a8a'}
LABEL = {'tie': '결속됨 (tie)', 'untie': '미결속 (untie)', 'crossing': '미분류'}


def load(path):
    recs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                recs.append(json.loads(line))
            except ValueError:
                continue          # 주행 중 강제종료로 마지막 줄이 잘릴 수 있다
    return recs


CALIB = '/home/koceti/ros2_ws/data/calibration/cad_transform_orbbec.yaml'


def pose_norm():
    """자세별 오프셋 차이 → 'r' 기준으로 정규화할 보정값.

    ★ 기록된 좌표는 교차점의 **물리 위치가 아니라 '결속기를 보낼 스테이지 좌표'**다.
      자세(r/l)에 따라 결속기 위치가 다르므로 같은 교차점도 값이 달라진다.
      실측(2026-08-19): 같은 점을 l이 (363.3, 35.8), r이 (378.8, 16.7)로 기록 —
      차이 (-15.5, +19.1)이 CAD 오프셋 차이 l-r=(-13.0, +20.6)와 일치했다.
      정규화 안 하면 한 교차점이 두 개로 갈라져 격자 간격이 86~122mm로 찍힌다.
    """
    try:
        import yaml
        po = yaml.safe_load(open(CALIB))['pose_offset']
        r, l = po['r'], po['l']
        return {'r': (0.0, 0.0),
                'l': (float(r[0]) - float(l[0]), float(r[1]) - float(l[1]))}
    except Exception as e:
        print(f'⚠ 자세 오프셋 로드 실패 → 정규화 없이 진행: {e}')
        return {}


def build(recs, merge_mm):
    """검출을 전역좌표로 펴고 병합. (points, path, tie_reqs) 반환."""
    norm = pose_norm()
    raw, path, tie_reqs = [], [], []
    for r in recs:
        if r.get('type') != 'detect':
            continue
        ox = r.get('odom_x_mm')
        if ox is None:
            continue
        lane = r.get('lane', 0) or 0
        nx, ny = norm.get(r.get('pose'), (0.0, 0.0))   # 'r' 기준 정규화
        path.append((ox, lane, r.get('dir'), r.get('heading_deg')))
        for d in r.get('det', []):
            if len(d) < 2:
                continue
            raw.append((ox + float(d[0]) + nx, float(d[1]) + ny,
                        d[2] if len(d) > 2 else None, lane))
        # 결속 요청 좌표는 **그 점을 결속할 때 쓴 자세**의 좌표계다(side가 곧 자세).
        for side, pts in (r.get('tie_req') or {}).items():
            sx, sy = norm.get(side, (0.0, 0.0))
            for p in pts:
                tie_reqs.append((ox + float(p[0]) + sx, float(p[1]) + sy, side))

    points = _cluster(raw, merge_mm)
    return points, path, tie_reqs


def _cluster(raw, merge_mm):
    """거리 기준 병합. 격자 버킷 + **이웃 버킷까지** 확인한다.

    ⚠ 버킷만 쓰면 경계에 걸친 같은 점이 갈라진다 — 실제로 20mm 떨어진 한 점이
      270/290으로 나뉘어 두 개로 그려졌다(2026-08-19). 이웃 8칸을 같이 봐야 한다.
    """
    cell = merge_mm            # 한 칸 = 병합반경 → 이웃 3x3이면 반경을 덮는다
    grid = defaultdict(list)   # (bx,by) -> [cluster index]
    cl_sum, cl_n, cl_votes = [], [], []

    def key(x, y):
        return int(x // cell), int(y // cell)

    for x, y, cls, lane in raw:
        bx, by = key(x, y)
        best, bestd = None, merge_mm ** 2
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for ci in grid.get((bx + dx, by + dy), ()):
                    cx = cl_sum[ci][0] / cl_n[ci]
                    cy = cl_sum[ci][1] / cl_n[ci]
                    d = (cx - x) ** 2 + (cy - y) ** 2
                    if d < bestd:
                        best, bestd = ci, d
        if best is None:
            ci = len(cl_n)
            cl_sum.append([x, y]); cl_n.append(1)
            cl_votes.append(defaultdict(int)); cl_votes[ci][cls] += 1
            grid[(bx, by)].append(ci)
        else:
            cl_sum[best][0] += x; cl_sum[best][1] += y; cl_n[best] += 1
            cl_votes[best][cls] += 1
            # 중심이 옮겨가면 버킷도 바뀔 수 있다 → 새 버킷에도 등록(중복 무해)
            nb = key(cl_sum[best][0] / cl_n[best], cl_sum[best][1] / cl_n[best])
            if best not in grid[nb]:
                grid[nb].append(best)

    out = []
    for i in range(len(cl_n)):
        v = cl_votes[i]
        # tie 우선 — 결속 흔적은 한 번만 보여도 결속된 것이다(사라지지 않는다)
        cls = 'tie' if v.get('tie') else (
            max(v.items(), key=lambda kv: kv[1])[0] if v else None)
        out.append((cl_sum[i][0] / cl_n[i], cl_sum[i][1] / cl_n[i], cls, cl_n[i]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=None, help='기록 JSONL (기본: 가장 최근)')
    ap.add_argument('--merge', type=float, default=60.0, help='중복 병합 반경(mm)')
    # ★ 관측 횟수 필터 (2026-08-19 실측 근거).
    #   한 번만 잡힌 점은 오검출일 확률이 높다 — 실측에서 n=1 점만 간격이
    #   61~100mm(배근 pitch 205의 절반 이하)로 어긋났고, 2회 이상 관측된 점은
    #   예외 없이 173~246mm로 규칙적이었다. 시야 가장자리(Y≈404)에 몰려 있었다.
    #   ⚠ 기본 1(전부 표시) — 주행 양 끝의 **진짜 교차점도 1회만 잡히기 때문**.
    #     깨끗한 도면을 뽑을 때만 2를 준다.
    ap.add_argument('--min-detect', type=int, default=1,
                    dest='min_detect', help='이 횟수 미만 관측된 점은 제외')
    ap.add_argument('--out', default=None, help='출력 이미지 경로')
    ap.add_argument('--csv', action='store_true', help='좌표 CSV도 저장')
    a = ap.parse_args()

    path = a.file
    if not path:
        # ⚠ 이름순이 아니라 **수정시각순**으로 고른다 — 이름순이면 'run_TEST_*' 같은
        #   파일이 'run_2026*'보다 뒤로 가서 엉뚱한 걸 집는다(실제로 겪음).
        files = sorted(glob.glob(os.path.join(REC_DIR, 'run_*.jsonl')),
                       key=os.path.getmtime)
        if not files:
            print(f'❌ 기록이 없다: {REC_DIR}')
            return
        path = files[-1]
    if not os.path.exists(path):
        print(f'❌ 파일 없음: {path}'); return

    recs = load(path)
    points, rpath, tie_reqs = build(recs, a.merge)
    if a.min_detect > 1:
        drop = [p for p in points if p[3] < a.min_detect]
        points = [p for p in points if p[3] >= a.min_detect]
        if drop:
            print(f'⚠ 관측 {a.min_detect}회 미만 {len(drop)}점 제외: ' +
                  ', '.join(f'({x:.0f},{y:.0f})' for x, y, _, _ in drop[:8])
                  + (' …' if len(drop) > 8 else ''))
    if not points:
        print('❌ 검출 기록이 없다 (주행 중 검출이 한 번도 안 됐거나 기록이 꺼져 있었다)')
        return

    n_by = defaultdict(int)
    for _, _, c, _ in points:
        n_by[c] += 1
    xs = [p[0] for p in points]; ys = [p[1] for p in points]
    print(f'■ {os.path.basename(path)}')
    print(f'  기록 {len(recs)}줄 → 교차점 **{len(points)}개** (병합 {a.merge:.0f}mm)')
    for c in ('tie', 'untie', 'crossing', None):
        if n_by.get(c):
            print(f'    {LABEL.get(c, c):<18} {n_by[c]:3d}개')
    print(f'  결속 요청 {len(tie_reqs)}점')
    print(f'  범위  X {min(xs):.0f}~{max(xs):.0f}mm ({(max(xs)-min(xs))/1000:.2f}m)'
          f'   Y {min(ys):.0f}~{max(ys):.0f}mm')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for fam in ('NanumGothic', 'Noto Sans CJK KR', 'DejaVu Sans'):
        try:
            matplotlib.rcParams['font.family'] = fam
            break
        except Exception:
            continue
    matplotlib.rcParams['axes.unicode_minus'] = False

    span_x = (max(xs) - min(xs)) / 1000.0 or 1.0
    span_y = (max(ys) - min(ys)) / 1000.0 or 1.0
    fig, ax = plt.subplots(figsize=(max(9, span_x * 4 + 3), max(5, span_y * 4 + 3)))

    # 주행 경로
    if rpath:
        px = [p[0] for p in rpath]
        ax.plot(px, [0] * len(px), '-', color='#bcd', lw=6, alpha=.5,
                zorder=1, label='로봇 주행 경로')

    # 교차점
    for cls in ('crossing', 'untie', 'tie'):
        sel = [(x, y, n) for x, y, c, n in points if c == cls]
        if not sel:
            continue
        ax.scatter([s[0] for s in sel], [s[1] for s in sel],
                   s=[40 + 14 * min(s[2], 6) for s in sel],
                   c=COLOR[cls], edgecolors='white', linewidths=.8,
                   zorder=3, label=f'{LABEL[cls]} ({len(sel)})')

    # 이번 작업에서 결속 요청한 점
    if tie_reqs:
        ax.scatter([t[0] for t in tie_reqs], [t[1] for t in tie_reqs],
                   s=190, facecolors='none', edgecolors='#1a53c0',
                   linewidths=2.0, zorder=4,
                   label=f'이번 작업 결속 요청 ({len(tie_reqs)})')

    ax.set_xlabel('주행 방향 X (mm)')
    ax.set_ylabel('횡방향 Y (mm)')
    ax.set_title(f'철근 배근 작업도  —  {os.path.basename(path)}\n'
                 f'교차점 {len(points)}개 · 결속요청 {len(tie_reqs)}점 · '
                 f'주행 {(max(xs)-min(xs))/1000:.2f}m')
    ax.grid(alpha=.25, linestyle=':')
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.12), ncol=3, frameon=False)
    fig.tight_layout()

    out = a.out or path.replace('.jsonl', '.png')
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'\n  📈 저장: {out}')

    if a.csv:
        cp = path.replace('.jsonl', '.csv')
        with open(cp, 'w') as f:
            f.write('x_mm,y_mm,class,n_detect\n')
            for x, y, c, n in sorted(points):
                f.write(f'{x:.1f},{y:.1f},{c or ""},{n}\n')
        print(f'  📄 저장: {cp}')


if __name__ == '__main__':
    main()
