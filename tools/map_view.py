#!/usr/bin/env python3
"""주행이 만든 지도를 본다. **장비가 필요 없다** — 파일만 읽는다.

    python3 tools/map_view.py                 # 가장 최근 주행
    python3 tools/map_view.py ~/rebar_runs/20261106_101920_dry_map.json

반응형 고리의 전제는 "지도는 주행의 **산출물**" 이다. 그래서 `coverage_map` 은
정차마다 `_map.json` 으로 덮어써진다 (estop 으로 끊겨도 마지막 상태가 남는다).
그 산출물을 눈으로 확인하는 것이 이 도구다 — CSV 의 개수만으로는 **어디가**
안 묶였는지 알 수 없다.

    ● 결속  ○ 미결속  · 빈칸(철근 없음)  ✖ 차단  ⊕ 로봇
"""
import glob
import json
import os
import sys

MARK = {'tied': '●', 'pending': '○', 'empty': '·', 'blocked': '✖'}


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else ''
    if not arg:
        hits = sorted(glob.glob(os.path.expanduser('~/rebar_runs/*_map.json')))
        if not hits:
            sys.exit('~/rebar_runs 에 _map.json 이 없다 — 주행을 한 번 돌려야 한다')
        arg = hits[-1]
    d = json.load(open(os.path.expanduser(arg)))
    cells = d['cells']
    if not cells:
        sys.exit(f'{arg} — 셀이 비었다 (검출이 하나도 안 됐다)')

    print(f"\n  {os.path.basename(arg)}")
    pu, pv = d.get('pitch') or (None, None)
    print(f"  자세 u={d['pose'][0]:.0f} v={d['pose'][1]:.0f} · 원점 {d.get('origin')}"
          f" · 피치 U {pu} / V {pv}")
    print(f"  {d['counts']}\n")

    ii = sorted({c['ij'][0] for c in cells})
    jj = sorted({c['ij'][1] for c in cells})
    at = {tuple(c['ij']): c for c in cells}
    # 로봇이 있는 격자 (가장 가까운 셀)
    rp = min(cells, key=lambda c: (c['uv'][0] - d['pose'][0]) ** 2
             + (c['uv'][1] - d['pose'][1]) ** 2)['ij']

    print('       ' + ''.join(f'{i:>3}' for i in ii) + '   ← i (U·주행방향)')
    for j in reversed(jj):                       # V 가 위로 증가
        row = ''
        for i in ii:
            c = at.get((i, j))
            m = ' ' if c is None else MARK.get(c['state'], '?')
            if c is not None and [i, j] == rp:
                m = '⊕'
            row += f'  {m}'
        v = next(c['uv'][1] for c in cells if c['ij'][1] == j)
        print(f'  j={j:<3d}' + row + f'   v≈{v:+.0f}mm')
    print()
    pend = [c for c in cells if c['state'] == 'pending']
    if pend:
        print(f'  미결속 {len(pend)}개 — uv:')
        for c in pend[:12]:
            print(f"    ij={c['ij']}  u={c['uv'][0]:+7.1f}  v={c['uv'][1]:+7.1f}"
                  f"  관측 {c['n']}회" + (f"  ({c['why']})" if c.get('why') else ''))
        if len(pend) > 12:
            print(f'    … 외 {len(pend) - 12}개')
    else:
        print('  미결속 없음 — 본 곳은 모두 묶였다')
    print('\n  ● 결속  ○ 미결속  · 빈칸  ✖ 차단  ⊕ 로봇\n')


if __name__ == '__main__':
    main()
