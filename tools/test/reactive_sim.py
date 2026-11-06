#!/usr/bin/env python3
"""반응형 순회 단독 시뮬. **장비가 필요 없다.**

    python3 tools/test/reactive_sim.py

무엇을 보려는 것인가 — 반응형 고리는 지도를 안 보고 **센서만** 보고 돈다.
그러면 "정말 데크를 다 덮나", "막혔을 때 제대로 비키나", "지그재그가
저절로 나오나" 를 책상에서 확인해야 한다. 설계는
`docs/design/AUTONOMY_COVERAGE_DESIGN.md`.

시뮬이 흉내 내는 것 / 안 내는 것:
  흉내 냄 — 데크 경계(전후·좌우), 결속영역, 횡 50mm 양자화, 주행 오차
  안 냄   — 검출 실패·충돌·결속 사이클. 그건 실기에서 본다
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src', 'rmd_robot_control'))
from rmd_robot_control.reactive import (        # noqa: E402
    Reactive, step_for, pitch_from, DRIVE, LATERAL, DONE)

OK = FAIL = 0


def check(name, got, want, tol=None):
    global OK, FAIL
    good = (abs(got - want) <= tol) if tol is not None else (got == want)
    print(f"  {'OK ' if good else '★NG'} {name:44s} {got!s:>14s}  (기대 {want})")
    if good:
        OK += 1
    else:
        FAIL += 1


print('■ 피치 추정 — 2026-11-05 실측 교차점 (스테이지 mm)')
PTS = [(-158.1, -232.4), (-152.1, -13.3), (-147.8, 189.1),
       (72.3, -234.0), (80.2, -16.7), (84.8, 190.4),
       (306.6, -457.2), (312.2, -236.1), (318.5, -19.3), (321.1, 188.0)]
check('X 피치', round(pitch_from(PTS, 0)), 232, 6)
check('Y 피치', round(pitch_from(PTS, 1)), 215, 10)
check('점이 2개뿐이면 None', pitch_from(PTS[:2], 0), None)
check('한 줄만 있으면 None', pitch_from([(0, 0), (0.5, 0), (1, 0)], 0), None)

print('\n■ 스텝 = 결속영역에 들어가는 피치의 정수배 (영역 U 520mm)')
check('피치 225 → 450 (2칸)', step_for(225, 520), 450.0)
check('피치 232 → 464 (2칸)', step_for(232, 520), 464.0)
check('피치 300 → 300 (1칸)', step_for(300, 520), 300.0)
check('피치 None → 기본 225 ×2', step_for(None, 520), 450.0)
check('피치 10(깨짐) → 기본값으로', step_for(10, 520), 450.0)

print('\n■ 막히지 않으면 계속 전진한다')
r = Reactive(520, 380, pitch_mm=225)
d = r.decide(go_mm=None, blocked_dirs=set(), side_open={})
check('전진', d['kind'], DRIVE)
check('  거리', d['mm'], 450.0)
d = r.decide(go_mm=2000, blocked_dirs=set(), side_open={})
check('여유 충분하면 그대로', d['mm'], 450.0)

print('\n■ 막히면 그때만 비킨다 — 그리고 진행 방향이 뒤집힌다')
r = Reactive(520, 380, pitch_mm=225)
d = r.decide(go_mm=200, blocked_dirs=set(), side_open={1: True, -1: True})
check('남은 200 < 스텝 450 → 횡', d['kind'], LATERAL)
# 내림이다 — 올리면 겹침이 의도(15%)보다 얇아진다 (30mm vs 80mm)
check('  레인 폭 380×0.85 → 6회전(내림)', d['turns'], 6)
r.moved(LATERAL, d['turns'])
check('  진행 방향 뒤집힘', r.u_dir, -1)
d = r.decide(go_mm=None, blocked_dirs=set(), side_open={})
check('되돌아간다 (음수 전진)', d['mm'], -450.0)

print('\n■ deck_edge 가 막아도 같다')
r = Reactive(520, 380, pitch_mm=225)
d = r.decide(go_mm=None, blocked_dirs={'forward'}, side_open={1: True})
check('forward 차단 → 횡', d['kind'], LATERAL)
r2 = Reactive(520, 380, pitch_mm=225, u_dir=-1)
d = r2.decide(go_mm=None, blocked_dirs={'forward'}, side_open={1: True})
check('후진 중이면 forward 차단은 무시', d['kind'], DRIVE)
d = r2.decide(go_mm=None, blocked_dirs={'backward'}, side_open={1: True})
check('후진 중 backward 차단 → 횡', d['kind'], LATERAL)

print('\n■ 횡 방향은 한 번 정하면 유지 — 왕복하지 않는다')
r = Reactive(520, 380, pitch_mm=225)
d = r.decide(go_mm=0, blocked_dirs=set(), side_open={1: True, -1: True})
r.moved(LATERAL, d['turns'])
check('첫 횡 이후 잠긴다', r.v_locked, True)
d = r.decide(go_mm=0, blocked_dirs=set(), side_open={1: False, -1: True})
check('그 방향 막히면 완료 (반대로 안 간다)', d['kind'], DONE)

print('\n■ 첫 횡 전에는 반대쪽도 고른다')
r = Reactive(520, 380, pitch_mm=225, v_dir=1)
d = r.decide(go_mm=0, blocked_dirs={'forward'}, side_open={1: False, -1: True})
check('우측 막힘 → 좌측으로', d['turns'] < 0, True)
r = Reactive(520, 380, pitch_mm=225)
d = r.decide(go_mm=0, blocked_dirs={'forward'}, side_open={1: False, -1: False})
check('양쪽 다 막힘 → 완료', d['kind'], DONE)

print('\n■ 가상 데크 순회 — 데크 3000(U) × 1400(V), 결속영역 520×380')
DECK_U, DECK_V = 3000.0, 1400.0
r = Reactive(520, 380, pitch_mm=225)
u = v = 0.0                      # 로봇 위치 (데크 좌하단 기준)
covered = []
for _ in range(200):
    go = (DECK_U - u) if r.u_dir > 0 else u          # 그 방향 남은 거리
    side = {1: v + 300 <= DECK_V, -1: v - 300 >= 0}
    d = r.decide(go_mm=go, blocked_dirs=set(), side_open=side)
    if d['kind'] == DONE:
        break
    if d['kind'] == DRIVE:
        u += d['mm']; r.moved(DRIVE, d['mm'])
        covered.append((u, v))
    else:
        v += d['turns'] * 50.0; r.moved(LATERAL, d['turns'])
print(f"  정차 {r.stops}회 · 레인 {r.lanes}개 · 최종 (u={u:.0f}, v={v:.0f})")
us = [c[0] for c in covered]; vs = sorted({c[1] for c in covered})
check('레인 수 = 1400/300 + 1', r.lanes + 1, 5)
check('횡 방향이 고정됐다', r.v_locked, True)
check('U 를 끝까지 훑는다', max(us) >= DECK_U - 450, True)
check('U 시작점까지 되돌아온다', min(us) <= 450, True)
check('레인 간격 = 6회전 × 50mm', vs[1] - vs[0] if len(vs) > 1 else 0, 300.0)
check('정차 간격 = 스텝', round(sorted(us)[1] - sorted(us)[0]) in (0, 450), True)

print(f"\n{'통과' if not FAIL else '★ 실패 있음'} — {OK} OK / {FAIL} NG")
sys.exit(1 if FAIL else 0)
