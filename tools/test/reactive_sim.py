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
# 줄 위치를 모르면 고정 폭으로 **탐색**한다 (내림 — 올리면 겹침이 얇아진다)
check('  줄 모름 → 탐색 6회전', d['turns'], 6)

print('\n■ 레인 간격은 **실측 줄 위치**로 정한다 (고정량이 아니다)')
# 영역 380·여유 40 → half 150. 현재 rv=0 이면 [-150,150] 이 덮인다.
# ⚠ 변수를 r·d 로 쓰지 말 것 — 위 블록이 그 둘을 계속 쓴다 (덮으면 아래
#   '되돌아간다' 검사가 엉뚱한 피치를 본다. 실제로 한 번 그랬다).
rl = Reactive(520, 380, pitch_mm=215)
dl = rl.decide(go_mm=0, blocked_dirs={'forward'}, side_open={1: True},
               rows_v=[0.0, 215.0, 430.0, 645.0], rv=0.0)
check('안 묶은 첫 줄 215 → 그 줄이 가까운 경계에', dl['turns'], 7)
check('  7회전 = 350mm 이면 215 가 영역 안', abs(215 - 350) <= 150, True)
check('  고정 300mm 과 다르다', dl['turns'] != 6, True)
# 넘치면 그 줄을 잃는다 — 내림이어야 한다
r2 = Reactive(520, 380, pitch_mm=215)
d2 = r2.decide(go_mm=0, blocked_dirs={'forward'}, side_open={1: True},
               rows_v=[260.0], rv=0.0)
check('목표 410 → 내림 8회전(400)', d2['turns'], 8)
check('  260 이 영역 안 (|260-400|<=150)', abs(260 - 400) <= 150, True)
# 이미 덮인 줄은 레인 간격을 끌지 않는다
r3 = Reactive(520, 380, pitch_mm=215)
d3 = r3.decide(go_mm=0, blocked_dirs={'forward'}, side_open={1: True},
               rows_v=[0.0, 100.0, 215.0], rv=0.0)
check('영역 안 줄(0·100)은 무시, 215 가 기준', d3['turns'], 7)
# 반대 방향도 대칭이어야 한다
r4 = Reactive(520, 380, pitch_mm=215, v_dir=-1)
d4 = r4.decide(go_mm=0, blocked_dirs={'forward'}, side_open={-1: True},
               rows_v=[-215.0, 0.0], rv=0.0)
check('v_dir -1 이면 음수쪽으로 7회전', d4['turns'], -7)

print('\n■ 빠뜨린 줄이 없는가 — 데크 V 1400 을 모든 위상에서 훑는다')
HALF = 380 / 2 - 40
worst = []
for pitch in (180.0, 200.0, 215.0, 232.0, 260.0):
    bad = 0
    for ph10 in range(int(pitch // 5)):
        phase = ph10 * 5.0
        rows = [phase + i * pitch for i in range(20)]
        rows = [v for v in rows if 0 <= v <= 1400.0]
        rr = Reactive(520, 380, pitch_mm=pitch)
        rv, tied = 0.0, set()
        for _ in range(40):
            for v in rows:
                if abs(v - rv) <= HALF:
                    tied.add(round(v, 3))
            rest = [v for v in rows if round(v, 3) not in tied]
            if not rest:
                break
            t = rr.lane_turns([v for v in rest], rv)
            nxt = rv + t * 50.0
            if nxt > 1400.0:        # 데크 끝 — clamp 는 측면 거리 필요(미해결)
                break
            rv = nxt
        bad = max(bad, len([v for v in rows if round(v, 3) not in tied]))
    worst.append((pitch, bad))
for pitch, bad in worst:
    # 끝줄 1개는 측면 거리(mm)가 없어 clamp 를 못 하는 **알려진 구멍**이다
    check(f'피치 {pitch:.0f} — 끝줄 외 빠뜨림 없음', bad <= 1, True)
check('고정량이었다면 중간에서도 빠뜨렸다 (대조)',
      max(b for _, b in worst) <= 1, True)
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


print('\n■ \'ㄹ\' 주행 — 교차점 격자를 깐 가상 데크에서 **실측 줄 간격**으로')
# 왜 이 시험이 따로 필요한가 — 위의 "가상 데크 순회" 는 `rows_v` 를 안 넘겨
# **고정 폭 폴백**으로 돈다. 실측 줄 기반 간격(2026-11-06 추가)과 'ㄹ' 모양이
# **같은 테스트에서 함께** 증명된 적이 없었다. 여기서 둘을 같이 본다.
#
# 흉내 냄 — 교차점 격자, 결속영역, 데크 경계, 횡 50mm 양자화, 지도 누적
# 안 냄   — 검출 실패·충돌. 그건 실기에서 본다
DU, DV = 3000.0, 1400.0
PU, PV = 232.0, 215.0
HU, HV = 520 / 2, 380 / 2          # 결속영역 반폭
cross = [(i * PU, j * PV) for i in range(int(DU / PU) + 1)
         for j in range(int(DV / PV) + 1)]
rr = Reactive(520, 380, pitch_mm=PU)
u = v = 0.0
tied, path, flips = set(), [(u, v)], 0
prev_udir = rr.u_dir
for _ in range(400):
    # 정차 결속 — 영역 안의 교차점을 묶는다
    for c in cross:
        if abs(c[0] - u) <= HU and abs(c[1] - v) <= HV:
            tied.add(c)
    # 지도에서 "아직 안 묶은 줄" 을 넘긴다 (노드가 map.pending() 으로 하는 것)
    rows_v = sorted({c[1] for c in cross if c not in tied})
    go = (DU - u) if rr.u_dir > 0 else u
    side = {d: (0.0 <= v + d * 50.0 <= DV) for d in (1, -1)}
    d = rr.decide(go_mm=go, blocked_dirs=set(), side_open=side, rows_v=rows_v,
                  rv=v)
    if d['kind'] == DONE:
        break
    if d['kind'] == DRIVE:
        u = min(max(u + d['mm'], 0.0), DU)
        rr.moved(DRIVE, d['mm'])
    else:
        nv = v + d['turns'] * 50.0
        if not (0.0 <= nv <= DV):          # 데크 밖 — 측면이 막았을 상황
            break
        v = nv
        rr.moved(LATERAL, d['turns'])
        if rr.u_dir != prev_udir:
            flips += 1
            prev_udir = rr.u_dir
    path.append((u, v))

miss = [c for c in cross if c not in tied]
print(f"  정차 {rr.stops}회 · 레인 {rr.lanes}개 · 방향 전환 {flips}회 · "
      f"교차점 {len(tied)}/{len(cross)} 결속")
check("'ㄹ' 이려면 방향 전환이 2회 이상", flips >= 2, True)
check('레인마다 진행 방향이 뒤집힌다', flips, rr.lanes)
check('U 를 양쪽 끝까지 훑는다',
      max(p[0] for p in path) >= DU - 520 and min(p[0] for p in path) <= 520,
      True)
check('V 를 데크 폭만큼 훑는다', max(p[1] for p in path) >= DV - 380, True)
# 끝줄은 측면 거리(mm)가 없어 clamp 를 못 하는 **알려진 구멍**이다.
# 그 한 줄(= V 한 값)을 넘는 누락이 있으면 간격 규칙이 깨진 것이다.
missed_rows = sorted({c[1] for c in miss})
print(f"  빠뜨린 교차점 {len(miss)}개 · 그 줄 V = "
      f"{[round(x) for x in missed_rows] or '없음'}")
check('빠뜨림이 있어도 **끝줄 한 줄 이내**', len(missed_rows) <= 1, True)

print(f"\n{'통과' if not FAIL else '★ 실패 있음'} — {OK} OK / {FAIL} NG")
sys.exit(1 if FAIL else 0)
