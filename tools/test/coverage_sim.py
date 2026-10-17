#!/usr/bin/env python3
"""커버리지 지도·플래너 단독 시뮬. **장비가 필요 없다.**

    python3 tools/test/coverage_sim.py                 # 전부
    python3 tools/test/coverage_sim.py --only 3x3      # 하나만
    python3 tools/test/coverage_sim.py -v              # 정차마다 자세히

무엇을 보려는 것인가 — 2차년도 자율주행은 중간이 막히면 그 레인의 나머지를
통째로 버렸다. 지도를 넣었을 때 **막힌 자리를 건너뛰고 나중에 돌아오는지**를
책상에서 확인한다. 설계는 `docs/design/AUTONOMY_COVERAGE_DESIGN.md`.

좌표: 월드 = 작업 좌표 (정준 좌표계, 1번 자세 스테이지 mm). 로봇 포즈도 같은 틀.

시뮬이 흉내 내는 것 / 안 내는 것:
  흉내 냄 — 결속영역(실제 캘리 모델·가동범위)·검출 시야(실제 내부파라미터),
            횡이동 50mm 양자화, 데크 끝, 결속 불가 지점, 주행 오차
  안 냄   — 충돌·검출 실패율·자세 전환 시간·결속 사이클. 그건 실기에서 본다
"""
import argparse
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src', 'rmd_robot_control'))
from rmd_robot_control.coverage_map import (      # noqa: E402
    CoverageMap, Reach, Cost, LAT_MM_PER_TURN, TIED, FAILED, BLOCKED)


class World:
    """진짜 배근. 지도는 이걸 모르고 관측으로만 알아간다."""

    def __init__(self, us, vs, no_tie=(), columns=(), drive_sigma=0.0,
                 seed=1, pad=120.0, obs_range=(2500.0, 1500.0)):
        """no_tie : 묶을 수 없는 교차점 (주행은 된다)
        columns : 크기 있는 장애물 [(u, v, 길이_U, 폭_V)] — 주행도 막는다
        obs_range : 기둥 감지 거리 (전후, 좌우) mm

        ⚠ **기둥과 결속 불가 지점은 보는 센서가 다르다.** 기둥은 전후방
          카메라가 수 미터 앞에서 보고, 결속 불가 지점은 교차점 카메라가
          가까이서야 가린다. 기둥을 교차점 시야로만 감지하게 했더니
          **시야에 들어오기 전에 그 자리로 가는 계획이 만들어져**, 로봇이
          기둥 위에 서 있는 결과가 나왔다 (사용자 지적 2026-10-17).
        """
        self.obs_range = obs_range
        self.us, self.vs = list(us), list(vs)
        self.pts = [(u, v) for v in vs for u in us]
        self.no_tie = set(no_tie)
        self.columns = [tuple(float(x) for x in c) for c in columns]
        self.rng = random.Random(seed)
        self.sigma = drive_sigma
        # 데크 경계 = 배근 외곽 + 여유. 이걸 넘으면 주행불가 (deck_edge 역할)
        self.deck = (min(us) - pad, max(us) + pad, min(vs) - pad, max(vs) + pad)

    def detect(self, ru, rv, fov):
        """그 포즈에서 보이는 교차점 → 정준 좌표(로봇 기준)."""
        return [(u - ru, v - rv) for u, v in self.pts
                if fov[0] <= u - ru <= fov[1] and fov[2] <= v - rv <= fov[3]]

    def edges_seen(self, ru, rv, fov):
        """시야가 데크 밖까지 닿았으면 그 경계를 알려준다 (deck_edge 역할)."""
        out = []
        for axis, lo_i, hi_i, lo, hi in (('u', 0, 1, self.deck[0], self.deck[1]),
                                         ('v', 2, 3, self.deck[2], self.deck[3])):
            r = ru if axis == 'u' else rv
            if r + fov[hi_i] > hi:
                out.append((axis, +1, hi))
            if r + fov[lo_i] < lo:
                out.append((axis, -1, lo))
        return out

    def obstacles_seen(self, ru, rv, fov):
        """그 자리에서 **보이는** 장애물 (로봇 기준). 가 보지 않고 감지된다.

        돌려주는 것은 `(du, dv, size)` — size 가 None 이면 결속 불가 지점,
        값이 있으면 주행도 막는 기둥이다.
        """
        def seen(u, v):
            return (fov[0] <= u - ru <= fov[1] and fov[2] <= v - rv <= fov[3])
        out = [(u - ru, v - rv, None) for u, v in self.no_tie if seen(u, v)]
        of, ol = self.obs_range
        obox = (-of, of, -ol, ol)
        # 기둥은 보통 **일부만** 시야에 들어온다. 보이는 만큼만 등록한다 —
        # 실제로도 지도는 다가가며 조금씩 채워진다.
        for u, v, su, sv in self.columns:
            a0, a1 = u - su / 2 - ru, u + su / 2 - ru
            b0, b1 = v - sv / 2 - rv, v + sv / 2 - rv
            c0, c1 = max(a0, obox[0]), min(a1, obox[1])
            d0, d1 = max(b0, obox[2]), min(b1, obox[3])
            if c1 > c0 and d1 > d0:
                out.append(((c0 + c1) / 2, (d0 + d1) / 2, (c1 - c0, d1 - d0)))
        return out

    def tieable(self, u, v, tol=60.0):
        if any(abs(u - a) < tol and abs(v - b) < tol for a, b in self.no_tie):
            return False
        return not any(abs(u - cu) <= su / 2 and abs(v - cv) <= sv / 2
                       for cu, cv, su, sv in self.columns)

    def drive(self, mm):
        return mm + (self.rng.gauss(0.0, self.sigma) if self.sigma else 0.0)


def run(world, reach, label, start, name_of=None, max_stops=200, verbose=False):
    cm = CoverageMap(reach, cost=Cost())
    cm.ru, cm.rv = start
    fov = cm.fov
    visited, n_tied, n_fail = [], 0, 0
    trace = {'stops': [], 'cells': {}, 'lat': 0, 'drive_mm': 0.0, 'cost_s': 0.0}
    print(f'\n{"="*74}\n■ {label}\n{"="*74}')

    for step in range(max_stops):
        info = cm.observe(world.detect(cm.ru, cm.rv, fov))
        # ★ 장애물은 **가 보기 전에** 보인다 (측면·전방 판정). 지도에 BLOCKED 로
        #   찍히면 그 위에 서겠다는 계획이 아예 안 만들어진다.
        for du, dv, size in world.obstacles_seen(cm.ru, cm.rv, fov):
            cm.see_obstacle(du, dv, size)
        for axis, sign, val in world.edges_seen(cm.ru, cm.rv, fov):
            cm.set_edge(axis, sign, val)

        done = fail = 0
        for c, _pose in cm.covered(cm.ru, cm.rv):
            if world.tieable(c.u, c.v):
                cm.mark(c.key(), TIED); done += 1
            else:
                cm.mark(c.key(), FAILED, '장애물 — 결속 불가'); fail += 1
        n_tied += done; n_fail += fail

        here = name_of(cm.ru, cm.rv) if name_of else f'({cm.ru:+.0f},{cm.rv:+.0f})'
        visited.append(here + ('*' if done else ''))
        trace['stops'].append({'pose': (cm.ru, cm.rv), 'tied': done, 'fail': fail})
        if verbose:
            print(f'  [{step:2d}] {here:>12s}  검출 {info["n"]:2d}(신규 {info.get("new",0):2d})'
                  f'  결속 {done}{" 실패"+str(fail) if fail else ""}'
                  f'  피치 {_f(cm.pitch_u)}×{_f(cm.pitch_v)}'
                  f'  보정 {_f(info.get("shift",(0,0))[0],1)},{_f(info.get("shift",(0,0))[1],1)}')

        nxt = cm.plan_next()
        if nxt is None:
            break
        if verbose:
            tgt = name_of(nxt['ru'], nxt['rv']) if name_of else ''
            print(f'        → {nxt["kind"]:7s} {tgt:>12s} 주행 {nxt["drive_mm"]:+7.0f}mm'
                  f' 횡 {nxt["turns"]:+3d}회({nxt["turns"]*LAT_MM_PER_TURN:+5.0f}mm)'
                  f' 셀 {len(nxt["cells"])}  {nxt["cost_s"]:5.1f}s')
        trace['lat'] += abs(nxt['turns'])
        trace['drive_mm'] += abs(nxt['drive_mm'])
        trace['cost_s'] += nxt['cost_s']
        cm.advance(world.drive(nxt['drive_mm']), nxt['turns'])
    else:
        print('  ⚠ 정차 상한 도달 — 플래너가 수렴하지 않았다')

    bad = [i + 1 for i, s_ in enumerate(trace['stops'])
           if not cm.space.occupiable(*s_['pose'])]
    if bad:
        print(f'  ★★ 설 수 없는 자리에 선 정차지 {bad} — 감지 거리나 계획이 틀렸다')
    cnt = cm.counts()
    print(f'  정차 {len(visited)}회 · 결속 {n_tied}/{len(world.pts)} · 실패 {n_fail}')
    print(f'  방문: {" ".join(visited)}')
    print(f'  상태: {cnt}')
    missed = [c for c in cm.cells.values()
              if c.state not in (TIED, FAILED, BLOCKED)]
    if missed:
        # **왜** 남았는지 가른다. "플래너가 포기" 와 "몸체가 못 들어감" 은 다르다.
        g = {}
        for c in missed:
            g.setdefault(_why_missed(cm, c), []).append(c)
        if g.get('body'):
            print(f'  · 도달 불가 {len(g["body"])}점 — 그 자리에 **몸체가 안 들어간다** (기둥 옆)')
        if g.get('lattice'):
            print(f'  · 격자 위상 {len(g["lattice"])}점 — 설 자리는 있는데 '
                  f'**횡이동 50mm 격자에 안 맞는다**')
        if g.get('plan'):
            print(f'  ★ 미처리 {len(g["plan"])}점 — 갈 수 있는데 계획이 멈췄다: '
                  f'{[c.key() for c in g["plan"]][:8]}')
    trace['cells'] = _state_at(world, cm)
    n_blk = sum(1 for v in trace['cells'].values() if v == BLOCKED)
    trace['summary'] = dict(stops=len(visited), tied=n_tied, total=len(world.pts),
                            fail=n_fail + n_blk, lat=trace['lat'],
                            drive_mm=trace['drive_mm'], cost_s=trace['cost_s'])
    return visited, cm, trace


def _state_at(world, cm, tol=90.0):
    """월드의 진짜 교차점마다 지도가 매긴 최종 상태."""
    out = {}
    for u, v in world.pts:
        best, bd = None, tol
        for c in cm.cells.values():
            d = abs(c.u - u) + abs(c.v - v)
            if d < bd:
                bd, best = d, c
        out[(u, v)] = best.state if best else 'unknown'
    return out


def legacy_run(world, reach, label, start, name_of=None, max_stops=200,
               lookahead=350.0):
    """**2차년도 방식 재현** — 지도 없이 "더 갈 수 있나" 만 보고 움직인다.

    `rebar_drive_node` 의 판정 경로를 그대로 옮겼다:

      · 결속영역 안을 전부 묶는다
      · 다음 거리 = (마지막 결속 열) - (도달범위 시작) + 마진
      · **진행 방향 전방**에 못 가는 것이 보이면 deck_edge 가 STOP →
        `FWD_SETTLE`. 여기서 **막힌 것과 배근 끝을 구별하지 않는다** (핵심 한계)
      · SETTLE 에서 `_enter_lateral('REV_DETECT')` — 레인을 옮기고 **방향을 뒤집는다**
      · 단, `_tick_lateral_judge` 가 **측면 카메라로 옮겨갈 레인을 먼저 본다.**
        못 간다고 하면 `_finish_lateral(ok=False)` → 한 칸도 못 갔으면
        `'횡이동 불가 → 커버리지 종료'` = **DONE**

    마지막 항목이 중요하다 — 옆 레인의 **지금 서 있는 자리**가 막혀 있으면
    2차년도는 레인 전환 자체를 포기하고 작업을 끝낸다.

    ⚠ `lateral_max_lanes`(기본 4)는 **걸지 않는다.** 주행 테스트에서만 쓰던
      설정이라는 사용자 확인(2026-10-17)이 있었다. 설정값 차이로 이기면
      비교가 무의미하므로, 구조적 한계만 남긴다.
    """
    # ⚠ **몸체 제약은 양쪽에 똑같이 건다.** 2차년도에만 빼 두었더니 기둥 위에
    #   서서 묶으며 3차년도를 이겼다(149 vs 140). 알고리즘 차이가 아니라
    #   물리 제약을 한쪽만 안 건 것이다 — 실제 2차년도도 거기엔 못 선다.
    from rmd_robot_control.coverage_space import FreeSpace
    b = reach.bbox()
    space = FreeSpace(center=(0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3])))
    for cu, cv, su, sv in world.columns:
        space.add(cu - su / 2, cu + su / 2, cv - sv / 2, cv + sv / 2)
    ru, rv = start
    sign = +1
    trace = {'stops': [], 'cells': {}, 'lat': 0, 'drive_mm': 0.0, 'cost_s': 0.0}
    tied, lanes = set(), 0
    cost = Cost()
    print(f'\n{"="*74}\n■ {label}\n{"="*74}')
    visited, why_end = [], '정차 상한'

    def band(r_v):
        return [(u, v) for u, v in world.pts if b[2] <= v - r_v <= b[3]]

    for _ in range(max_stops):
        here_ok = space.occupiable(ru, rv)
        inreach = [(u, v) for u, v in band(rv) if b[0] <= u - ru <= b[1]]
        done = 0
        if here_ok:
            for p_ in inreach:
                if p_ not in tied and world.tieable(*p_):
                    tied.add(p_); done += 1
        here = name_of(ru, rv) if name_of else f'({ru:+.0f},{rv:+.0f})'
        visited.append(here + ('*' if done else ''))
        trace['stops'].append({'pose': (ru, rv), 'tied': done, 'fail': 0})

        # ── deck_edge: **진행 방향 전방**만 본다 ─────────────────────────
        fwd_lo = (b[1] if sign > 0 else -lookahead)
        fwd_hi = (b[1] + lookahead if sign > 0 else b[0])
        ahead = [(u, v) for u, v in band(rv) if fwd_lo <= (u - ru) * 1 <= fwd_hi] \
            if sign > 0 else \
            [(u, v) for u, v in band(rv) if b[0] - lookahead <= u - ru <= b[0]]
        stop = any(not world.tieable(*p_) for p_ in ahead)
        cols_ahead = [u for u, _v in band(rv)
                      if (u - ru - (b[1] if sign > 0 else b[0])) * sign > 0]

        # ★ 다음 걸음을 **미리 계산**해서, 거기 설 수 없으면 전방 차단으로 본다.
        #   ⚠ 여기서 `continue` 로 넘기면 같은 자리를 무한히 돈다 (실제로 200회
        #     돌았다). 반드시 `stop` 에 반영해서 레인 전환 분기로 보내야 한다.
        cols = [u - ru for u, _v in inreach] or [b[0] if sign > 0 else b[1]]
        last = max(cols) if sign > 0 else min(cols)
        step = (last - b[0] + 30.0) if sign > 0 else (last - b[1] - 30.0)
        if abs(step) < 1.0:
            step = sign * (b[1] - b[0])
        if not here_ok or not space.occupiable(ru + step, rv):
            stop = True                      # 설 수 없는 자리 = 전방 차단과 같다
        if stop or not cols_ahead:
            lanes += 1
            # ★ 2차년도 규칙: 레인 이동 = **결속한 행 수 × 행 피치**.
            #   ⚠ 피치는 **검출된 전체 행**에서 잰다 (`estimate_pitch` 와 같은
            #     자리). 결속범위 안 행만 보면 1개뿐일 때 피치를 못 재는데,
            #     거기서 상수로 폴백했더니 레인이 400mm 씩 건너뛰며 **한 줄씩
            #     빼먹었다** — 2차년도를 부당하게 깎는 재현이었다.
            rows = sorted({v for _u, v in inreach})
            allv = sorted({v for _u, v in world.pts
                           if abs(v - rv - (b[2] + b[3]) / 2) < 1200})
            d = [y - x for x, y in zip(allv, allv[1:])] or [400.0]
            pv = sorted(d)[len(d) // 2]
            turns = int(round(max(len(rows), 1) * pv / LAT_MM_PER_TURN))
            nrv = rv + turns * LAT_MM_PER_TURN
            # ★ 측면 판정 — 옮겨갈 레인의 **지금 자리**가 가능한가
            side = [(u, v) for u, v in band(nrv) if b[0] <= u - ru <= b[1]]
            if not side:
                why_end = '옆 레인에 배근 없음 → 커버리지 종료'; break
            if (any(not world.tieable(*p_) for p_ in side)
                    or not space.occupiable(ru, nrv)):
                why_end = '측면 판정 불가 → 횡이동 중단, 커버리지 종료'; break
            trace['lat'] += turns
            trace['cost_s'] += cost.move(0, turns)
            rv, sign = nrv, -sign
            continue

        # ★ 2차년도 규칙: **마지막 결속한 열**을 지나칠 거리
        cols = [u - ru for u, _v in inreach] or [b[0] if sign > 0 else b[1]]
        last = max(cols) if sign > 0 else min(cols)
        step = (last - b[0] + 30.0) if sign > 0 else (last - b[1] - 30.0)
        if abs(step) < 1.0:
            step = sign * (b[1] - b[0])
        if not space.occupiable(ru + step, rv):
            stop = True                      # 전방에 못 설 자리 = 더 못 간다
            continue
        trace['drive_mm'] += abs(step)
        trace['cost_s'] += cost.move(step, 0)
        ru += step

    st = {}
    for p_ in world.pts:
        st[p_] = 'tied' if p_ in tied else ('failed' if not world.tieable(*p_) else 'left')
    trace['cells'] = st
    n = sum(1 for v in st.values() if v == 'tied')
    print(f'  정차 {len(visited)}회 · 결속 {n}/{len(world.pts)}'
          f' · 횡이동 {trace["lat"]}회 · 종료사유: {why_end}')
    print(f'  방문: {" ".join(visited)}')
    trace['summary'] = dict(stops=len(visited), tied=n, total=len(world.pts),
                            fail=sum(1 for v in st.values() if v == 'failed'),
                            lat=trace['lat'], drive_mm=trace['drive_mm'],
                            cost_s=trace['cost_s'], why=why_end)
    return visited, trace


def _why_missed(cm, c, step=25.0):
    """그 셀이 왜 안 묶였나. 'lattice' / 'body' / 'plan' 중 하나.

    세 가지를 구별해야 한다 — 전부 "미결속" 으로 뭉뚱그리면 버그인지 한계인지
    알 수 없다:
      body    : 그 셀을 덮는 어떤 자리에도 **몸체가 안 들어간다**
      lattice : 들어가는 자리는 있는데 **횡이동 50mm 격자 위에 없다.**
                rv 는 1회전 단위로만 바뀌므로, 기둥이 가능 구간을 50mm 보다
                좁게 잘라 놓으면 몇 mm 차이로 못 간다
      plan    : 갈 수 있는데 계획이 멈췄다 = **진짜 결함**
    """
    b = cm.reach_bbox
    free = latt = False
    for du in _frange(b[0], b[1], step):
        for dv in _frange(b[2], b[3], step):
            if cm.reach.pose_for(du, dv) is None:
                continue
            ru, rv = c.u - du, c.v - dv
            if not cm.space.occupiable(ru, rv):
                continue
            free = True
            if abs(round((rv - cm.rv) / cm.lat_mm) * cm.lat_mm - (rv - cm.rv)) < 1.0:
                latt = True
    if not free:
        return 'body'
    return 'plan' if latt else 'lattice'


def _frange(a, b, s):
    x = float(a)
    while x <= b:
        yield x
        x += s


def _f(x, nd=0):
    return '—' if x is None else f'{x:.{nd}f}'


# ──────────────────────────────────────────────────────────────────────
# 시나리오
# ──────────────────────────────────────────────────────────────────────

def scn_field(reach, blocked_rect=None):
    """현실 배근 7열 × 5행, 피치 200mm. 한 정차지가 여러 점을 덮는다.

    `blocked_rect` 는 **결속 불가 지점**(주행은 된다). 기둥은 `scn_wall` 쪽이다.
    """
    us = [i * 200.0 for i in range(7)]
    vs = [j * 200.0 for j in range(5)]
    no_tie = []
    if blocked_rect:
        u0, u1, v0, v1 = blocked_rect
        no_tie = [(u, v) for u in us for v in vs
                  if u0 <= u <= u1 and v0 <= v <= v1]
    w = World(us, vs, no_tie=no_tie, drive_sigma=4.0)
    mid = _mid(reach)
    return w, None, (us[0] - mid[0], vs[0] - mid[1])


def scn_wall(reach, where='mid', pitch=200.0, col=300.0):
    """현실 배근(피치 200mm) + **기둥 하나**. 기둥 위치만 바꿔 두 경우를 본다.

      where='end' : 기둥이 레인의 **끝** 쪽 — 2차년도에 가장 덜 불리한 자리
      where='mid' : 기둥이 레인 **중간** — 2차년도의 진짜 약점

    ⚠ 피치는 **200mm** 다 (일반적인 배근 간격, 사용자 지정 2026-10-17).
      전에 쓰던 550mm 짜리 판은 "한 정차지에 교차점 하나" 를 만들어 방문
      순서를 눈으로 보려던 **인위적인 판**이었다. 그 대가로 첫 걸음에서
      피치를 몰라 헛정차가 한 번 생겼는데, 200mm 에서는 첫 시야에 열이
      서너 개 들어와 바로 피치가 잡히므로 그 현상이 없다.
    """
    # ⚠ 판을 작게 잡으면 안 된다. 기둥 300mm 하나가 몸체(1160×940) 때문에
    #   **1.58m × 1.36m 를 못 쓰게 만든다** — 1.6×1.2m 판에서는 그게 거의
    #   전부라 로봇이 둘레밖에 못 돈다. 실제 데크 크기(2.8 × 2.0m)로 잡는다.
    us = [i * pitch for i in range(15)]           # 0 ~ 2800
    vs = [j * pitch for j in range(11)]           # 0 ~ 2000
    cu = (us[-2] if where == 'end' else us[len(us) // 2])
    cols = [(cu, vs[len(vs) // 2], col, col)]
    w = World(us, vs, columns=cols, drive_sigma=3.0)
    mid = _mid(reach)
    return w, None, (us[0] - mid[0], vs[0] - mid[1])


def scn_gap(reach, gap_mm):
    """**기둥 둘 사이로 지나갈 수 있나.** 데크를 가로지르는 기둥 벽에 틈 하나.

    로봇 폭(940mm, 여유 포함 1060mm)보다 틈이 넓으면 통과해서 건너편까지
    묶고, 좁으면 건너편을 포기한다. 임계값 하나로는 답할 수 없는 질문이다.
    """
    PU = PV = 250.0
    us = [c * PU for c in range(8)]
    vs = [r * PV for r in range(5)]
    uw, th = 875.0, 300.0                 # 기둥 벽의 U 위치와 두께
    gc = 500.0                            # 틈 중심 (V)
    lo0, lo1 = -700.0, gc - gap_mm / 2
    hi0, hi1 = gc + gap_mm / 2, 1700.0
    cols = [(uw, (lo0 + lo1) / 2, th, lo1 - lo0),
            (uw, (hi0 + hi1) / 2, th, hi1 - hi0)]
    w = World(us, vs, columns=cols, drive_sigma=3.0)
    mid = _mid(reach)
    return w, None, (us[0] - mid[0], vs[0] - mid[1])


def _mid(reach):
    b = reach.bbox()
    return (0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default='')
    ap.add_argument('-v', '--verbose', action='store_true')
    ap.add_argument('--plot', default='', metavar='DIR',
                    help='그림 저장 폴더 (예: docs/design/figures)')
    a = ap.parse_args()

    reach = Reach.from_files()
    b, f = reach.bbox(), reach.image_fov()
    print('■ 정준 좌표 = 1번 자세 스테이지 mm   (U 주행방향 · V 횡방향)')
    print(f'   결속 가능  U {b[0]:+7.0f} ~ {b[1]:+7.0f} (폭 {b[1]-b[0]:4.0f})'
          f'   V {b[2]:+7.0f} ~ {b[3]:+7.0f} (폭 {b[3]-b[2]:4.0f})')
    print(f'   검출 시야  U {f[0]:+7.0f} ~ {f[1]:+7.0f} (폭 {f[1]-f[0]:4.0f})'
          f'   V {f[2]:+7.0f} ~ {f[3]:+7.0f} (폭 {f[3]-f[2]:4.0f})')
    print(f'   횡이동 양자화 {LAT_MM_PER_TURN:.0f}mm/회전')

    todo = []
    if a.only in ('', 'wall'):
        todo += [('현실 배근 15x11 + 기둥(끝쪽)', scn_wall(reach, 'end')),
                 ('현실 배근 15x11 + 기둥(한가운데)', scn_wall(reach, 'mid'))]
    if a.only in ('', 'field'):
        todo += [('현실 배근 7x5 (피치 200mm)', scn_field(reach)),
                 ('현실 배근 7x5 + 가운데 막힘',
                  scn_field(reach, blocked_rect=(400, 800, 200, 400)))]
    if a.only in ('', 'gap'):
        todo += [('기둥 사이 틈 1300mm — 통과', scn_gap(reach, 1300.0)),
                 ('기둥 사이 틈 850mm — 못 지나감', scn_gap(reach, 850.0))]

    plot = None
    if a.plot:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import coverage_plot as plot

    saved = []
    for label, (w, name_of, start) in todo:
        _v, _cm, tr = run(w, reach, label, start, name_of=name_of, verbose=a.verbose)
        if plot:
            fn = os.path.join(a.plot, 'coverage_' + _slug(label) + '.png')
            saved.append(plot.save_one(w, tr, b, label, fn, note=_note(tr)))

    # 발표용 — 2차년도(지도 없음) vs 3차년도(지도). 기둥 위치 두 가지로.
    #   끝 = 2차년도에 가장 덜 불리한 자리 / 중간 = 진짜 약점. 배근은 200mm.
    if plot:
        for where, tag, sub in (('end', 'blocked', '기둥이 끝쪽'),
                                ('mid', 'midlane', '기둥이 한가운데')):
            w2, nm2, st2 = scn_wall(reach, where)
            _v2, tr2 = legacy_run(w2, reach, f'2차년도 재현 — {sub}', st2, name_of=nm2)
            w3, nm3, st3 = scn_wall(reach, where)
            _v3, _c3, tr3 = run(w3, reach, f'3차년도 — {sub}', st3, name_of=nm3)
            fn = os.path.join(a.plot, f'coverage_compare_{tag}.png')
            saved.append(plot.save_compare(
                w2, tr2, tr3, b,
                (f'2차년도 — 지도 없음 · {sub}',
                 f'3차년도 — 격자 지도 · {sub}'),
                fn, notes=(_note(tr2), _note(tr3))))

    for f in saved:
        print(f'  저장 {f}')


def _slug(s):
    return (s.replace(' ', '_').replace('—', '').replace('(', '').replace(')', '')
             .replace('+', 'plus').replace('__', '_').strip('_'))


def _note(tr):
    s = tr['summary']
    return (f"정차 {s['stops']}회 · 결속 {s['tied']}/{s['total']}"
            + (f" · 장애물 {s['fail']}" if s['fail'] else '')
            + f" · 횡이동 {s['lat']}회({s['lat']*LAT_MM_PER_TURN:.0f}mm)"
              f" · 주행 {s['drive_mm']:.0f}mm · 이동 {s['cost_s']:.0f}s"
            + (f"\n종료: {s['why']}" if s.get('why') else ''))


if __name__ == '__main__':
    main()
