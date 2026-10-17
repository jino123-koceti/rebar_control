#!/usr/bin/env python3
"""**RL 정책 vs 탐욕 플래너** — 같은 판에서 비교한다. 장비가 필요 없다.

    python3 tools/test/coverage_vs_rl.py

왜 필요한가: RL 모델(8차, `model/cnn_ppo_agent_best.pth`)을 붙일지 말지는
"더 나은가" 로 정해야 한다. 둘은 **같은 일(칸 방문 순서)** 을 하므로 바로 비교된다.

공정하게 맞춘 것:
  · 같은 격자·같은 장애물·같은 시작점 (0,0)
  · 같은 칸 크기 (U 550mm · V 400mm) 와 같은 비용 모델(`Cost`)
  · RL 의 한 칸씩 이동을 **실제 로봇 동작으로 환산**한다 — 묶을 필요 없는 칸을
    지나갈 때는 멈추지 않고 이어서 간다. 안 그러면 RL 에 불리하게 센다.

맞추지 못한 것(해석할 때 감안):
  · RL 은 작업영역·장애물을 **미리 다 안다.** 탐욕 쪽은 돌아다니며 알아간다.
    즉 이 비교는 RL 에 유리한 조건이다.
"""
import argparse
import os
import sys

R = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
sys.path.insert(0, os.path.join(R, 'src', 'rmd_robot_control'))
sys.path.insert(0, os.path.join(R, 'path_generation', 'reinforcement_learning'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rmd_robot_control.coverage_map import Cost, Reach, LAT_MM_PER_TURN  # noqa: E402
import coverage_sim as S                                                  # noqa: E402

# 칸 = **정차지** 하나. 결속영역(520 × 380mm)보다 작아야 칸 사이에 안 닿는
# 띠가 안 생긴다. V 는 횡이동 1회전(50mm)의 배수여야 한다.
PU, PV = 450.0, 350.0          # 350 = 7회전
MODEL = os.path.join(R, 'model', 'cnn_ppo_agent_best.pth')

GRIDS = [
    ('3x3', 3, 3, []),
    ('3x3 +장애물1', 3, 3, [(2, 1)]),
    ('4x5', 4, 5, []),
    ('4x5 +장애물1', 4, 5, [(2, 1)]),
    ('5x8', 5, 8, []),
    ('5x8 +장애물2', 5, 8, [(1, 2), (4, 3)]),
    ('6x10 +장애물3', 6, 10, [(2, 1), (5, 3), (8, 4)]),
]


def rl_route(rows, cols, obstacles):
    """정책을 끝까지 굴려 **칸 방문 순서**를 얻는다. [(col,row), ...]"""
    from agents_cnn import CNNPPOAgent
    from environment_cnn import RebarGridCNNEnv
    ag = CNNPPOAgent()
    ag.load(MODEL)
    env = RebarGridCNNEnv()
    obs, _ = env.reset(grid_rows=rows, grid_cols=cols,
                       obstacles=list(obstacles), start_pos=(0, 0))
    done = False
    info = {}
    while not done:
        a = ag.predict(obs, env.get_aux_input())
        obs, _r, term, trunc, info = env.step(a)
        done = term or trunc
    return env.extract_path(), info.get('completion_rate', 0.0)


def cost_of_route(path, obstacles, cost):
    """칸 경로 → 실제 로봇 동작 비용.

    **묶을 칸에서만 선다.** 이미 방문한 칸을 지나갈 때는 멈추지 않고 같은
    방향으로 이어 간다 — 그래야 실제 주행과 맞다.
    """
    obs = set(obstacles)
    seen, stops, lat, drv, t = set(), 0, 0, 0.0, 0.0
    seg_axis, seg_n = None, 0

    def flush():
        nonlocal seg_axis, seg_n, lat, drv, t
        if not seg_n:
            return
        if seg_axis == 'x':
            d = seg_n * PU
            drv += d
            t += cost.move(d, 0)
        else:
            n = int(round(seg_n * PV / LAT_MM_PER_TURN))
            lat += n
            t += cost.move(0, n)
        seg_axis, seg_n = None, 0

    for i, cell in enumerate(path):
        if i:
            prev = path[i - 1]
            ax = 'x' if cell[0] != prev[0] else 'y'
            if seg_axis and ax != seg_axis:
                flush()
            seg_axis, seg_n = ax, seg_n + 1
        tie = cell not in seen and cell not in obs
        seen.add(cell)
        if tie:
            flush()
            stops += 1
    flush()
    return dict(tied=len(seen - obs), stops=stops, lat=lat, drive_mm=drv, cost_s=t)


def greedy_route(rows, cols, obstacles, reach):
    """같은 판을 탐욕 플래너로. `coverage_sim` 의 World/run 을 재사용한다."""
    us = [c * PU for c in range(cols)]
    vs = [r * PV for r in range(rows)]
    no_tie = [(c * PU, r * PV) for c, r in obstacles]
    w = S.World(us, vs, no_tie=no_tie)
    mid = S._mid(reach)
    start = (us[0] - mid[0], vs[0] - mid[1])
    import contextlib
    import io as _io
    with contextlib.redirect_stdout(_io.StringIO()):
        _v, _cm, tr = S.run(w, reach, '', start)
    s = tr['summary']
    return dict(tied=s['tied'], stops=s['stops'], lat=s['lat'],
                drive_mm=s['drive_mm'], cost_s=s['cost_s'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default='')
    a = ap.parse_args()
    reach, cost = Reach.from_files(), Cost()
    print(f'칸 크기 U {PU:.0f}mm · V {PV:.0f}mm (= 횡이동 {PV/LAT_MM_PER_TURN:.0f}회전)')
    print(f'{"판":<14s} {"":>4s} {"묶음":>7s} {"정차":>5s} {"횡회전":>6s} '
          f'{"주행mm":>8s} {"이동s":>7s}')
    tot = {}
    for name, rows, cols, obs in GRIDS:
        if a.only and a.only not in name:
            continue
        path, cr = rl_route(rows, cols, obs)
        r = cost_of_route(path, obs, cost)
        g = greedy_route(rows, cols, obs, reach)
        n_valid = rows * cols - len(obs)
        for tag, d in (('RL', r), ('탐욕', g)):
            print(f'{name if tag == "RL" else "":<14s} {tag:>4s} '
                  f'{d["tied"]:3d}/{n_valid:<3d} {d["stops"]:5d} {d["lat"]:6d} '
                  f'{d["drive_mm"]:8.0f} {d["cost_s"]:7.0f}'
                  + (f'   CR {cr:.3f}' if tag == 'RL' and cr < 1.0 else ''))
            tot.setdefault(tag, []).append(d)
    print()
    for tag, ds in tot.items():
        print(f'  {tag:>4s} 합계  정차 {sum(d["stops"] for d in ds):3d} · '
              f'횡회전 {sum(d["lat"] for d in ds):3d} · '
              f'이동 {sum(d["cost_s"] for d in ds):5.0f}s')


if __name__ == '__main__':
    main()
