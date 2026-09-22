#!/usr/bin/env python3
"""
상부 작업부 Top-View(XY) 기구학 시뮬레이터 (이산 yaw 자세 모델)

top view(XY)에서 결속포인트를 클릭하면, **이산 yaw 자세 5개**
(max_limit / 9시 / 12시 / 3시 / min_limit) 각각에 대해 IK를 풀어
도달 가능 + 충돌 안전한 자세를 찾고, 유효 자세로 이동 애니메이션을 보여준다.

핵심
  - yaw는 연속값이 아니라 이산 자세 중 하나 → stage IK가 유일해.
  - 총구는 yaw축에서 230mm(top-view 수평) 뻗은 막대, 그 끝이 결속포인트(TCP).
    yaw를 돌리면 TCP가 230mm 반경으로 쓸고 지나감 → 자세가 stage 위치를 바꿈.
  - 자세마다 충돌 안전영역 다름 → 여러 자세 union = 작업영역 확대.

치수
  - mover중심 → yaw축: (+185, 0) mm (고정)
  - yaw축 → 결속포인트(TCP): 수평 230mm(green bar) / 수직 z 390mm(작업깊이, Z축)

좌표계 (kinematics.pdf p8): +x=우, +y=하(전진), yaw=z회전.  헤딩 0°=+x, +90°=+y(하)
base frame 원점 = xMin·yMin 코너 (0,0)

조작: 클릭=목표 / 1=max강제 2=min강제 / a=자동 / r=리셋 / q=종료
실행: python3 scripts/sim/workpart_topview_sim.py   (또는 --selftest 헤드리스)

⚠️ 충돌 안전박스 = 실측 단면(analysis §4) 근사. max/min 헤딩은 미확정(아래 표시).
   근거: docs/design/workpart_workspace_analysis.md
"""
import argparse
import json
import math
import os
import sys

# ======================= CONFIG =======================
X_MIN_92 = 1818.69      # 0x92 @ xMin(home)
KX = 4.497              # deg/mm (X)
Y_MIN_92 = -989.36      # 0x92 @ yMin(home)
KY = 4.462              # deg/mm (Y)
STAGE_X_MAX_MM = 410.0
STAGE_Y_MAX_MM = 295.0

MOVER_TO_YAW = (185.0, 0.0)    # mover중심 → yaw축 (고정)
# 결속포인트(TCP) = yaw축과 동일점 (총구는 z로 390mm 내려가 top-view 투영이 yaw축).
# 230mm 녹색 막대 = 총구 "몸체" 수평길이 — 회전하며 프레임에 충돌하는 부분(결속점 아님).
GUN_BODY_LEN = 230.0           # 총구 몸체 top-view 수평 길이(mm, 실측) — 충돌 관련

# yaw 헤딩 환산 (analysis §3) — max/min 외삽용 fallback
YAW92_AT_3OCLOCK = -304.0
HEAD_SLOPE = -0.652            # heading[+x기준] = HEAD_SLOPE*(yaw92 - YAW92_AT_3OCLOCK)


def _head(yaw92):
    return HEAD_SLOPE * (yaw92 - YAW92_AT_3OCLOCK)


# 이산 yaw 자세 2개. head(deg): plot +x=0°, +y=+90°(하).
#   시계 기준 = 유저(12시=plot -x). 그 시계로 6시=+x, → 7시=+30°, 5시=-30°.
#   max=7시(우하향, +30°), min=5시(우상향, -30°) — 실측방향(2026-05-29)
YAW_POSES = [
    dict(name='max',  yaw92=36.1,   head=30.0,  approx_head=False),  # 7시(우하)
    dict(name='min',  yaw92=-360.0, head=-30.0, approx_head=False),  # 5시(우상)
]

FRAME = dict(x0=-60.0, x1=660.0, y0=-60.0, y1=360.0)  # 배경용(대략). 충돌은 파란 작업영역으로 판정
ANIM_FRAMES = 45
DATA_JSON = 'data/calibration/workpart_enc.json'
# ======================================================


def x_to_mm(v92):
    return (X_MIN_92 - v92) / KX


def y_to_mm(v92):
    return (v92 - Y_MIN_92) / KY


def dir_vec(head_deg):
    h = math.radians(head_deg)
    return math.cos(h), math.sin(h)


def fk(x_stage, y_stage, head_deg):
    """stage(mm) + 총구헤딩 → (yaw축=결속점, 손잡이끝) 좌표(mm).
    결속포인트(TCP) = yaw축.  손잡이끝 = yaw축 + 230mm*dir(head) (충돌 관련, 결속점 아님)."""
    yaw_axis = (x_stage + MOVER_TO_YAW[0], y_stage + MOVER_TO_YAW[1])
    dx, dy = dir_vec(head_deg)
    handle = (yaw_axis[0] + GUN_BODY_LEN * dx, yaw_axis[1] + GUN_BODY_LEN * dy)
    return yaw_axis, handle


def _in_workbox(px, py):
    """파란 작업영역(stage 가용역) 안인가."""
    return 0.0 <= px <= STAGE_X_MAX_MM and 0.0 <= py <= STAGE_Y_MAX_MM


def ik_for_pose(tx, ty, pose):
    """결속포인트(tx,ty)=yaw축 → (x_stage,y_stage, reachable, safe).
    yaw축 = 결속점이므로 stage는 헤딩 무관:  mover = (tx,ty) - (185,0).
    제약: 230mm 손잡이가 파란 작업영역(0~410 × 0~295)을 넘지 않아야 safe."""
    x_stage = tx - MOVER_TO_YAW[0]
    y_stage = ty - MOVER_TO_YAW[1]
    reachable = (0.0 <= x_stage <= STAGE_X_MAX_MM
                 and 0.0 <= y_stage <= STAGE_Y_MAX_MM)
    dx, dy = dir_vec(pose['head'])
    hx, hy = tx + GUN_BODY_LEN * dx, ty + GUN_BODY_LEN * dy   # 손잡이 끝
    safe = _in_workbox(tx, ty) and _in_workbox(hx, hy)        # 손잡이 ⊂ 파란박스
    return x_stage, y_stage, reachable, safe


def evaluate_poses(tx, ty):
    out = []
    for pose in YAW_POSES:
        xs, ys, reach, safe = ik_for_pose(tx, ty, pose)
        out.append(dict(pose=pose, x=xs, y=ys, reachable=reach,
                        safe=safe, valid=(reach and safe)))
    return out


def load_measured(path):
    if not os.path.exists(path):
        return []
    data = json.load(open(path))
    pts = []
    for s in data:
        def deg(ax):
            return s.get(f'{ax}_0x92_deg', s.get(f'{ax}_deg'))
        if deg('x') is None or deg('y') is None:
            continue
        pts.append(dict(idx=s.get('index'), x_mm=x_to_mm(deg('x')),
                        y_mm=y_to_mm(deg('y')), yaw92=deg('yaw')))
    return pts


# ----------------------------- 헤드리스 검증 -----------------------------
def selftest():
    print("=== 이산 yaw 자세 (head: +x=0,+y=+90 / 손잡이=%.0fmm) ===" % GUN_BODY_LEN)
    for p in YAW_POSES:
        flag = ' ⚠외삽' if p['approx_head'] else ''
        print(f"  {p['name']:5s} yaw92={p['yaw92']:7.1f} head={p['head']:7.1f}°{flag}")
    print("\n=== 클릭점 평가 (TCP=결속포인트) ===")
    for tx, ty in [(300, 60), (300, 250), (250, 130), (450, 150)]:
        evs = evaluate_poses(tx, ty)
        for e in evs:
            tag = 'valid' if e['valid'] else ('reach' if e['reachable'] else 'X')
            print(f"  TCP=({tx},{ty}) {e['pose']['name']:5s}: "
                  f"stage=({e['x']:6.1f},{e['y']:6.1f}) {tag}")
        print()
    pts = load_measured(DATA_JSON)
    print(f"측정 샘플 {len(pts)}개")
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 6))
        _draw_static(ax, pts)
        _draw_pose(ax, 0, 0, YAW_POSES[-1]['head'])
        out = 'data/calibration/workpart_topview_preview.png'
        fig.savefig(out, dpi=110, bbox_inches='tight')
        print(f"미리보기 저장: {out}")
    except Exception as e:
        print(f"(PNG 생략: {e})")
    return 0


# ----------------------------- 그리기 -----------------------------
_dyn = []


def _draw_static(ax, pts):
    import matplotlib.patches as mpatches
    ax.add_patch(mpatches.Rectangle(
        (FRAME['x0'], FRAME['y0']), FRAME['x1'] - FRAME['x0'],
        FRAME['y1'] - FRAME['y0'], fill=False, edgecolor='dimgray', lw=2))
    ax.add_patch(mpatches.Rectangle(
        (0, 0), STAGE_X_MAX_MM, STAGE_Y_MAX_MM, fill=True,
        facecolor='#eaf4ff', edgecolor='steelblue', lw=1.2, alpha=0.5))
    for p in pts:
        ax.plot(p['x_mm'], p['y_mm'], 'x', color='seagreen', ms=7, mew=2)
        ax.annotate(f"#{p['idx']}", (p['x_mm'], p['y_mm']),
                    textcoords='offset points', xytext=(4, 3), fontsize=7)
    ax.set_xlabel('+x (mm) ->')
    ax.set_ylabel('+y (mm) down')
    ax.set_xlim(FRAME['x0'] - 20, FRAME['x1'] + 20)
    ax.set_ylim(FRAME['y0'] - 20, FRAME['y1'] + 20)
    ax.invert_yaxis()
    ax.set_aspect('equal')
    ax.grid(True, ls=':', alpha=0.5)


def _draw_pose(ax, x_stage, y_stage, head_deg, target=None, ok=True):
    global _dyn
    for a in _dyn:
        a.remove()
    _dyn = []
    yaw_axis, handle = fk(x_stage, y_stage, head_deg)
    # 손잡이가 파란 작업영역 벗어나면 빨강(충돌), 안이면 teal
    handle_ok = _in_workbox(*yaw_axis) and _in_workbox(*handle)
    # Y축 레일(mover의 x를 따라다님) — mover 뒤에 먼저(점선)
    a0, = ax.plot([x_stage, x_stage], [0.0, STAGE_Y_MAX_MM],
                  '--', color='slateblue', lw=1.3, alpha=0.7, zorder=1)
    a1, = ax.plot(x_stage, y_stage, 's', color='navy', ms=10, zorder=5)  # mover
    a2, = ax.plot([x_stage, yaw_axis[0]], [y_stage, yaw_axis[1]],
                  '-', color='gray', lw=2)                              # mover→yaw
    a4, = ax.plot([yaw_axis[0], handle[0]], [yaw_axis[1], handle[1]], '-',
                  color=('teal' if handle_ok else 'red'),
                  lw=4, solid_capstyle='round')                         # 손잡이 230mm
    a5, = ax.plot(*yaw_axis, '*', color='red', ms=15)                  # 결속점=yaw축
    _dyn += [a0, a1, a2, a4, a5]
    if target is not None:
        a6, = ax.plot(*target, '+', color=('green' if ok else 'red'), ms=16, mew=3)
        _dyn.append(a6)


# ----------------------------- 인터랙티브 -----------------------------
def run_interactive():
    import matplotlib.pyplot as plt

    pts = load_measured(DATA_JSON)
    fig, ax = plt.subplots(figsize=(11, 7.5))
    _draw_static(ax, pts)

    st = dict(x=0.0, y=0.0, head=YAW_POSES[-1]['head'],
              force=None, target=None, evs=None, chosen=None, path=None, frame=0)

    def pick(evs, force):
        if force is not None and evs[force]['valid']:
            return evs[force]
        for e in evs:
            if e['valid']:
                return e
        return None

    def redraw():
        _draw_pose(ax, st['x'], st['y'], st['head'], st['target'], ok=bool(st['chosen']))
        if st['evs'] is not None:
            valid = ','.join(e['pose']['name'] for e in st['evs'] if e['valid'])
            ch = st['chosen']['pose']['name'] if st['chosen'] else 'none'
            t = st['target']
            ax.set_title(f"target={t and (round(t[0]),round(t[1]))}  "
                         f"valid=[{valid or '-'}]  chosen={ch}")
        fig.canvas.draw_idle()

    redraw()
    timer = fig.canvas.new_timer(interval=20)

    def on_tick():
        if st['path'] is None:
            timer.stop(); return
        if st['frame'] >= len(st['path']):
            st['path'] = None; timer.stop(); return
        st['x'], st['y'], st['head'] = st['path'][st['frame']]
        st['frame'] += 1
        redraw()

    timer.add_callback(on_tick)

    def go(chosen):
        st['chosen'] = chosen
        if chosen is None:
            redraw(); return
        x0, y0, h0 = st['x'], st['y'], st['head']
        xs, ys, h1 = chosen['x'], chosen['y'], chosen['pose']['head']
        st['path'] = [(x0 + (xs - x0) * k / ANIM_FRAMES,
                       y0 + (ys - y0) * k / ANIM_FRAMES,
                       h0 + (h1 - h0) * k / ANIM_FRAMES)
                      for k in range(1, ANIM_FRAMES + 1)]
        st['frame'] = 0
        timer.start()

    def on_click(event):
        if event.inaxes != ax or event.xdata is None:
            return
        st['target'] = (event.xdata, event.ydata)
        st['evs'] = evaluate_poses(*st['target'])
        chosen = pick(st['evs'], st['force'])
        valid = [e['pose']['name'] for e in st['evs'] if e['valid']]
        print(f"클릭({st['target'][0]:.0f},{st['target'][1]:.0f}) valid={valid} "
              f"→ {chosen['pose']['name'] if chosen else '도달/안전 자세 없음'}")
        go(chosen)

    def rotate_in_place(h1):
        """stage 고정, 헤딩만 h1으로 회전 (제자리 미리보기)."""
        x0, y0, h0 = st['x'], st['y'], st['head']
        st['chosen'] = None
        st['path'] = [(x0, y0, h0 + (h1 - h0) * k / ANIM_FRAMES)
                      for k in range(1, ANIM_FRAMES + 1)]
        st['frame'] = 0
        timer.start()

    def on_key(event):
        if event.key == 'q':
            plt.close(fig); return
        if event.key.isdigit() and 1 <= int(event.key) <= len(YAW_POSES):
            st['force'] = int(event.key) - 1
            if st['target'] is not None and st['evs'] is not None:
                go(pick(st['evs'], st['force']))   # 목표 있으면 이동+회전
            else:
                rotate_in_place(YAW_POSES[st['force']]['head'])  # 없으면 제자리 회전
            return
        elif event.key == 'a':
            st['force'] = None
        elif event.key == 'r':
            st.update(x=0.0, y=0.0, head=YAW_POSES[-1]['head'],
                      target=None, evs=None, chosen=None, path=None)
            redraw(); return
        if st['target'] is not None and st['evs'] is not None:
            go(pick(st['evs'], st['force']))

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    print("클릭=목표 / 1=max강제 2=min강제 / a=자동 / r=리셋 / q=종료")
    plt.show()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    try:
        return run_interactive()
    except Exception as e:
        print(f"인터랙티브 실패({e}). 디스플레이 없으면 --selftest.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
