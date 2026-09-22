#!/usr/bin/env python3
"""워크샵 발표자료(14장) 그림 생성.

    python3 docs/presentations/make_figures.py          # 전부
    python3 docs/presentations/make_figures.py s07 s08  # 일부

출력: docs/presentations/figures/sNN_*.png
원본: data/rebar_map/*.jsonl, data/logs/auto_tying/*.log, docs/reports/*.md 의 실측치.
수치를 코드에 박아둔 것(100점 시험, 캘리브 오차 등)은 옆에 출처를 적었다.
"""
import glob
import json
import math
import os
import re
import shutil
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
import numpy as np

WS = os.path.expanduser('~/ros2_ws')
OUT = os.path.join(WS, 'docs/presentations/figures')
os.makedirs(OUT, exist_ok=True)

# ── 공통 스타일 ────────────────────────────────────────────────
for p in ('/usr/share/fonts/truetype/nanum/NanumSquareR.ttf',
          '/usr/share/fonts/truetype/nanum/NanumSquareB.ttf'):
    if os.path.exists(p):
        font_manager.fontManager.addfont(p)
plt.rcParams.update({
    'font.family': 'NanumSquare', 'axes.unicode_minus': False,
    'font.size': 13, 'axes.titlesize': 16, 'axes.titleweight': 'bold',
    'axes.labelsize': 13, 'axes.spines.top': False, 'axes.spines.right': False,
    'axes.grid': True, 'grid.alpha': 0.25, 'figure.dpi': 100,
    'savefig.dpi': 200, 'savefig.bbox': 'tight', 'savefig.facecolor': 'white',
})
INK, MUTED = '#1f2937', '#6b7280'
BLUE, ORANGE, GREEN, RED, PURPLE, GRAY = (
    '#2563eb', '#ea580c', '#16a34a', '#dc2626', '#7c3aed', '#9ca3af')
POSE_C = {'r': GREEN, 'l': RED}
POSE_N = {'r': '우측 자세', 'l': '좌측 자세'}

# 도달범위 (rebar_drive_node / tying_orchestrator.yaml 실측)
TIE_X = (0.0, 345.0)
TIE_Y = {'r': (0.0, 142.0), 'l': (124.0, 288.0)}
# 자세 정규화: 같은 교차점도 자세별 오프셋만큼 좌표가 다르다 (l-r = -13.0, +20.6)
POSE_NORM = {'r': (0.0, 0.0), 'l': (13.0, -20.6)}


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path)
    plt.close(fig)
    print('  →', os.path.relpath(path, WS))


def load(run):
    return [json.loads(l) for l in open(
        os.path.join(WS, 'data/rebar_map', run + '.jsonl')) if l.strip()]


def box(ax, x, y, w, h, text, fc, ec=None, fs=12, tc=INK, bold=False, r=0.02):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f'round,pad=0,rounding_size={r}',
                                fc=fc, ec=ec or fc, lw=1.5))
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=fs,
            color=tc, fontweight='bold' if bold else 'normal', linespacing=1.4)


def arrow(ax, p0, p1, color=MUTED, lw=1.8, style='-|>', rad=0.0, ls='-'):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=16,
                                 color=color, lw=lw, ls=ls,
                                 connectionstyle=f'arc3,rad={rad}'))


def blank(w, h):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis('off')
    return fig, ax


# ══ S03 로봇 구성 — 소프트웨어 노드 구성 ══════════════════════════
def s03():
    fig, ax = blank(14, 7.2)
    ax.set_title('소프트웨어 구성 — 인지 · 판단 · 실행', loc='left', pad=24)
    col = {'sense': '#dbeafe', 'plan': '#ede9fe', 'act': '#dcfce7', 'hw': '#f3f4f6'}
    # 센서
    box(ax, 0.00, 0.74, 0.19, 0.15, 'Orbbec Gemini2L\n(상부 · 교차점)', col['sense'])
    box(ax, 0.00, 0.50, 0.19, 0.15, 'ZED X Mini ×2\n(전 · 후방 주행)', col['sense'])
    box(ax, 0.00, 0.26, 0.19, 0.15, 'ZED X One ×2\n(좌 · 우 레인판정)', col['sense'])
    box(ax, 0.00, 0.02, 0.19, 0.15, '범퍼 · 리미트 DIO\n주행 엔코더', col['hw'])
    # 인지
    box(ax, 0.26, 0.74, 0.21, 0.15, '교차점 검출·위치식별\nYOLO + CAD 변환', col['sense'], bold=True)
    box(ax, 0.26, 0.40, 0.21, 0.22, '주행가능 판정\ndeck_edge\n(seg · rebar_frac ·\nheading · 레벨봉)', col['sense'], bold=True)
    # 판단
    box(ax, 0.54, 0.46, 0.21, 0.30, '자율작업 상태머신\nrebar_drive\n\n· 격자 피치 → 스텝거리\n· 결속대상 선별\n· 레인 전환', col['plan'], bold=True)
    box(ax, 0.54, 0.10, 0.21, 0.22, '결속 오케스트레이터\ntying_orchestrator\n(자세 · XY · Z · 트리거)', col['plan'], bold=True)
    # 실행
    box(ax, 0.82, 0.62, 0.18, 0.15, 'drive_controller\n(최후 안전차단)', col['act'])
    box(ax, 0.82, 0.40, 0.18, 0.15, 'joint_controller\n(횡이동 0x143)', col['act'])
    box(ax, 0.82, 0.10, 0.18, 0.22, '결속부 4축\nX · Y · Z · Yaw\n+ 결속기 트리거', col['act'])
    for y in (0.815, 0.575, 0.335):
        pass
    arrow(ax, (0.19, 0.815), (0.26, 0.815))
    arrow(ax, (0.19, 0.575), (0.26, 0.54))
    arrow(ax, (0.19, 0.335), (0.26, 0.46))
    arrow(ax, (0.47, 0.80), (0.54, 0.70))
    ax.text(0.505, 0.79, '교차점', fontsize=10, color=MUTED, ha='center')
    arrow(ax, (0.47, 0.51), (0.54, 0.56))
    ax.text(0.505, 0.47, 'GO/SLOW/STOP', fontsize=10, color=MUTED, ha='center')
    arrow(ax, (0.645, 0.46), (0.645, 0.32))
    ax.text(0.655, 0.39, '결속 요청 ↓\n완료 ↑', fontsize=10, color=MUTED)
    arrow(ax, (0.75, 0.66), (0.82, 0.69))
    ax.text(0.785, 0.70, '/cmd_vel', fontsize=10, color=MUTED, ha='center')
    arrow(ax, (0.75, 0.52), (0.82, 0.475))
    arrow(ax, (0.75, 0.21), (0.82, 0.21))
    # 안전 차단은 상태머신을 거치지 않고 모터 직전으로 직접 들어간다
    ax.plot([0.365, 0.365, 0.785, 0.785], [0.40, 0.355, 0.355, 0.665], color=RED, ls='--', lw=1.6)
    arrow(ax, (0.785, 0.665), (0.82, 0.665), color=RED)
    ax.text(0.50, 0.325, '방향별 차단 — 상태머신 우회, 모터 직전으로', fontsize=10.5,
            color=RED, ha='center')
    ax.plot([0.19, 0.80], [0.095, 0.095], color=RED, ls=':', lw=1.4)
    ax.plot([0.80, 0.80], [0.095, 0.02], color=RED, ls=':', lw=0)
    ax.text(0.50, 0.035, '범퍼 → drive_controller 방향별 차단 · 엔코더 → 스텝 폐루프',
            fontsize=10, color=RED, ha='center')
    for x, t in ((0.095, '센서'), (0.365, '인지'), (0.645, '판단'), (0.91, '실행')):
        ax.text(x, 0.955, t, ha='center', fontsize=13, fontweight='bold', color=MUTED)
    save(fig, 's03_software_architecture.png')


# ══ S04 전체 흐름 — 상태머신 ═══════════════════════════════════
def s04():
    fig, ax = blank(14, 6.2)
    ax.set_title('자율작업 상태머신 — 지도 없이, 보이는 배근을 따라간다', loc='left', pad=12)
    D, S, L = '#dbeafe', '#fef3c7', '#ede9fe'
    box(ax, 0.00, 0.44, 0.09, 0.12, 'IDLE', '#f3f4f6', bold=True)
    box(ax, 0.15, 0.64, 0.16, 0.16, '전진 검출\n+ 결속', D, bold=True)
    box(ax, 0.40, 0.64, 0.16, 0.16, '전진 스텝\n(엔코더 폐루프)', D, bold=True)
    box(ax, 0.65, 0.64, 0.13, 0.16, '배근 끝\n정지', S, bold=True)
    box(ax, 0.84, 0.40, 0.16, 0.20, '레인 전환\n70mm × N회\n(회마다 측면판정)', L, bold=True)
    box(ax, 0.65, 0.18, 0.16, 0.16, '후진 검출\n+ 결속', D, bold=True)
    box(ax, 0.40, 0.18, 0.16, 0.16, '후진 스텝', D, bold=True)
    box(ax, 0.15, 0.18, 0.16, 0.16, '배근 끝\n정지', S, bold=True)
    arrow(ax, (0.09, 0.53), (0.15, 0.70)); ax.text(0.085, 0.64, 'S23', fontsize=10, color=MUTED)
    arrow(ax, (0.31, 0.745), (0.40, 0.745)); arrow(ax, (0.40, 0.69), (0.31, 0.69))
    ax.text(0.355, 0.765, '거리 d', fontsize=10, color=MUTED, ha='center')
    ax.text(0.355, 0.645, '도착', fontsize=10, color=MUTED, ha='center')
    arrow(ax, (0.56, 0.72), (0.65, 0.72)); ax.text(0.605, 0.74, 'STOP×3', fontsize=10, color=RED, ha='center')
    arrow(ax, (0.78, 0.72), (0.88, 0.60), rad=-0.2)
    arrow(ax, (0.88, 0.40), (0.81, 0.30), rad=-0.2)
    arrow(ax, (0.65, 0.29), (0.56, 0.29)); arrow(ax, (0.56, 0.23), (0.65, 0.23))
    ax.text(0.605, 0.305, '거리 d', fontsize=10, color=MUTED, ha='center')
    ax.text(0.605, 0.19, '도착', fontsize=10, color=MUTED, ha='center')
    arrow(ax, (0.40, 0.26), (0.31, 0.26), color=RED)
    ax.text(0.355, 0.28, 'STOP×3', fontsize=10, color=RED, ha='center')
    arrow(ax, (0.23, 0.34), (0.23, 0.64), color=PURPLE, ls=':')
    ax.text(0.24, 0.47, "레인 전환 후 다시 전진\n('ㄹ'자, 레인 상한까지)", fontsize=10, color=PURPLE)
    # 한 사이클 설명
    ax.text(0.00, 0.03, '한 사이클:  ① 교차점 검출(YOLO + depth)  →  ② 격자 피치 측정  →  '
            '③ 도달범위 안 점만 결속  →  ④ "마지막 결속 열 + 마진"만큼 이동',
            fontsize=12, color=INK)
    ax.text(0.00, 0.93, '전 구간 감시: 주행가능 판정 · 범퍼 · 판정 노후화 워치독  |  '
            '중단: S24 · E-stop · S20 해제', fontsize=11, color=MUTED)
    save(fig, 's04_state_machine.png')


# ══ S05 교차점 검출 ════════════════════════════════════════════
def s05():
    run, fn = 'run_20260922_120018', 's0_fwd_r_46027.jpg'
    rec = next(r for r in load(run) if r['type'] == 'detect' and r.get('frame') == fn)
    img = cv2.cvtColor(cv2.imread(os.path.join(WS, 'data/rebar_map/frames', run, fn)),
                       cv2.COLOR_BGR2RGB)
    tie = {(round(x, 1), round(y, 1)) for v in rec['tie_req'].values() for x, y in v}
    fig = plt.figure(figsize=(15, 6.2))
    ax = fig.add_axes([0.0, 0.0, 0.60, 0.92])
    ax.imshow(img); ax.axis('off')
    ax.set_title(f'Orbbec 검출 결과 — 교차점 {len(rec["det"])}개 · 결속 대상 {len(tie)}개',
                 loc='left')
    for x, y, cls, conf, u, v in rec['det']:
        inreach = TIE_X[0] <= x <= TIE_X[1]
        c = GREEN if inreach else '#facc15'
        ax.add_patch(plt.Circle((u, v), 22, fill=False, ec=c, lw=3))
        ax.text(u + 28, v - 10, f'{conf:.2f}', color='white', fontsize=11,
                bbox=dict(fc=c, ec='none', pad=1.5, alpha=0.9))
    ax.text(15, 785, '●초록 = 스테이지 도달범위 안   ●노랑 = 범위 밖(피치 계산에만 사용)',
            color='white', fontsize=12, bbox=dict(fc='black', alpha=0.55, ec='none'))
    # 재학습 효과 (rc_domain_retraining, 2026-07-22)
    bx = fig.add_axes([0.68, 0.14, 0.30, 0.66])
    vals = [0.336, 0.895]
    bars = bx.bar(['재학습 전', '테스트베드 도메인\n재학습 후'], vals, color=[GRAY, BLUE], width=0.55)
    for b, v in zip(bars, vals):
        bx.text(b.get_x() + b.get_width() / 2, v + 0.02, f'{v:.3f}', ha='center',
                fontsize=15, fontweight='bold')
    bx.set_ylim(0, 1.05); bx.set_ylabel('평균 검출 신뢰도')
    bx.set_title('RC 테스트베드 도메인 재학습', fontsize=14)
    bx.text(0.5, -0.24, 'mAP50 = 0.991', transform=bx.transAxes, ha='center',
            fontsize=13, color=INK)
    bx.grid(axis='x', visible=False)
    save(fig, 's05_detection.png')


# ══ S06 위치식별 — 파이프라인 + 정확도 비교 ══════════════════════
def s06():
    fig, ax = blank(15, 3.2)
    steps = [('(u, v)\n+ 정합 depth', '#f3f4f6'), ('P_cam\n카메라 3D', '#dbeafe'),
             ('P_eef\n결속건 프레임', '#ede9fe'), ('스테이지 XY\n지령 (mm)', '#dcfce7')]
    ops = ['intrinsic\n역투영', 'CAD 강체변환\nR, t (고정)', '자세별 오프셋\n(r / l, 실측 2개)']
    w, gap = 0.16, 0.12
    for i, (t, c) in enumerate(steps):
        x = i * (w + gap)
        box(ax, x, 0.25, w, 0.5, t, c, fs=14, bold=True)
        if i < 3:
            arrow(ax, (x + w + 0.005, 0.5), (x + w + gap - 0.005, 0.5), color=INK, lw=2.2)
            ax.text(x + w + gap / 2, 0.82, ops[i], ha='center', va='bottom', fontsize=12,
                    color=PURPLE if i == 1 else MUTED)
    ax.text(0, 0.02, '실측으로 맞출 자유도 = 자세별 XY 오프셋 2쌍뿐. '
            '카메라를 옮겨도 CAD 배치값만 바꾸면 된다.', fontsize=12, color=MUTED)
    save(fig, 's06_pipeline.png')

    # 정확도 (MEMORY Vision Calibration / orbbec_cad_transform.py 헤더 4점 실측)
    fig, ax = plt.subplots(figsize=(7.5, 5))
    names = ['회귀 · 우측\n(78쌍)', '회귀 · 좌측\n(24쌍)', 'CAD 강체변환\n(4점 검증)']
    vals = [6.72, 5.67, 2.0]
    bars = ax.bar(names, vals, color=[GRAY, GRAY, PURPLE], width=0.55)
    for b, v, s in zip(bars, vals, ['6.7', '5.7', '~2']):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.15, f'{s} mm', ha='center',
                fontsize=15, fontweight='bold')
    ax.set_ylabel('평균 위치 오차 (mm)'); ax.set_ylim(0, 8)
    ax.set_title('위치식별 정확도', loc='left'); ax.grid(axis='x', visible=False)
    ax.text(0.99, 0.93, 'CAD 방식: 스케일 ≈ 1.00', transform=ax.transAxes,
            ha='right', color=MUTED, fontsize=11)
    save(fig, 's06_accuracy.png')
    src = os.path.join(WS, 'docs/reports/figures/fig1a_cad_overlay.png')
    shutil.copy(src, os.path.join(OUT, 's06_cad_overlay.png'))
    print('  → (복사) s06_cad_overlay.png')


# ══ S07 결속 시퀀스 — 자세별 도달범위 + 실제 결속점 ═══════════════
def s07():
    pts = {'r': [], 'l': []}
    for run in ('run_20260922_120018', 'run_20260922_122725', 'run_20260922_111322'):
        for r in load(run):
            if r['type'] == 'tie_done':
                for p, v in r['points'].items():
                    pts[p] += v
    fig, ax = plt.subplots(figsize=(9, 6.5))
    for p, a in (('r', 0.15), ('l', 0.15)):
        y0, y1 = TIE_Y[p]
        ax.add_patch(Rectangle((TIE_X[0], y0), TIE_X[1] - TIE_X[0], y1 - y0,
                               fc=POSE_C[p], alpha=a, ec=POSE_C[p], lw=2))
        ax.text(TIE_X[1] - 6, (y0 + y1) / 2 + (-40 if p == 'r' else 40),
                f'{POSE_N[p]}\nY {y0:.0f}~{y1:.0f}mm', ha='right', va='center',
                color=POSE_C[p], fontsize=13, fontweight='bold')
        a_ = np.array(pts[p])
        if len(a_):
            ax.scatter(a_[:, 0], a_[:, 1], s=70, color=POSE_C[p], ec='white', lw=1.2,
                       zorder=3, label=f'{POSE_N[p]} 실제 결속점 ({len(a_)})')
    ax.axhspan(124, 142, color=PURPLE, alpha=0.18)
    ax.text(8, 133, '겹침 124~142 → 현재 자세 우선 (자세변경 최소화)', va='center',
            fontsize=10.5, color=PURPLE)
    ax.set_xlim(-20, 380); ax.set_ylim(-20, 320)
    ax.set_xlabel('스테이지 X (mm, 주행방향)'); ax.set_ylabel('스테이지 Y (mm)')
    ax.set_title('Yaw 399° 로 두 자세 → 한 정차에서 Y 288mm 커버', loc='left')
    ax.legend(loc='upper left', fontsize=11, framealpha=0.95)
    ax.text(0.99, -0.14, '결속점: 2026-09-22 자율결속 3회 기록', transform=ax.transAxes,
            ha='right', fontsize=10, color=MUTED)
    save(fig, 's07_reach_ranges.png')


# ══ S08 격자 피치 → 스텝 거리 ══════════════════════════════════
def s08():
    sys.path.insert(0, os.path.join(WS, 'src/rebar_vision'))
    from rebar_vision.coverage_planner import compute_next_move
    rec = next(r for r in load('run_20260922_120018')
               if r['type'] == 'detect' and r.get('frame') == 's0_fwd_r_46027.jpg')
    P = np.array([[d[0], d[1]] for d in rec['det']])
    res = compute_next_move(P.tolist(), tie_x_range=TIE_X, tie_y_range=(0.0, 288.0))
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(16, 6), gridspec_kw={'width_ratios': [1.35, 1]})
    ax.add_patch(Rectangle((TIE_X[0], 0), TIE_X[1], 288, fc=BLUE, alpha=0.08, ec=BLUE, lw=1.5))
    ax.text(5, 300, '스테이지 도달범위 (X 0~345)', color=BLUE, fontsize=11)
    ax.scatter(P[:, 0], P[:, 1], s=90, color=INK, zorder=3, label='검출된 교차점 (전부 → 피치)')
    for cx in res['cols_x']:
        ax.axvline(cx, color=GREEN, lw=2, alpha=0.7)
    last = res['last_col_x']; d = res['next_forward_mm']
    ax.annotate('', xy=(d, -45), xytext=(TIE_X[0], -45),
                arrowprops=dict(arrowstyle='<->', color=ORANGE, lw=2.2))
    ax.text(d / 2, -62, f'다음 이동 d = {d:.0f} mm', ha='center', color=ORANGE,
            fontsize=13, fontweight='bold', va='top')
    ax.axvline(last + 30, color=ORANGE, ls=':', lw=1.3)
    ax.axvline(d, color=ORANGE, ls='--', lw=1.8)
    ax.text(d + 4, 120, f'd = max(마지막 결속 열 + 30,\n          1 피치)\n'
            f'   = max({last + 30:.0f}, {res["pitch_x"]:.0f})', color=ORANGE, fontsize=10.5)
    ax.text(-350, 330, f'pitch_x = {res["pitch_x"]:.0f} mm  ·  '
            f'pitch_y = {res["pitch_y"]:.0f} mm  (결측 열 자동 보정)', fontsize=12)
    ax.set_xlim(-360, 520); ax.set_ylim(-110, 360)
    ax.set_xlabel('로봇 X (mm, + = 전진방향)'); ax.set_ylabel('로봇 Y (mm)')
    ax.set_title('실측 한 정차 (2026-09-22 스텝 0)', loc='left')
    ax.legend(loc='lower left', fontsize=11)

    # 실패 사례 (compute_next_move docstring, 2026-08-06 실측)
    c1, c2, p = 154.7, 311.6, 156.9
    bx.set_title("초기 설계  d = 열수 × 피치  의 실패", loc='left')
    for i, (lab, dd, col) in enumerate([('지령 2 × 156.9', 313.8, GRAY),
                                        ('실제 이동 (도달오차 -14)', 300.0, RED),
                                        ('현재: 마지막열 + 마진', c2 + 30, GREEN)]):
        y = 2 - i
        bx.barh(y, dd, color=col, height=0.5)
        bx.text(dd - 4, y, f'{dd:.1f}', va='center', ha='right', fontsize=13,
                fontweight='bold', color='white')
        bx.text(4, y + 0.38, lab, fontsize=11.5, color=INK)
    bx.axvline(c2, color=INK, lw=2)
    bx.text(c2 - 3, -0.75, f'유효 하한 {c2}\n(이 열을 지나쳐야 함)', ha='right', fontsize=11)
    bx.set_xlim(0, 400); bx.set_ylim(-1.0, 2.8); bx.set_yticks([])
    bx.set_xlabel('이동거리 (mm)'); bx.grid(axis='y', visible=False)
    bx.text(0.0, -0.22, '→ 311.6 열이 11.6mm 앞에 남아 재결속. 여유 2.2mm뿐이었다.',
            transform=bx.transAxes, fontsize=12, color=RED)
    fig.tight_layout()
    save(fig, 's08_pitch_step.png')


# ══ S09 주행가능 판정 — 기존 그림 + 실주행 rebar_frac 흐름 ═════════
def s09():
    shutil.copy(os.path.join(WS, 'docs/reports/figures/fig4_deck_edge.png'),
                os.path.join(OUT, 's09_deck_edge.png'))
    print('  → (복사) s09_deck_edge.png')
    # 슬라이드에 크게 넣으려고 전방(STOP)·후방(GO) 줄을 따로 자른다
    from PIL import Image
    im = Image.open(os.path.join(OUT, 's09_deck_edge.png'))
    for tag, box_ in (('front_stop', (235, 118, 1945, 712)), ('back_go', (235, 722, 1945, 1318))):
        im.crop(box_).save(os.path.join(OUT, f's09_deck_edge_{tag}.png'))
        print(f'  → (자름) s09_deck_edge_{tag}.png')
    # 실제 VerdictFSM 을 4Hz 신호에 돌려 단순 임계와 비교 (신호는 합성, 판정 코드는 실물)
    sys.path.insert(0, os.path.join(WS, 'src/rebar_vision'))
    from rebar_vision.deck_edge import VerdictFSM
    t = np.arange(0, 12, 0.25)
    rng = np.random.default_rng(7)
    frac = (0.62 - 0.18 / (1 + np.exp(-(t - 6.5) * 1.6)) + 0.035 * np.sin(t * 3.1)
            + rng.normal(0, 0.02, t.size))
    naive = np.where(frac < 0.45, 'STOP', np.where(frac < 0.55, 'SLOW', 'GO'))
    fsm = VerdictFSM()
    smart = [fsm.step(float(f))[0] for f in frac]
    CC = {'GO': GREEN, 'SLOW': ORANGE, 'STOP': RED}
    flips = lambda v: sum(1 for i in range(1, len(v)) if v[i] != v[i - 1])
    fig, (ax, bx) = plt.subplots(2, 1, figsize=(11, 5.6), sharex=True,
                                 gridspec_kw={'height_ratios': [2.3, 1]})
    ax.plot(t, frac, color=INK, lw=2, marker='o', ms=3)
    for yv, lab, c in ((0.45, 'STOP 0.45', RED), (0.55, 'SLOW 0.55', ORANGE)):
        ax.axhline(yv, color=c, ls='--', lw=1.5); ax.text(0.05, yv + 0.006, lab, color=c, fontsize=11)
    ax.set_ylabel('rebar_frac'); ax.set_ylim(0.33, 0.72)
    ax.set_title('판정은 떨린다 → 정지는 즉시, 해제는 보수적으로', loc='left')
    for row, (name, v) in enumerate([(f'단순 임계 — 전환 {flips(naive)}회', naive),
                                     (f'VerdictFSM — 전환 {flips(smart)}회', smart)]):
        for ti, vi in zip(t, v):
            bx.add_patch(Rectangle((ti, 1 - row - 0.4), 0.25, 0.8, color=CC[vi], lw=0))
        bx.text(-0.15, 1 - row, name, ha='right', va='center', fontsize=12)
    bx.set_ylim(-0.6, 1.6); bx.set_yticks([]); bx.grid(False)
    bx.set_xlabel('시간 (s, 판정 4 Hz)')
    bx.text(12, -2.3, '신호는 합성, 판정은 deck_edge.VerdictFSM 실물 코드 (중앙값 5프레임 + STOP 해제는 0.55 초과 시만)',
            ha='right', fontsize=10, color=MUTED)
    fig.tight_layout()
    save(fig, 's09_hysteresis.png')
    print(f'     (단순 {flips(naive)}회 / FSM {flips(smart)}회)')


# ══ S10 조향 + 장애물 ═════════════════════════════════════════
def s10():
    shutil.copy(os.path.join(WS, 'docs/reports/figures/heading_response_2026-09-02.png'),
                os.path.join(OUT, 's10_heading_response.png'))
    print('  → (복사) s10_heading_response.png')
    # 레벨봉 오검출 방어: 지속성 필터(4프레임 중 3) — 수치는 autotying_bugfix_2026-09-15
    fig, ax = plt.subplots(figsize=(10, 4))
    n = 18
    spacer = np.zeros(n); spacer[[3, 6]] = 0.62            # 스페이서: 9프레임 중 1장꼴 튐
    rod = np.zeros(n); rod[9:] = [0.49, 0.50, 0.49, 0.51, 0.50, 0.49, 0.50, 0.50, 0.49]
    x = np.arange(n)
    ax.bar(x - 0.2, spacer, 0.4, color=GRAY, label='스페이서 (오검출, 간헐)')
    ax.bar(x + 0.2, rod, 0.4, color='#eab308', label='레벨봉 (진짜, 연속)')
    fire = [i for i in range(n) if (rod[max(0, i - 3):i + 1] > 0.3).sum() >= 3]
    ax.scatter(fire, [0.72] * len(fire), marker='v', s=90, color=RED, zorder=3,
               label='정지 발동 (최근 4프레임 중 3)')
    ax.set_xlabel('프레임 (4 Hz)'); ax.set_ylabel('레벨봉 면적비')
    ax.set_xticks(range(0, n, 2))
    ax.text(0, -0.25, '개념도 — 면적비는 실측 대표값 (진짜 봉 0.49~0.51 연속, 오검출은 9프레임 중 1장꼴)',
            transform=ax.transAxes, fontsize=10, color=MUTED)
    ax.set_ylim(0, 0.8); ax.legend(loc='upper left', fontsize=11)
    ax.set_title('실루엣으로 못 가르는 스페이서 → "지속성"으로 가른다', loc='left')
    ax.text(n - 0.5, 0.76, '대가: 최악 4프레임 지연 (0.1 m/s면 100 mm)', ha='right',
            fontsize=10.5, color=MUTED)
    save(fig, 's10_levelrod_persistence.png')
    for f in ('102501_front_final.jpg',):   # 125101은 케이블 오검출 장면이라 제외
        src = os.path.join(WS, 'data/levelrod', f)
        if os.path.exists(src):
            shutil.copy(src, os.path.join(OUT, 's10_levelrod_' + f))
            print('  → (복사) s10_levelrod_' + f)


# ══ S11 중복 결속 — 이력 메모리 재생 ════════════════════════════
def s11():
    run = 'run_20260819_153645'
    recs = [r for r in load(run) if r['type'] == 'detect']
    req = []                                   # (gx, gy, step, dir)
    for r in recs:
        ox = r.get('odom_x_mm') or 0.0
        for p, v in r['tie_req'].items():
            nx, ny = POSE_NORM[p]
            for x, y in v:
                req.append((ox + x + nx, y + ny, r['step'], r['dir']))
    kept, dup = [], []
    for g in req:
        if any((g[0] - k[0]) ** 2 + (g[1] - k[1]) ** 2 <= 70 ** 2 for k in kept):
            dup.append(g)
        else:
            kept.append(g)
    fig, ax = plt.subplots(figsize=(12, 5))
    for k in kept:
        ax.add_patch(plt.Circle((k[0], k[1]), 70, fc=GREEN, alpha=0.10, ec=GREEN, lw=0.8, ls=':'))
    for g, is_dup in [(g, False) for g in kept] + [(g, True) for g in dup]:
        mk = '>' if g[3] == 'fwd' else '<'
        if is_dup:
            ax.scatter(g[0], g[1], s=420, facecolor='none', ec=RED, lw=2.5, zorder=4)
        ax.scatter(g[0], g[1], s=110, marker=mk, color=GREEN if not is_dup else RED,
                   ec='white', lw=1, zorder=5)
    ax.scatter([], [], marker='>', s=110, color=GREEN, label=f'결속 (전진 ▶ / 후진 ◀) — 고유 {len(kept)}지점')
    ax.scatter([], [], marker='o', s=260, facecolor='none', ec=RED, lw=2.5,
               label=f'중복 요청 → 이력 메모리가 제외 ({len(dup)})')
    ax.scatter([], [], marker='o', s=260, facecolor=GREEN, alpha=0.15, ec=GREEN, ls=':',
               label='기억 반경 70mm (= 피치의 1/3)')
    pct = 100 * len(dup) / max(1, len(req))
    ax.set_title(f'결속 요청 {len(req)}점 중 {len(dup)}점({pct:.0f}%)이 이미 묶은 자리'
                 f' — 전부 후진 구간', loc='left')
    ax.set_xlabel('전역 X (mm, 주행 엔코더 + 검출 좌표, 자세 정규화)')
    ax.set_ylabel('Y (mm)'); ax.set_aspect('equal')
    ax.set_xlim(-150, 900); ax.set_ylim(-90, 330)
    ax.legend(loc='upper left', fontsize=11, bbox_to_anchor=(0, -0.2), ncol=3, frameon=False)
    ax.text(1.0, -0.36, f'{run} 기록 재생', transform=ax.transAxes,
            ha='right', fontsize=10, color=MUTED)
    save(fig, 's11_tie_memory.png')
    print(f'     (검증: 요청 {len(req)} / 고유 {len(kept)} / 중복 {len(dup)})')


# ══ S12 안전 계층 + 결속 중 횡이동 사고 ═══════════════════════════
def s12():
    fig, ax = blank(8.5, 7)
    ax.set_title('계층 안전 — 위층이 뚫려도 아래층이 막는다', loc='left', pad=10)
    layers = [('기본값 안전: dry-run · 결속 off', '#f3f4f6'),
              ('소유권 중재: 웨이포인트(A) 우선, B는 즉시 양보', '#e0e7ff'),
              ('상태머신 감시: STOP 3연속 확인 · 전환 직전 재확인', '#dbeafe'),
              ('판정 노후화 워치독: 1.0s 끊기면 중단 · 화석 프레임 차단', '#cffafe'),
              ('drive_controller: 방향별 차단 · cmd_vel 0.5s 끊기면 정지', '#dcfce7'),
              ('범퍼: 부딪힌 방향만 차단 (전방향 차단 → 갇힘)', '#fef9c3'),
              ('모터 워치독 0xB3 500ms', '#fee2e2')]
    n = len(layers)
    for i, (t, c) in enumerate(layers):
        inset = i * 0.018
        box(ax, inset, 0.86 - i * 0.123, 1 - 2 * inset, 0.105, t, c, fs=12.5)
    ax.text(0.5, -0.03, '↓ 모터에 가까울수록 "무엇이 명령하든" 막는 층', ha='center',
            fontsize=11, color=MUTED)
    save(fig, 's12_safety_layers.png')

    # 2026-09-15 실측 타임라인 (autotying_bugfix ②)
    fig, ax = plt.subplots(figsize=(11, 3.8))
    t0 = 602.4
    up = [(609.1, '상부 X 202mm 뻗음'), (612.0, 'Z 하강'), (613.6, '트리거 발사')]
    lo = [(602.4, '하부 배근끝 STOP 확정\n→ SETTLE'), (610.6, '하부 횡이동 시작\n(총 420mm)')]
    ax.axhline(1, color=PURPLE, lw=6, alpha=0.25); ax.axhline(0, color=BLUE, lw=6, alpha=0.25)
    ax.axvspan(608.5 - t0, 614.5 - t0, color=RED, alpha=0.10)
    for t, s in up:
        ax.plot(t - t0, 1, 'o', color=PURPLE, ms=11); ax.text(t - t0, 1.2, s, ha='center', fontsize=11.5)
    for t, s in lo:
        ax.plot(t - t0, 0, 'o', color=BLUE, ms=11); ax.text(t - t0, -0.45, s, ha='center', fontsize=11.5)
    ax.set_yticks([0, 1], ['하부\n(주행)', '상부\n(결속)']); ax.set_ylim(-0.9, 1.6)
    ax.set_xlim(-1.5, 13.5); ax.set_xlabel('경과 시간 (s)')
    ax.grid(axis='y', visible=False)
    ax.set_title('실사고 (2026-09-15): 결속하는 도중 하부가 옆으로 420mm 이동', loc='left')
    ax.text(13.4, 0.5, '원인: 데크끝 검사가\n결속대기 검사보다 앞에\n→ 순서 교체로 수정',
            ha='right', va='center', fontsize=11, color=RED)
    save(fig, 's12_incident_timeline.png')


# ══ S13 실증 결과 ═════════════════════════════════════════════
def s13():
    # ① 100점 3회 (AUTONOMOUS_100PT_TYING_REPORT.md)
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(14, 4.8), gridspec_kw={'width_ratios': [1, 1.2]})
    tr = ['1회', '2회', '3회']; mins = [17.4, 22.8, 17.1]; pts = [100, 106, 101]
    b = ax.bar(tr, mins, color=BLUE, width=0.55)
    for bb, m, p in zip(b, mins, pts):
        ax.text(bb.get_x() + bb.get_width() / 2, m + 0.4, f'{m}분\n{p}점', ha='center', fontsize=12)
    ax.set_ylim(0, 27); ax.set_ylabel('소요 시간 (분)'); ax.grid(axis='x', visible=False)
    ax.set_title('100점 연속 결속 3회 — 평균 19.1분, 점당 11.2초', loc='left', fontsize=14)
    parts = [('결속 동작', 73, RED), ('주행', 17, GREEN), ('자세변경', 8, PURPLE), ('검출', 2, BLUE)]
    left = 0
    for name, v, c in parts:
        bx.barh(0, v, left=left, color=c, height=0.45)
        if v >= 15:
            bx.text(left + v / 2, 0, f'{name}\n{v}%', ha='center', va='center', color='white',
                    fontsize=14, fontweight='bold')
        else:
            bx.text(left + v / 2, 0.30 if name == '검출' else -0.30, f'{name} {v}%',
                    ha='center', va='center', color=c, fontsize=11.5, fontweight='bold')
        left += v
    bx.set_xlim(0, 100); bx.set_ylim(-0.7, 0.7); bx.set_yticks([]); bx.set_xlabel('시간 비중 (%)')
    bx.set_title('시간 구성 → 병목은 검출이 아니라 결속 동작', loc='left', fontsize=14)
    bx.grid(axis='y', visible=False)
    fig.tight_layout(); save(fig, 's13_100pt.png')

    # ② 스텝 도달오차: 전부 -13~-15 (로그 파싱)
    errs = []
    for f in sorted(glob.glob(os.path.join(WS, 'data/logs/auto_tying/202609*.log'))):
        for m in re.finditer(r'스텝 \d+ 완료: (\d+)mm \(목표 (\d+)mm\)', open(f, errors='ignore').read()):
            errs.append(int(m.group(1)) - int(m.group(2)))
    fig, ax = plt.subplots(figsize=(7.5, 4.3))
    vals, cnt = np.unique(errs, return_counts=True)
    ax.bar(vals, cnt, color=ORANGE, width=0.8)
    ax.axvline(-15, color=INK, ls='--')
    ax.text(-14.9, cnt.max() * 1.02, '도달 판정 허용오차 15mm', fontsize=11)
    ax.set_xticks(range(min(vals) - 1, max(vals) + 2)); ax.set_ylim(0, cnt.max() * 1.15)
    ax.set_xlabel('실제 - 목표 (mm)'); ax.set_ylabel('스텝 수')
    ax.set_title(f'스텝 {len(errs)}회 도달오차 — 무작위가 아니라 체계적', loc='left', fontsize=14)
    ax.grid(axis='x', visible=False)
    save(fig, 's13_step_error.png')
    print(f'     (스텝 {len(errs)}회, 평균 {np.mean(errs):.1f}, 범위 {min(errs)}~{max(errs)})')

    shutil.copy(os.path.join(WS, 'data/rebar_map/run_20260922_120018_lanes.png'),
                os.path.join(OUT, 's13_lanes_20260922.png'))
    print('  → (복사) s13_lanes_20260922.png')


ALL = {'s03': s03, 's04': s04, 's05': s05, 's06': s06, 's07': s07, 's08': s08,
       's09': s09, 's10': s10, 's11': s11, 's12': s12, 's13': s13}

if __name__ == '__main__':
    for k in (sys.argv[1:] or ALL):
        print(k)
        ALL[k]()
