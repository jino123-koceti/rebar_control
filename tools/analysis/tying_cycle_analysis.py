import os
IMG_DIR = os.path.join("/home/koceti/ros2_ws", "data", "images")
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
import numpy as np

plt.rcParams['font.family'] = ['DejaVu Sans']
plt.rcParams['font.size'] = 11

# === Measured averages ===
right_point_avg = 6.4
left_point_avg = 8.6
pose_change_rl = 7.7
detection = 0.8

# === Gantt data: 결속 작업 구간만 (Pose L>R 복귀, Home 제거) ===
run_phases = []

r1 = [
    ('Detect', 0, 1.35),
    ('P1 XY', 1.40, 5.05), ('P1 Z+Trig', 5.10, 8.30),
    ('P2 XY', 8.45, 11.60), ('P2 Z+Trig', 11.65, 14.95),
    ('Pose R>L', 15.00, 22.70),
    ('P3 XY', 22.75, 28.35), ('P3 Z+Trig', 28.40, 31.70),
]
run_phases.append(('Run 1 (3pts)', 31.7, r1))

r2 = [
    ('Detect', 0, 0.10),
    ('P1 XY', 0.15, 3.45), ('P1 Z+Trig', 3.50, 6.80),
    ('P2 XY', 6.85, 9.95), ('P2 Z+Trig', 10.00, 13.30),
    ('Pose R>L', 13.35, 20.95),
    ('P3 XY', 21.00, 26.55), ('P3 Z+Trig', 26.60, 29.90),
]
run_phases.append(('Run 2 (3pts)', 29.9, r2))

r3 = [
    ('Detect', 0, 0.16),
    ('P1 XY', 0.21, 3.66), ('P1 Z+Trig', 3.71, 7.01),
    ('P2 XY', 7.06, 10.26), ('P2 Z+Trig', 10.31, 13.61),
    ('Pose R>L', 13.66, 21.26),
    ('P3 XY', 21.31, 26.81), ('P3 Z+Trig', 26.86, 30.16),
]
run_phases.append(('Run 3 (3pts)', 30.2, r3))

r4 = [
    ('Detect', 0, 0.17),
    ('P1 XY', 0.22, 3.67), ('P1 Z+Trig', 3.72, 7.02),
    ('P2 XY', 7.07, 10.37), ('P2 Z+Trig', 10.42, 13.72),
    ('Pose R>L', 13.77, 21.62),
    ('P3 XY', 21.67, 27.22), ('P3 Z+Trig', 27.27, 30.57),
    ('P4 XY', 30.62, 36.12), ('P4 Z+Trig', 36.17, 39.42),
]
run_phases.append(('Run 4 (4pts)', 39.4, r4))

# Estimated 6pt
t = 0
r6 = []
r6.append(('Detect', t, t + detection)); t += detection + 0.05
for i in range(3):
    r6.append((f'P{i+1} XY', t, t + 3.4)); t += 3.4 + 0.05
    r6.append((f'P{i+1} Z+Trig', t, t + 3.0)); t += 3.0 + 0.15
r6.append(('Pose R>L', t, t + pose_change_rl)); t += pose_change_rl + 0.05
for i in range(3):
    r6.append((f'P{i+4} XY', t, t + 5.5)); t += 5.5 + 0.05
    r6.append((f'P{i+4} Z+Trig', t, t + 3.0)); t += 3.0 + 0.15
est_total = round(t - 0.15, 1)  # last settling removed
run_phases.append(('Est. 6pts', est_total, r6))

# Colors
color_map = {
    'Detect': '#2196F3',
    'XY': '#4CAF50',
    'Z+Trig': '#F44336',
    'Pose': '#9C27B0',
}

def get_color(name):
    for key, c in color_map.items():
        if key in name:
            return c
    return '#999'

# ============================================================
# Figure 1: Gantt timeline (tying work only)
# ============================================================
fig, axes = plt.subplots(5, 1, figsize=(14, 9))
fig.suptitle('Automated Rebar Tying - Work Time Analysis (2026-03-09)\n(Tying work only, excludes post-tying return)', 
             fontsize=14, fontweight='bold')

for idx, (label, total, phases) in enumerate(run_phases):
    ax = axes[idx]
    is_est = idx == 4
    
    for name, ps, pe in phases:
        dur = pe - ps
        color = get_color(name)
        alpha = 0.6 if is_est else 1.0
        hatch = '///' if is_est else None
        ax.barh(0, dur, left=ps, height=0.6, color=color, edgecolor='white',
                linewidth=0.5, alpha=alpha, hatch=hatch)
        if dur > 1.8:
            short = name.replace('Z+Trig', 'Trig').replace('Pose R>L', 'R>L')
            ax.text(ps + dur/2, 0, f'{short}\n{dur:.1f}s', ha='center', va='center',
                   fontsize=7, fontweight='bold', color='white' if not is_est else 'black')
    
    ax.set_xlim(-1, 58)
    ax.set_yticks([])
    style = 'italic' if is_est else 'normal'
    ax.set_ylabel(label, fontsize=10, fontweight='bold', rotation=0, labelpad=72, va='center', style=style)
    ax.axvline(x=total, color='red', linestyle='--', alpha=0.7)
    ax.text(total + 0.5, 0, f'{total:.1f}s', color='red', fontsize=10, fontweight='bold', va='center')
    if is_est:
        ax.set_xlabel('Time (seconds)', fontsize=11)

from matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor='#2196F3', label='Detection'),
    Patch(facecolor='#4CAF50', label='XY Stage Move'),
    Patch(facecolor='#F44336', label='Z + Trigger'),
    Patch(facecolor='#9C27B0', label='Pose Change (Yaw)'),
    Patch(facecolor='white', edgecolor='black', hatch='///', label='Estimated'),
]
fig.legend(handles=legend_elements, loc='lower center', ncol=5, fontsize=9, bbox_to_anchor=(0.5, -0.01))
plt.tight_layout(rect=[0, 0.04, 1, 0.94])
plt.savefig(os.path.join(IMG_DIR, 'tying_cycle_timeline.png'), dpi=120, bbox_inches='tight')
print('Saved: tying_cycle_timeline.png')

# ============================================================
# Figure 2: Breakdown chart
# ============================================================
fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
fig2.suptitle('Tying Work Time Breakdown + 6-Point Estimation\n(Excludes post-tying return)', fontsize=14, fontweight='bold')

cats = ['Detection', 'XY Move', 'Z + Trigger', 'Pose Change']
cat_colors_list = ['#2196F3', '#4CAF50', '#F44336', '#9C27B0']

def calc_cats(phases):
    d = {c: 0.0 for c in cats}
    for name, ps, pe in phases:
        dur = pe - ps
        if 'Detect' in name: d['Detection'] += dur
        elif 'XY' in name: d['XY Move'] += dur
        elif 'Z+Trig' in name: d['Z + Trigger'] += dur
        elif 'Pose' in name: d['Pose Change'] += dur
    return d

all_cats = [calc_cats(p) for _, _, p in run_phases]
totals_list = [t for _, t, _ in run_phases]
labels = ['Run 1\n3pts', 'Run 2\n3pts', 'Run 3\n3pts', 'Run 4\n4pts', 'Est.\n6pts']

x = np.arange(len(labels))
bottom = np.zeros(len(labels))
for ci, cat in enumerate(cats):
    vals = [d[cat] for d in all_cats]
    bars = ax1.bar(x, vals, bottom=bottom, color=cat_colors_list[ci], label=cat,
                   edgecolor='white', linewidth=0.5)
    for i, v in enumerate(vals):
        if v > 2.0:
            ax1.text(i, bottom[i] + v/2, f'{v:.1f}s', ha='center', va='center',
                    fontsize=8, fontweight='bold', color='white')
    bottom += vals

for patch_set in ax1.containers:
    for patch in patch_set:
        if patch.get_x() > 3.5:
            patch.set_hatch('///')

for i, total in enumerate(totals_list):
    ax1.text(i, bottom[i] + 0.8, f'{total:.1f}s', ha='center', va='bottom',
            fontsize=11, fontweight='bold', color='red')

ax1.set_xticks(x)
ax1.set_xticklabels(labels, fontsize=10)
ax1.set_ylabel('Time (seconds)', fontsize=12)
ax1.set_title('Total Work Time by Category', fontsize=13, fontweight='bold')
ax1.legend(loc='upper left', fontsize=8)
ax1.set_ylim(0, 60)

# Per-point tying time
run_colors_bar = ['#1976D2', '#388E3C', '#F57C00', '#D32F2F', '#7B1FA2']
actual_per_point = [
    [('P1(R)', 6.7), ('P2(R)', 6.2), ('P3(L)', 8.7)],
    [('P1(R)', 6.3), ('P2(R)', 6.2), ('P3(L)', 8.6)],
    [('P1(R)', 6.5), ('P2(R)', 6.2), ('P3(L)', 8.5)],
    [('P1(R)', 6.5), ('P2(R)', 6.3), ('P3(L)', 8.6), ('P4(L)', 8.6)],
    [('P1(R)', right_point_avg), ('P2(R)', right_point_avg), ('P3(R)', right_point_avg),
     ('P4(L)', left_point_avg), ('P5(L)', left_point_avg), ('P6(L)', left_point_avg)],
]

p_labels = []
p_times = []
p_colors = []
p_hatch = []
for ri, pts in enumerate(actual_per_point):
    for pname, pt in pts:
        p_labels.append(f'R{ri+1}\n{pname}')
        p_times.append(pt)
        p_colors.append(run_colors_bar[ri])
        p_hatch.append('///' if ri == 4 else None)

x2 = np.arange(len(p_labels))
bars2 = ax2.bar(x2, p_times, color=p_colors, edgecolor='white', linewidth=0.5, width=0.7)
for i, h in enumerate(p_hatch):
    if h:
        bars2[i].set_hatch(h)

for i, v in enumerate(p_times):
    ax2.text(i, v + 0.1, f'{v:.1f}', ha='center', va='bottom', fontsize=7, fontweight='bold')

ax2.set_xticks(x2)
ax2.set_xticklabels(p_labels, fontsize=6)
ax2.set_ylabel('Time (seconds)', fontsize=12)
ax2.set_title('Per-Point Tying Time (XY+Z+Trigger)', fontsize=13, fontweight='bold')
ax2.set_ylim(0, 12)

avg_all = np.mean(p_times)
ax2.axhline(y=avg_all, color='red', linestyle='--', alpha=0.7)
ax2.text(len(p_labels)-0.5, avg_all+0.2, f'Avg: {avg_all:.1f}s', color='red', fontsize=9, fontweight='bold', ha='right')

run_legend = [Patch(facecolor=c, label=f'Run {i+1}' if i < 4 else 'Est. 6pts')
              for i, c in enumerate(run_colors_bar)]
ax2.legend(handles=run_legend, loc='upper right', fontsize=8)

plt.tight_layout(rect=[0, 0, 1, 0.93])
plt.savefig(os.path.join(IMG_DIR, 'tying_cycle_breakdown.png'), dpi=120, bbox_inches='tight')
print('Saved: tying_cycle_breakdown.png')

# Summary
print(f'\n=== Work Time Summary (tying only, no return) ===')
for idx, (label, total, phases) in enumerate(run_phases):
    d = calc_cats(phases)
    n = sum(1 for name, _, _ in phases if 'Z+Trig' in name)
    print(f'{label}: {total:.1f}s total | {n}pts | XY={d["XY Move"]:.1f}s Z+Trig={d["Z + Trigger"]:.1f}s Pose={d["Pose Change"]:.1f}s')

