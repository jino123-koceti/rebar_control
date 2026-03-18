import os
IMG_DIR = os.path.join("/home/koceti/ros2_ws", "data", "images")
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
import numpy as np

plt.rcParams['font.family'] = ['DejaVu Sans']
plt.rcParams['font.size'] = 11

# ============================================================
# Data from 5 test runs
# ============================================================
runs = [
    {
        'label': 'Run 1\nBaseline',
        'config': 'speed=400\naccel=5k/5k',
        'pts': 3,
        'total': 45.9,
        'p_times': [6.7, 6.2, 8.7],  # per-point (XY+Z+trig+Z)
        'pose_rl': 7.7,
        'pose_lr': 11.7,
        'detect': 1.4,
        'home_return': 2.2,
    },
    {
        'label': 'Run 2\naccel 10k',
        'config': 'speed=400\naccel=10k/10k',
        'pts': 3,
        'total': 30.4,
        'p_times': [4.5, 4.5, 4.5],  # estimated from log gaps
        'pose_rl': 5.6,
        'pose_lr': 6.2,
        'detect': 0.2,
        'home_return': 1.8,
    },
    {
        'label': 'Run 3\n500 dps',
        'config': 'speed=500\naccel=10k/10k',
        'pts': 3,
        'total': 29.2,
        'p_times': [4.3, 4.1, 4.0],
        'pose_rl': 5.3,
        'pose_lr': 5.5,
        'detect': 0.2,
        'home_return': 1.6,
    },
    {
        'label': 'Run 4\n800 dps',
        'config': 'speed=800\naccel=10k/10k',
        'pts': 4,
        'total': 35.4,
        'p_times': [4.5, 4.2, 4.7, 4.3],
        'pose_rl': 5.5,
        'pose_lr': 5.5,
        'detect': 0.2,
        'home_return': 1.8,
    },
    {
        'label': 'Run 5\nFinal',
        'config': 'speed=800\naccel=10k/5k',
        'pts': 4,
        'total': 37.9,
        'p_times': [5.0, 4.6, 4.9, 4.8],
        'pose_rl': 6.1,
        'pose_lr': 5.8,
        'detect': 0.2,
        'home_return': 1.8,
    },
]

# ============================================================
# Figure: 3 subplots
# ============================================================
fig = plt.figure(figsize=(18, 14))
fig.suptitle('Rebar Tying Cycle - Optimization Analysis (2026-03-11)', 
             fontsize=18, fontweight='bold', y=0.98)

# ------ Plot 1: Stacked bar - time breakdown ------
ax1 = fig.add_subplot(2, 2, 1)

categories = ['Detection', 'Tying (XY+Z+Trig)', 'Pose R→L', 'Pose L→R', 'Home Return']
cat_colors = ['#2196F3', '#4CAF50', '#9C27B0', '#E040FB', '#607D8B']

x_pos = np.arange(len(runs))
bar_width = 0.6

bottoms = np.zeros(len(runs))
for ci, cat in enumerate(categories):
    vals = []
    for r in runs:
        if ci == 0:
            vals.append(r['detect'])
        elif ci == 1:
            vals.append(sum(r['p_times']))
        elif ci == 2:
            vals.append(r['pose_rl'])
        elif ci == 3:
            vals.append(r['pose_lr'])
        elif ci == 4:
            vals.append(r['home_return'])
    
    bars = ax1.bar(x_pos, vals, bar_width, bottom=bottoms, color=cat_colors[ci], 
                   label=cat, edgecolor='white', linewidth=0.5)
    for i, v in enumerate(vals):
        if v > 1.5:
            ax1.text(i, bottoms[i] + v/2, f'{v:.1f}s', ha='center', va='center',
                    fontsize=8, fontweight='bold', color='white')
    bottoms += vals

for i, r in enumerate(runs):
    ax1.text(i, bottoms[i] + 0.8, f'{r["total"]:.1f}s\n({r["pts"]}pt)', 
             ha='center', va='bottom', fontsize=10, fontweight='bold', color='red')

ax1.set_xticks(x_pos)
ax1.set_xticklabels([r['label'] for r in runs], fontsize=9)
ax1.set_ylabel('Time (seconds)', fontsize=12)
ax1.set_title('Total Cycle Time Breakdown', fontsize=13, fontweight='bold')
ax1.legend(loc='upper right', fontsize=8)
ax1.set_ylim(0, 55)

# ------ Plot 2: Per-point tying time trend ------
ax2 = fig.add_subplot(2, 2, 2)

avg_per_point = [np.mean(r['p_times']) for r in runs]
colors = ['#90CAF9', '#66BB6A', '#43A047', '#2E7D32', '#1B5E20']

bars2 = ax2.bar(x_pos, avg_per_point, bar_width, color=colors, edgecolor='white', linewidth=0.5)
for i, v in enumerate(avg_per_point):
    ax2.text(i, v + 0.1, f'{v:.1f}s', ha='center', va='bottom', fontsize=11, fontweight='bold')

# Target line
ax2.axhline(y=5.0, color='red', linestyle='--', alpha=0.7, linewidth=1.5)
ax2.text(len(runs)-0.5, 5.15, 'Target: 5.0s/pt\n(12 pt/min)', color='red', 
         fontsize=9, fontweight='bold', ha='right')

ax2.set_xticks(x_pos)
ax2.set_xticklabels([r['label'] for r in runs], fontsize=9)
ax2.set_ylabel('Time per Point (seconds)', fontsize=12)
ax2.set_title('Average Per-Point Tying Time', fontsize=13, fontweight='bold')
ax2.set_ylim(0, 9)

# Add config annotations
for i, r in enumerate(runs):
    ax2.text(i, 0.3, r['config'], ha='center', va='bottom', fontsize=7, 
             color='gray', style='italic')

# ------ Plot 3: 12-point estimation ------
ax3 = fig.add_subplot(2, 1, 2)

# 12-point = 6 right + 6 left = 2 pose changes + 12 points
# Estimation: detect + 6*per_pt + pose_rl + 6*per_pt + pose_lr + home
est_12pt = []
labels_12 = []
breakdown_12 = []

for r in runs:
    avg_pt = np.mean(r['p_times'])
    detect = r['detect']
    pose_rl = r['pose_rl']
    pose_lr = r['pose_lr']
    home = r['home_return']
    
    total_12 = detect + 6*avg_pt + pose_rl + 6*avg_pt + pose_lr + home
    est_12pt.append(total_12)
    labels_12.append(r['label'])
    breakdown_12.append({
        'detect': detect,
        'tying_right': 6*avg_pt,
        'pose_rl': pose_rl,
        'tying_left': 6*avg_pt,
        'pose_lr': pose_lr,
        'home': home,
    })

# Stacked horizontal bars
categories_12 = ['Detection', 'Right 6pt', 'Pose R→L', 'Left 6pt', 'Pose L→R', 'Home']
colors_12 = ['#2196F3', '#4CAF50', '#9C27B0', '#66BB6A', '#E040FB', '#607D8B']
keys_12 = ['detect', 'tying_right', 'pose_rl', 'tying_left', 'pose_lr', 'home']

y_pos = np.arange(len(runs))
lefts = np.zeros(len(runs))

for ci, (cat, key, color) in enumerate(zip(categories_12, keys_12, colors_12)):
    vals = [b[key] for b in breakdown_12]
    bars = ax3.barh(y_pos, vals, 0.6, left=lefts, color=color, label=cat, 
                    edgecolor='white', linewidth=0.5)
    for i, v in enumerate(vals):
        if v > 4:
            ax3.text(lefts[i] + v/2, i, f'{v:.1f}s', ha='center', va='center',
                    fontsize=8, fontweight='bold', color='white')
    lefts += vals

# Total labels
for i, total in enumerate(est_12pt):
    ppm = 12 * 60 / total  # points per minute
    ax3.text(total + 1, i, f'{total:.0f}s ({ppm:.1f} pt/min)', 
             va='center', fontsize=11, fontweight='bold', color='red')

# 60s target line
ax3.axvline(x=60, color='red', linestyle='--', alpha=0.7, linewidth=2)
ax3.text(61, len(runs)-0.5, '60s target\n(12 pt/min)', color='red', fontsize=10, 
         fontweight='bold', va='center')

ax3.set_yticks(y_pos)
ax3.set_yticklabels([r['label'] for r in runs], fontsize=10)
ax3.set_xlabel('Estimated Time for 12 Points (seconds)', fontsize=12)
ax3.set_title('12-Point Cycle Time Estimation (6 Right + 6 Left)', fontsize=13, fontweight='bold')
ax3.legend(loc='lower right', fontsize=9, ncol=3)
ax3.set_xlim(0, max(est_12pt) + 20)

# Improvement annotation
improvement = (1 - est_12pt[-1]/est_12pt[0]) * 100
fig.text(0.5, 0.01, 
         f'Overall improvement: {est_12pt[0]:.0f}s → {est_12pt[-1]:.0f}s '
         f'({improvement:.0f}% reduction) | '
         f'Current: {12*60/est_12pt[-1]:.1f} pt/min | Target: 12.0 pt/min',
         ha='center', fontsize=12, fontweight='bold',
         bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', edgecolor='orange'))

plt.tight_layout(rect=[0, 0.04, 1, 0.95])
plt.savefig(os.path.join(IMG_DIR, 'tying_optimization_analysis.png'), dpi=150, bbox_inches='tight')
print('Saved: tying_optimization_analysis.png')

# Summary
print('\n=== 12-Point Estimation ===')
for r, est in zip(runs, est_12pt):
    ppm = 12 * 60 / est
    print(f'{r["label"].replace(chr(10)," ")}: {est:.0f}s ({ppm:.1f} pt/min)')

