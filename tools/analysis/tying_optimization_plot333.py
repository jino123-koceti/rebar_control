import os
IMG_DIR = os.path.join("/home/koceti/ros2_ws", "data", "images")
#!/usr/bin/env python3
"""Tying cycle optimization analysis - Run comparison plot (2026-03-11)
Includes Run 5~7 with speed/accel tuning and ready position optimization.
Pure tying time only (excludes Pose L→R return and Home return).
"""

import matplotlib.pyplot as plt
import matplotlib
import numpy as np

matplotlib.rcParams['font.size'] = 11

# ============================================================
# Run data (순수 결속 시간: 검출 → 마지막 포인트 Z↑ 완료)
# Pose L→R, Home return 제외
# ============================================================

runs = {
    'Run 5\n(800dps\n10k/5k)\n4pts': {
        'detection': 1.2,
        'points': [
            {'label': 'P1(R)', 'xy': 2.8, 'z_trig': 2.1},  # Z0.4+trig1.3+Z0.4
            {'label': 'P2(R)', 'xy': 2.2, 'z_trig': 2.1},
            {'label': 'P3(L)', 'xy': 2.7, 'z_trig': 2.0},
            {'label': 'P4(L)', 'xy': 2.3, 'z_trig': 2.0},
        ],
        'pose_rl': 5.5,
        'pose_lr': 5.5,  # excluded
        'home': 1.8,     # excluded
        'config': '800 dps, accel 10k, decel 5k',
    },
    'Run 6\n(1000dps\n15k/5k)\n4pts': {
        'detection': 1.1,
        'points': [
            {'label': 'P1(R)', 'xy': 2.6, 'z_trig': 2.0},  # Z0.4+trig1.2+Z0.4
            {'label': 'P2(R)', 'xy': 2.5, 'z_trig': 2.0},
            {'label': 'P3(L)', 'xy': 2.6, 'z_trig': 2.0},
            {'label': 'P4(L)', 'xy': 2.6, 'z_trig': 2.0},
        ],
        'pose_rl': 5.7,
        'pose_lr': 5.6,
        'home': 1.7,
        'config': '1000 dps, accel 15k, decel 5k',
    },
    'Run 7\n(+Ready\nX=200mm)\n3pts': {
        'detection': 1.1,
        'points': [
            {'label': 'P1(R)', 'xy': 1.2, 'z_trig': 2.0},
            {'label': 'P2(R)', 'xy': 2.4, 'z_trig': 2.0},
            {'label': 'P3(L)', 'xy': 1.0, 'z_trig': 2.0},
        ],
        'pose_rl': 5.6,
        'pose_lr': 5.6,
        'home': 1.7,
        'config': '1000 dps, 15k/5k, ready X=200mm',
    },
}

# 12-point estimation (순수 결속만: 검출 + 12pt work + Pose R→L)
est_12pt = {}
for name, data in runs.items():
    pts = data['points']
    avg_xy = np.mean([p['xy'] for p in pts])
    avg_ztrig = np.mean([p['z_trig'] for p in pts])
    avg_per_pt = avg_xy + avg_ztrig
    # first point uses actual P1 time, rest use average
    p1_time = pts[0]['xy'] + pts[0]['z_trig']
    total = data['detection'] + p1_time + 11 * avg_per_pt + data['pose_rl']
    est_12pt[name] = {
        'total': total,
        'pt_per_min': 12 / total * 60,
        'avg_per_pt': avg_per_pt,
        'p1_time': p1_time,
    }

# ============================================================
# Figure 1: Breakdown stacked bar + 12pt estimation
# ============================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 7))
fig.suptitle('Tying Cycle Optimization - Speed & Ready Position (2026-03-11)',
             fontsize=15, fontweight='bold', y=0.98)

colors = {
    'detection': '#4285F4',
    'xy': '#34A853',
    'z_trig': '#EA4335',
    'pose_rl': '#9C27B0',
}

# Left: Stacked bar - actual measured (순수 결속)
run_names = list(runs.keys())
x = np.arange(len(run_names))
bar_width = 0.5

for i, (name, data) in enumerate(runs.items()):
    bottom = 0
    pts = data['points']
    total_xy = sum(p['xy'] for p in pts)
    total_ztrig = sum(p['z_trig'] for p in pts)
    tying_total = data['detection'] + total_xy + total_ztrig + data['pose_rl']

    # Detection
    ax1.bar(i, data['detection'], bar_width, bottom=bottom, color=colors['detection'],
            label='Detection' if i == 0 else '')
    bottom += data['detection']

    # XY Move
    ax1.bar(i, total_xy, bar_width, bottom=bottom, color=colors['xy'],
            label='XY Move' if i == 0 else '')
    ax1.text(i, bottom + total_xy/2, f'{total_xy:.1f}s', ha='center', va='center',
             fontweight='bold', fontsize=10, color='white')
    bottom += total_xy

    # Z + Trigger
    ax1.bar(i, total_ztrig, bar_width, bottom=bottom, color=colors['z_trig'],
            label='Z + Trigger' if i == 0 else '')
    ax1.text(i, bottom + total_ztrig/2, f'{total_ztrig:.1f}s', ha='center', va='center',
             fontweight='bold', fontsize=10, color='white')
    bottom += total_ztrig

    # Pose R→L
    ax1.bar(i, data['pose_rl'], bar_width, bottom=bottom, color=colors['pose_rl'],
            label='Pose R→L' if i == 0 else '')
    ax1.text(i, bottom + data['pose_rl']/2, f'{data["pose_rl"]:.1f}s', ha='center', va='center',
             fontweight='bold', fontsize=10, color='white')
    bottom += data['pose_rl']

    # Total label
    ax1.text(i, bottom + 0.5, f'{tying_total:.1f}s', ha='center', va='bottom',
             fontweight='bold', fontsize=13, color='red')

ax1.set_xticks(x)
ax1.set_xticklabels(run_names, fontsize=9)
ax1.set_ylabel('Time (seconds)')
ax1.set_title('Actual Tying Time (excl. L→R return & Home)')
ax1.legend(loc='upper right')
ax1.set_ylim(0, 35)
ax1.grid(axis='y', alpha=0.3)

# Right: 12-point estimation comparison
est_names = list(est_12pt.keys())
x2 = np.arange(len(est_names))
bar_width2 = 0.35

# Total time bars
bars1 = ax2.bar(x2 - bar_width2/2, [est_12pt[n]['total'] for n in est_names],
                bar_width2, color=['#FF9800', '#2196F3', '#4CAF50'], alpha=0.8,
                label='12pt Est. Time (s)')

# pt/min bars on secondary axis
ax2b = ax2.twinx()
bars2 = ax2b.bar(x2 + bar_width2/2, [est_12pt[n]['pt_per_min'] for n in est_names],
                 bar_width2, color=['#FF9800', '#2196F3', '#4CAF50'], alpha=0.4,
                 edgecolor=['#FF9800', '#2196F3', '#4CAF50'], linewidth=2,
                 label='pt/min')

# Labels
for i, name in enumerate(est_names):
    d = est_12pt[name]
    ax2.text(i - bar_width2/2, d['total'] + 0.5, f"{d['total']:.1f}s",
             ha='center', va='bottom', fontweight='bold', fontsize=12, color='red')
    ax2b.text(i + bar_width2/2, d['pt_per_min'] + 0.2, f"{d['pt_per_min']:.1f}",
              ha='center', va='bottom', fontweight='bold', fontsize=12, color='blue')

# Target line
ax2b.axhline(y=12, color='red', linestyle='--', linewidth=2, alpha=0.7)
ax2b.text(len(est_names)-0.3, 12.3, 'Target: 12 pt/min', color='red',
          fontweight='bold', fontsize=11)

ax2.set_xticks(x2)
ax2.set_xticklabels(est_names, fontsize=9)
ax2.set_ylabel('Estimated 12pt Time (seconds)')
ax2b.set_ylabel('Points per Minute')
ax2.set_title('12-Point Estimation (Pure Tying)')
ax2.set_ylim(0, 80)
ax2b.set_ylim(0, 18)
ax2.grid(axis='y', alpha=0.3)

# Combined legend
lines1, labels1 = ax2.get_legend_handles_labels()
lines2, labels2 = ax2b.get_legend_handles_labels()
ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, 'tying_cycle_breakdown333.png'), dpi=150, bbox_inches='tight')
print("Saved: tying_cycle_breakdown333.png")

# ============================================================
# Figure 2: Timeline view
# ============================================================
fig2, axes = plt.subplots(3, 1, figsize=(18, 8), sharex=True)
fig2.suptitle('Tying Cycle Timeline - Optimization Comparison (2026-03-11)',
              fontsize=15, fontweight='bold', y=0.98)

color_map = {
    'det': '#4285F4',
    'xy': '#34A853',
    'ztrig': '#EA4335',
    'pose_rl': '#9C27B0',
}

for idx, (name, data) in enumerate(runs.items()):
    ax = axes[idx]
    t = 0
    pts = data['points']

    # Detection
    ax.barh(0, data['detection'], left=t, color=color_map['det'], height=0.6)
    ax.text(t + data['detection']/2, 0, f"Det\n{data['detection']:.1f}s",
            ha='center', va='center', fontsize=8, fontweight='bold', color='white')
    t += data['detection']

    # Right-side points
    right_pts = [p for p in pts if 'R' in p['label']]
    for p in right_pts:
        # XY
        ax.barh(0, p['xy'], left=t, color=color_map['xy'], height=0.6)
        ax.text(t + p['xy']/2, 0, f"{p['label']}\nXY {p['xy']:.1f}s",
                ha='center', va='center', fontsize=7, fontweight='bold', color='white')
        t += p['xy']
        # Z+Trigger
        ax.barh(0, p['z_trig'], left=t, color=color_map['ztrig'], height=0.6)
        ax.text(t + p['z_trig']/2, 0, f"Trig\n{p['z_trig']:.1f}s",
                ha='center', va='center', fontsize=7, fontweight='bold', color='white')
        t += p['z_trig']

    # Pose R→L
    ax.barh(0, data['pose_rl'], left=t, color=color_map['pose_rl'], height=0.6)
    ax.text(t + data['pose_rl']/2, 0, f"R→L\n{data['pose_rl']:.1f}s",
            ha='center', va='center', fontsize=8, fontweight='bold', color='white')
    t += data['pose_rl']

    # Left-side points
    left_pts = [p for p in pts if 'L' in p['label']]
    for p in left_pts:
        ax.barh(0, p['xy'], left=t, color=color_map['xy'], height=0.6)
        ax.text(t + p['xy']/2, 0, f"{p['label']}\nXY {p['xy']:.1f}s",
                ha='center', va='center', fontsize=7, fontweight='bold', color='white')
        t += p['xy']
        ax.barh(0, p['z_trig'], left=t, color=color_map['ztrig'], height=0.6)
        ax.text(t + p['z_trig']/2, 0, f"Trig\n{p['z_trig']:.1f}s",
                ha='center', va='center', fontsize=7, fontweight='bold', color='white')
        t += p['z_trig']

    # End marker
    tying_total = t
    ax.axvline(x=tying_total, color='red', linestyle='-', linewidth=2.5)
    ax.text(tying_total + 0.3, 0.35, f'{tying_total:.1f}s',
            fontweight='bold', fontsize=13, color='red')

    npts = len(pts)
    short_name = name.split('\n')[0]
    config_line = name.replace('\n', ' ')
    ax.set_ylabel(f'{short_name}\n({npts}pts)', fontsize=10, fontweight='bold')
    ax.set_yticks([])
    ax.set_xlim(0, 35)
    ax.grid(axis='x', alpha=0.3)

    # Config annotation
    est = est_12pt[name]
    ax.text(0.98, 0.95, f"12pt est: {est['total']:.1f}s ({est['pt_per_min']:.1f} pt/min)",
            transform=ax.transAxes, ha='right', va='top',
            fontsize=10, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.8))

axes[-1].set_xlabel('Time (seconds)', fontsize=12)

# Legend
from matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor=color_map['det'], label='Detection'),
    Patch(facecolor=color_map['xy'], label='XY Stage Move'),
    Patch(facecolor=color_map['ztrig'], label='Z + Trigger'),
    Patch(facecolor=color_map['pose_rl'], label='Pose Change R→L'),
]
fig2.legend(handles=legend_elements, loc='lower center', ncol=4, fontsize=10,
            bbox_to_anchor=(0.5, -0.02))

plt.tight_layout(rect=[0, 0.03, 1, 0.96])
plt.savefig(os.path.join(IMG_DIR, 'tying_cycle_timeline333.png'), dpi=150, bbox_inches='tight')
print("Saved: tying_cycle_timeline333.png")

plt.show()
