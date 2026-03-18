#!/usr/bin/env python3
"""
100pt 자율결속 3회 시험 분석 및 플롯
2026-03-16 실측 데이터
"""
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from datetime import datetime

matplotlib.rcParams['font.family'] = ['DejaVu Sans', 'sans-serif']

# ============================================================
# 데이터 (로그에서 추출)
# ============================================================

# 1차 시험: 15:36:55 ~ 15:54:18, 4WP 원본, 11회차, 100pt
trial1 = {
    'name': 'Trial 1',
    'start': '15:36:55', 'end': '15:54:18',
    'original_wp': 4,
    'laps': 11,
    'total_pts': 100,
    # (timestamp, wp_pts, cumulative)
    'events': [
        ('15:37:17', 3, 3), ('15:37:29', 0, 3), ('15:38:01', 2, 5), ('15:38:23', 2, 7),
        ('15:39:01', 4, 11), ('15:39:24', 2, 13), ('15:40:09', 4, 17),
        ('15:40:35', 2, 19), ('15:41:09', 3, 22), ('15:41:45', 3, 25),
        ('15:42:24', 4, 29), ('15:43:01', 4, 33), ('15:43:26', 3, 36),
        ('15:43:45', 2, 38), ('15:44:19', 3, 41), ('15:44:54', 3, 44),
        ('15:45:33', 4, 48), ('15:46:11', 4, 52), ('15:46:24', 0, 52),
        ('15:46:45', 2, 54), ('15:47:18', 3, 57), ('15:47:55', 3, 60),
        ('15:48:33', 4, 64), ('15:49:11', 4, 68), ('15:49:30', 2, 70),
        ('15:49:49', 2, 72), ('15:50:32', 5, 77), ('15:51:09', 4, 81),
        ('15:51:48', 4, 85), ('15:52:20', 2, 87), ('15:52:40', 2, 89),
        ('15:52:59', 2, 91), ('15:53:41', 5, 96), ('15:54:18', 4, 100),
    ],
}

# 2차 시험: 16:41:30 ~ 17:04:17, 3WP 원본, 26회차, 106pt
trial2 = {
    'name': 'Trial 2',
    'start': '16:41:30', 'end': '17:04:17',
    'original_wp': 3,
    'laps': 26,
    'total_pts': 106,
    'events': [
        ('16:42:43', 3, 3), ('16:43:13', 0, 3), ('16:43:55', 2, 5), ('16:44:17', 2, 7),
        ('16:44:55', 4, 11), ('16:45:18', 2, 13), ('16:46:03', 4, 17),
        ('16:46:30', 2, 19), ('16:47:03', 3, 22), ('16:47:39', 3, 25),
        ('16:48:18', 4, 29), ('16:48:55', 4, 33), ('16:49:20', 3, 36),
        ('16:49:39', 2, 38), ('16:50:13', 3, 41), ('16:50:48', 3, 44),
        ('16:51:27', 4, 48), ('16:52:05', 4, 52), ('16:52:18', 0, 52),
        ('16:52:39', 2, 54), ('16:53:12', 3, 57), ('16:53:49', 3, 60),
        ('16:54:27', 4, 64), ('16:55:05', 4, 68), ('16:55:24', 2, 70),
        ('16:55:43', 2, 72), ('16:56:26', 5, 77), ('16:57:03', 4, 81),
        ('16:57:19', 0, 81), ('16:57:49', 2, 83), ('16:58:08', 2, 85),
        ('16:58:50', 5, 90), ('16:59:04', 0, 90), ('16:59:35', 2, 92),
        ('16:59:52', 2, 94), ('17:00:35', 5, 99), ('17:01:05', 2, 101),
        ('17:01:29', 3, 104), ('17:01:48', 2, 106),  # approximate tail
    ],
}

# 3차 시험: 17:10:35 ~ 17:27:39, 4WP 원본, 13회차, 101pt
trial3 = {
    'name': 'Trial 3',
    'start': '17:10:35', 'end': '17:27:39',
    'original_wp': 4,
    'laps': 13,
    'total_pts': 101,
    'events': [
        ('17:10:47', 2, 2), ('17:10:59', 2, 4), ('17:11:22', 2, 6), ('17:12:03', 3, 9),
        ('17:12:41', 4, 13), ('17:13:19', 4, 17), ('17:13:33', 2, 19),
        ('17:13:43', 2, 21), ('17:14:05', 2, 23), ('17:14:47', 3, 26),
        ('17:15:24', 4, 30), ('17:16:02', 4, 34), ('17:16:16', 2, 36),
        ('17:16:26', 2, 38), ('17:16:48', 2, 40), ('17:17:11', 2, 42),
        ('17:17:49', 4, 46), ('17:18:21', 2, 48), ('17:18:35', 2, 50),
        ('17:18:44', 2, 52), ('17:19:07', 2, 54), ('17:19:49', 3, 57),
        ('17:20:26', 4, 61), ('17:21:01', 3, 64), ('17:21:16', 2, 66),
        ('17:21:25', 2, 68), ('17:21:48', 2, 70), ('17:22:30', 3, 73),
        ('17:23:07', 4, 77), ('17:23:35', 1, 78), ('17:23:49', 2, 80),
        ('17:23:58', 2, 82), ('17:24:21', 2, 84), ('17:25:03', 3, 87),
        ('17:25:41', 4, 91), ('17:26:13', 2, 93), ('17:26:27', 2, 95),
        ('17:26:34', 1, 96), ('17:26:57', 2, 98), ('17:27:39', 3, 101),
    ],
}


def parse_time(t_str):
    return datetime.strptime(t_str, '%H:%M:%S')


def get_elapsed_seconds(events, start_str):
    start = parse_time(start_str)
    times = []
    for ts, pts, cum in events:
        t = parse_time(ts)
        elapsed = (t - start).total_seconds()
        times.append(elapsed)
    return times


def analyze_trial(trial):
    """시험 데이터 분석"""
    start = parse_time(trial['start'])
    end = parse_time(trial['end'])
    total_sec = (end - start).total_seconds()

    pts_list = [e[1] for e in trial['events']]
    cum_list = [e[2] for e in trial['events']]
    elapsed = get_elapsed_seconds(trial['events'], trial['start'])

    # WP간 시간 (연속 이벤트 간 간격)
    wp_times = [elapsed[0]] + [elapsed[i] - elapsed[i-1] for i in range(1, len(elapsed))]

    n_zero = sum(1 for p in pts_list if p == 0)
    n_wp = len(pts_list)

    return {
        'total_sec': total_sec,
        'total_pts': cum_list[-1],
        'n_wp': n_wp,
        'pts_per_wp': cum_list[-1] / n_wp,
        'sec_per_pt': total_sec / cum_list[-1],
        'n_zero': n_zero,
        'zero_pct': n_zero / n_wp * 100,
        'pts_list': pts_list,
        'cum_list': cum_list,
        'elapsed': elapsed,
        'wp_times': wp_times,
        'laps': trial['laps'],
        'original_wp': trial['original_wp'],
    }


# ============================================================
# 분석
# ============================================================
trials = [trial1, trial2, trial3]
analyses = [analyze_trial(t) for t in trials]

# 터미널 출력
print('=' * 70)
print('  100pt Autonomous Tying - 3 Trials Analysis')
print('=' * 70)
print(f'\n{"":>20} {"Trial 1":>12} {"Trial 2":>12} {"Trial 3":>12} {"Average":>12}')
print('-' * 70)

metrics = [
    ('Total Points', 'total_pts', '{:.0f}', 'pt'),
    ('Total Time', 'total_sec', '{:.0f}', 'sec'),
    ('Total Time', 'total_sec', None, 'min'),
    ('Laps', 'laps', '{:.0f}', ''),
    ('Original WPs', 'original_wp', '{:.0f}', ''),
    ('Total WP Visits', 'n_wp', '{:.0f}', ''),
    ('Points/WP', 'pts_per_wp', '{:.1f}', ''),
    ('Seconds/Point', 'sec_per_pt', '{:.1f}', 'sec'),
    ('Zero-detect WPs', 'n_zero', '{:.0f}', ''),
    ('Zero-detect %', 'zero_pct', '{:.1f}', '%'),
]

for label, key, fmt, unit in metrics:
    vals = [a[key] for a in analyses]
    avg = np.mean(vals)
    if key == 'total_sec' and unit == 'min':
        vals_str = [f'{v/60:.1f}min' for v in vals]
        avg_str = f'{avg/60:.1f}min'
        print(f'{label + " (min)":<20} {vals_str[0]:>12} {vals_str[1]:>12} {vals_str[2]:>12} {avg_str:>12}')
    elif fmt:
        vals_str = [f'{fmt.format(v)}{unit}' for v in vals]
        avg_str = f'{fmt.format(avg)}{unit}'
        print(f'{label:<20} {vals_str[0]:>12} {vals_str[1]:>12} {vals_str[2]:>12} {avg_str:>12}')

# ============================================================
# 플롯
# ============================================================
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('100-Point Autonomous Tying: 3-Trial Analysis (2026-03-16)',
             fontsize=14, fontweight='bold')

colors = ['#2196F3', '#FF9800', '#4CAF50']
labels = ['Trial 1 (4WP)', 'Trial 2 (3WP)', 'Trial 3 (4WP)']

# --- Plot 1: Cumulative points over time ---
ax1 = axes[0, 0]
for i, (trial, a) in enumerate(zip(trials, analyses)):
    elapsed = a['elapsed']
    cum = a['cum_list']
    # Add starting point
    ax1.plot([0] + elapsed, [0] + cum, '-o', color=colors[i], label=labels[i],
             markersize=2, linewidth=1.5)

ax1.axhline(y=100, color='red', linestyle='--', alpha=0.5, label='Target (100pt)')
ax1.set_xlabel('Elapsed Time (sec)')
ax1.set_ylabel('Cumulative Points')
ax1.set_title('Cumulative Points vs Time')
ax1.legend(fontsize=8)
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0)
ax1.set_ylim(0, 115)

# --- Plot 2: Points per WP histogram ---
ax2 = axes[0, 1]
bins = range(0, 7)
for i, a in enumerate(analyses):
    counts, _ = np.histogram(a['pts_list'], bins=list(bins) + [7])
    ax2.bar(np.array(list(bins)) + i*0.25 - 0.25, counts, width=0.25,
            color=colors[i], label=labels[i], alpha=0.8)

ax2.set_xlabel('Points per WP Visit')
ax2.set_ylabel('Count')
ax2.set_title('Detection Distribution per WP')
ax2.set_xticks(range(0, 7))
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3, axis='y')

# --- Plot 3: WP processing time ---
ax3 = axes[1, 0]
for i, a in enumerate(analyses):
    ax3.plot(range(len(a['wp_times'])), a['wp_times'], '-', color=colors[i],
             label=labels[i], alpha=0.7, linewidth=1)
    avg_time = np.mean(a['wp_times'])
    ax3.axhline(y=avg_time, color=colors[i], linestyle=':', alpha=0.5)

ax3.set_xlabel('WP Visit Index')
ax3.set_ylabel('Time (sec)')
ax3.set_title('Per-WP Processing Time (tying + travel)')
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.3)

# --- Plot 4: Summary bar chart ---
ax4 = axes[1, 1]
trial_labels = ['Trial 1\n(4WP, 11lap)', 'Trial 2\n(3WP, 26lap)', 'Trial 3\n(4WP, 13lap)']
total_times = [a['total_sec'] for a in analyses]
total_pts = [a['total_pts'] for a in analyses]
sec_per_pt = [a['sec_per_pt'] for a in analyses]

x = np.arange(len(trial_labels))
width = 0.35

bars1 = ax4.bar(x - width/2, [t/60 for t in total_times], width,
                color=[c for c in colors], alpha=0.7, label='Total Time (min)')
bars2_ax = ax4.twinx()
bars2 = bars2_ax.bar(x + width/2, sec_per_pt, width,
                     color=[c for c in colors], alpha=0.4, hatch='//',
                     label='Sec/Point')

ax4.set_xlabel('')
ax4.set_ylabel('Total Time (min)', color='black')
bars2_ax.set_ylabel('Seconds per Point', color='gray')
ax4.set_title('Performance Summary')
ax4.set_xticks(x)
ax4.set_xticklabels(trial_labels, fontsize=9)

# Add value labels
for bar, val in zip(bars1, total_times):
    ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
             f'{val/60:.1f}m', ha='center', va='bottom', fontsize=9, fontweight='bold')
for bar, val in zip(bars2, sec_per_pt):
    bars2_ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                  f'{val:.1f}s', ha='center', va='bottom', fontsize=9, color='gray')

ax4.grid(True, alpha=0.3, axis='y')

# Average line
avg_time_min = np.mean(total_times) / 60
ax4.axhline(y=avg_time_min, color='red', linestyle='--', alpha=0.5)
ax4.text(2.5, avg_time_min, f'Avg: {avg_time_min:.1f}min', color='red', fontsize=8)

plt.tight_layout()
plt.savefig('tying_100pt_analysis.png', dpi=150, bbox_inches='tight')
print(f'\nPlot saved: tying_100pt_analysis.png')

# 2nd plot: rate over time (points per minute)
fig2, ax = plt.subplots(figsize=(12, 5))
fig2.suptitle('Tying Rate Over Time (Points/min, rolling 5-WP window)',
              fontsize=13, fontweight='bold')

for i, (trial, a) in enumerate(zip(trials, analyses)):
    elapsed = np.array(a['elapsed'])
    pts = np.array(a['pts_list'])

    # Rolling rate: pts per minute over 5-WP window
    window = 5
    rates = []
    rate_times = []
    for j in range(window, len(elapsed)):
        dt = elapsed[j] - elapsed[j - window]
        if dt > 0:
            rate = sum(pts[j-window:j]) / (dt / 60.0)
            rates.append(rate)
            rate_times.append(elapsed[j])

    ax.plot(rate_times, rates, '-', color=colors[i], label=labels[i],
            alpha=0.7, linewidth=1.5)

ax.axhline(y=np.mean([a['total_pts'] / (a['total_sec']/60) for a in analyses]),
           color='red', linestyle='--', alpha=0.5, label='Overall Avg')
ax.set_xlabel('Elapsed Time (sec)')
ax.set_ylabel('Points / min')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_xlim(0)

plt.tight_layout()
plt.savefig('tying_100pt_rate.png', dpi=150, bbox_inches='tight')
print(f'Plot saved: tying_100pt_rate.png')

plt.show()
