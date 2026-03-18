import os
IMG_DIR = os.path.join("/home/koceti/ros2_ws", "data", "images")
#!/usr/bin/env python3
"""12포인트 체결 사이클 분석 보고서 차트 생성"""

import matplotlib.pyplot as plt
import matplotlib
import numpy as np

matplotlib.rcParams['font.family'] = 'DejaVu Sans'
matplotlib.rcParams['font.size'] = 11

# === 3회 사이클 데이터 (홈복귀 제외) ===
cycles = ['Cycle 1', 'Cycle 2', 'Cycle 3']

# 각 사이클별 구간 시간 (초)
detection_1 = [1.8, 0.9, 0.8]          # 초기 검출
forward_right = [11.4, 11.2, 11.3]      # 전진 Right P1~P3
pose_change_1 = [6.1, 6.1, 6.2]         # 자세변경1 (R→L)
forward_left = [10.3, 10.3, 10.3]       # 전진 Left P4~P6
re_detection = [0.9, 0.9, 0.9]          # 재검출
return_left = [9.8, 9.9, 9.8]           # 복귀 Left P6'~P4'
pose_change_2 = [5.8, 5.9, 6.1]         # 자세변경2 (L→R)
return_right = [12.8, 12.6, 12.6]       # 복귀 Right P3'~P1'

# 전체 시간 (TYING_START ~ 마지막 트리거)
total_times = [58.9, 57.8, 58.0]

# === Figure 1: 프로세스별 Stacked Bar Chart ===
fig, axes = plt.subplots(1, 2, figsize=(16, 7), gridspec_kw={'width_ratios': [1.2, 1]})

# 색상 팔레트
colors = ['#4CAF50', '#2196F3', '#FF9800', '#03A9F4', '#9C27B0', '#00BCD4', '#FF5722', '#3F51B5']
labels = [
    'Detection',
    'Forward: Right (P1-P3)',
    'Pose Change 1 (R→L)',
    'Forward: Left (P4-P6)',
    'Re-detection',
    'Return: Left (P6\'-P4\')',
    'Pose Change 2 (L→R)',
    'Return: Right (P3\'-P1\')'
]

data = [detection_1, forward_right, pose_change_1, forward_left,
        re_detection, return_left, pose_change_2, return_right]

ax1 = axes[0]
x = np.arange(len(cycles))
bar_width = 0.5
bottoms = np.zeros(len(cycles))

for i, (d, color, label) in enumerate(zip(data, colors, labels)):
    bars = ax1.bar(x, d, bar_width, bottom=bottoms, color=color, label=label, edgecolor='white', linewidth=0.5)
    # 구간 시간 표시 (충분히 큰 구간만)
    for j, val in enumerate(d):
        if val >= 2.0:
            ax1.text(x[j], bottoms[j] + val/2, f'{val:.1f}s',
                    ha='center', va='center', fontsize=9, fontweight='bold', color='white')
    bottoms += np.array(d)

# 전체 시간 표시
for j, total in enumerate(total_times):
    ax1.text(x[j], bottoms[j] + 0.8, f'{total:.1f}s',
            ha='center', va='bottom', fontsize=13, fontweight='bold', color='#333')

ax1.set_xlabel('Test Cycle', fontsize=12)
ax1.set_ylabel('Time (seconds)', fontsize=12)
ax1.set_title('12-Point Tying Cycle Time Breakdown', fontsize=14, fontweight='bold')
ax1.set_xticks(x)
ax1.set_xticklabels(cycles)
ax1.set_ylim(0, max(total_times) + 5)
ax1.legend(loc='lower center', bbox_to_anchor=(0.5, -0.18), fontsize=8, framealpha=0.9, ncol=4)
ax1.grid(axis='y', alpha=0.3)
ax1.axhline(y=60, color='red', linestyle='--', alpha=0.5, label='60s target')
ax1.text(2.35, 60.5, '60s target', color='red', fontsize=9, alpha=0.7)

# === Figure 2: 평균 시간 파이 차트 ===
ax2 = axes[1]

avg_data = [np.mean(d) for d in data]
avg_labels_with_time = [f'{l}\n({v:.1f}s)' for l, v in zip(labels, avg_data)]

wedges, texts, autotexts = ax2.pie(
    avg_data, labels=None, colors=colors, autopct='%1.1f%%',
    startangle=90, pctdistance=0.78,
    wedgeprops=dict(width=0.55, edgecolor='white', linewidth=1.5)
)

for autotext in autotexts:
    autotext.set_fontsize(9)
    autotext.set_fontweight('bold')

# 범례
ax2.legend(wedges, [f'{l} ({v:.1f}s)' for l, v in zip(labels, avg_data)],
          loc='center left', bbox_to_anchor=(-0.15, -0.25), fontsize=8.5, ncol=2)

avg_total = np.mean(total_times)
ax2.set_title(f'Average Time Distribution\n(Total: {avg_total:.1f}s / 12 points)',
             fontsize=14, fontweight='bold')

# 중앙 텍스트
ax2.text(0, 0, f'{avg_total:.1f}s\navg', ha='center', va='center',
        fontsize=16, fontweight='bold', color='#333')

plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, 'tying_cycle_analysis_report.png'), dpi=150, bbox_inches='tight',
            facecolor='white', edgecolor='none')
plt.close()

# === Figure 3: 포인트별 상세 타이밍 (사이클1 기준) ===
fig2, ax3 = plt.subplots(figsize=(16, 6))

# 사이클1 포인트별 상세 데이터
points = ['P1', 'P2', 'P3', 'PC1', 'P4', 'P5', 'P6', 'Det', 'P6\'', 'P5\'', 'P4\'', 'PC2', 'P3\'', 'P2\'', 'P1\'']
xy_move =  [1.8, 2.6, 2.4, 0, 0.6, 2.3, 2.4, 0, 0.3, 2.4, 2.4, 0, 1.7, 2.6, 2.6]
z_down =   [0.2, 0.2, 0.2, 0, 0.2, 0.2, 0.2, 0, 0.2, 0.2, 0.2, 0, 0.2, 0.2, 0.2]
trigger =  [0.9, 0.9, 0.9, 0, 0.9, 0.9, 0.9, 0, 0.9, 0.9, 0.9, 0, 0.9, 0.9, 0.9]
z_up =     [0.2, 0.2, 0.2, 0, 0.2, 0.2, 0.2, 0, 0.2, 0.2, 0.2, 0, 0.2, 0.2, 0.2]
overhead = [0.1, 0.1, 0.1, 6.1, 0.1, 0.1, 0.1, 0.9, 0.1, 0.1, 0.1, 5.8, 0.1, 0.1, 0.1]

x_pos = np.arange(len(points))
bar_w = 0.6

colors2 = ['#2196F3', '#FF9800', '#F44336', '#FF9800', '#9E9E9E']
labels2 = ['XY Move', 'Z Down', 'Trigger', 'Z Up', 'Pose Change / Overhead']

b = np.zeros(len(points))
for d, c, l in zip([xy_move, z_down, trigger, z_up, overhead], colors2, labels2):
    ax3.bar(x_pos, d, bar_w, bottom=b, color=c, label=l, edgecolor='white', linewidth=0.3)
    b += np.array(d)

# 포인트 총 시간 표시
for i, total in enumerate(b):
    if total > 0.5:
        ax3.text(x_pos[i], total + 0.1, f'{total:.1f}s', ha='center', va='bottom', fontsize=8, fontweight='bold')

# 전진/복귀 구분선
ax3.axvline(x=2.7, color='gray', linestyle=':', alpha=0.5)
ax3.axvline(x=6.7, color='gray', linestyle=':', alpha=0.5)
ax3.axvline(x=10.7, color='gray', linestyle=':', alpha=0.5)

ax3.text(1, -0.8, 'Forward: Right', ha='center', fontsize=10, color='#2196F3', fontweight='bold')
ax3.text(5, -0.8, 'Forward: Left', ha='center', fontsize=10, color='#03A9F4', fontweight='bold')
ax3.text(8.5, -0.8, 'Return: Left', ha='center', fontsize=10, color='#00BCD4', fontweight='bold')
ax3.text(13, -0.8, 'Return: Right', ha='center', fontsize=10, color='#3F51B5', fontweight='bold')

ax3.set_xlabel('Action Sequence', fontsize=12)
ax3.set_ylabel('Time (seconds)', fontsize=12)
ax3.set_title('Per-Point Time Breakdown (Cycle 1, 100% Speed)', fontsize=14, fontweight='bold')
ax3.set_xticks(x_pos)
ax3.set_xticklabels(points, fontsize=9)
ax3.legend(loc='upper right', fontsize=9)
ax3.grid(axis='y', alpha=0.3)
ax3.set_ylim(-1.2, 8)

plt.tight_layout()
plt.savefig(os.path.join(IMG_DIR, 'tying_cycle_detail_report.png'), dpi=150, bbox_inches='tight',
            facecolor='white', edgecolor='none')
plt.close()

print('보고서 이미지 생성 완료:')
print('  1. tying_cycle_analysis_report.png  (3회 사이클 분석 + 평균 분포)')
print('  2. tying_cycle_detail_report.png    (포인트별 상세 타이밍)')

# === 터미널 데이터 출력 ===
print('\n' + '='*70)
print('  12-Point Tying Cycle Analysis (3 Trials, 100% Speed)')
print('='*70)

print('\n[ 사이클별 구간 시간 (초) ]')
print(f'{"구간":<28} {"Cycle 1":>8} {"Cycle 2":>8} {"Cycle 3":>8} {"평균":>8}')
print('-'*62)
section_data = [
    ('초기 검출', detection_1),
    ('Forward Right P1~P3', forward_right),
    ('자세변경1 (R→L)', pose_change_1),
    ('Forward Left P4~P6', forward_left),
    ('재검출', re_detection),
    ('Return Left P6\'~P4\'', return_left),
    ('자세변경2 (L→R)', pose_change_2),
    ('Return Right P3\'~P1\'', return_right),
]
for name, vals in section_data:
    avg = np.mean(vals)
    print(f'{name:<28} {vals[0]:>8.1f} {vals[1]:>8.1f} {vals[2]:>8.1f} {avg:>8.1f}')
print('-'*62)
for i, label in enumerate(cycles):
    print(f'{"합계 " + label:<28} {total_times[i]:>8.1f}')
print(f'{"평균":<28} {"":>8} {"":>8} {"":>8} {np.mean(total_times):>8.1f}')

print('\n[ 포인트당 상세 타이밍 - Cycle 1 ]')
print(f'{"포인트":<8} {"XY이동":>7} {"Z하강":>6} {"트리거":>7} {"Z상승":>6} {"오버헤드":>9} {"합계":>7}')
print('-'*52)
for i, pt in enumerate(points):
    total = xy_move[i] + z_down[i] + trigger[i] + z_up[i] + overhead[i]
    print(f'{pt:<8} {xy_move[i]:>7.1f} {z_down[i]:>6.1f} {trigger[i]:>7.1f} {z_up[i]:>6.1f} {overhead[i]:>9.1f} {total:>7.1f}')

print('\n[ 시간 비중 (평균) ]')
avg_tying = np.mean(forward_right) + np.mean(forward_left) + np.mean(return_left) + np.mean(return_right)
avg_pose = np.mean(pose_change_1) + np.mean(pose_change_2)
avg_det = np.mean(detection_1) + np.mean(re_detection)
avg_total = np.mean(total_times)
print(f'  결속 작업:  {avg_tying:.1f}초 ({avg_tying/avg_total*100:.1f}%)')
print(f'  자세변경:   {avg_pose:.1f}초 ({avg_pose/avg_total*100:.1f}%)')
print(f'  검출:       {avg_det:.1f}초 ({avg_det/avg_total*100:.1f}%)')
print(f'  포인트당:   {avg_total/12:.2f}초')
print('='*70)
