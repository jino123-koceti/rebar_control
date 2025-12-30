#!/usr/bin/env python3
"""
Performance Test Results Visualization
CSV 파일을 읽어 matplotlib으로 시각화

사용법:
    python3 visualize_results.py <csv_file>

출력:
    - 위치 궤적 (X-Y Plot)
    - 오차 시계열
    - 오차 히스토그램
    - 전진 vs 후진 박스플롯
    - PNG 파일로 저장
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os


def visualize_results(csv_file):
    """결과 시각화"""
    if not os.path.exists(csv_file):
        print(f"오류: 파일을 찾을 수 없습니다: {csv_file}")
        sys.exit(1)

    # CSV 읽기
    df = pd.read_csv(csv_file)
    print(f"데이터 로드: {len(df)} 개 측정값")

    # Figure 생성
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Performance Test Results: {os.path.basename(csv_file)}', fontsize=16, fontweight='bold')

    # 1. 위치 궤적 (X-Y Plot)
    ax1 = axes[0, 0]
    forward_df = df[df['phase'] == 'forward']
    backward_df = df[df['phase'] == 'backward']

    ax1.scatter(forward_df['actual_x'], forward_df['actual_y'],
                c='blue', marker='o', s=50, label='Forward', alpha=0.7, edgecolors='darkblue')
    ax1.scatter(backward_df['actual_x'], backward_df['actual_y'],
                c='red', marker='x', s=50, label='Backward', alpha=0.7)

    # 목표 위치 표시
    ax1.axvline(x=0, color='green', linestyle='--', linewidth=2, alpha=0.6, label='Origin (0,0)')
    if len(forward_df) > 0:
        target_x = forward_df['target_x'].iloc[0]
        ax1.axvline(x=target_x, color='orange', linestyle='--', linewidth=2, alpha=0.6,
                    label=f'Target ({target_x:.3f}m)')

    ax1.set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
    ax1.set_title('Position Trajectory', fontsize=12, fontweight='bold')
    ax1.legend(loc='best', fontsize=9)
    ax1.grid(True, alpha=0.3, linestyle=':')
    ax1.axis('equal')

    # 2. 오차 시계열
    ax2 = axes[0, 1]
    errors_mm = df['error'] * 1000
    colors = ['blue' if p == 'forward' else 'red' for p in df['phase']]

    ax2.scatter(range(len(df)), errors_mm, c=colors, s=40, alpha=0.7, edgecolors='black', linewidth=0.5)
    ax2.axhline(y=errors_mm.mean(), color='black', linestyle='--', linewidth=2,
                label=f'Mean: {errors_mm.mean():.2f} mm')
    ax2.axhline(y=10, color='green', linestyle=':', linewidth=1.5, alpha=0.7,
                label='Target: ±10 mm')

    ax2.set_xlabel('Test Index', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Position Error (mm)', fontsize=11, fontweight='bold')
    ax2.set_title('Error over Time', fontsize=12, fontweight='bold')
    ax2.legend(loc='best', fontsize=9)
    ax2.grid(True, alpha=0.3, linestyle=':')

    # 3. 오차 히스토그램
    ax3 = axes[1, 0]
    n, bins, patches = ax3.hist(errors_mm, bins=20, color='skyblue', edgecolor='black', alpha=0.7)

    # 평균, 표준편차 표시
    ax3.axvline(x=errors_mm.mean(), color='red', linestyle='--', linewidth=2,
                label=f'Mean: {errors_mm.mean():.2f} mm')
    ax3.axvline(x=errors_mm.mean() + errors_mm.std(), color='orange', linestyle=':', linewidth=1.5,
                label=f'+1σ: {errors_mm.std():.2f} mm')
    ax3.axvline(x=errors_mm.mean() - errors_mm.std(), color='orange', linestyle=':', linewidth=1.5)

    # 목표 범위 표시
    ax3.axvline(x=10, color='green', linestyle=':', linewidth=1.5, alpha=0.7,
                label='Target: ±10 mm')

    ax3.set_xlabel('Position Error (mm)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Frequency', fontsize=11, fontweight='bold')
    ax3.set_title('Error Distribution', fontsize=12, fontweight='bold')
    ax3.legend(loc='best', fontsize=9)
    ax3.grid(True, alpha=0.3, linestyle=':', axis='y')

    # 4. 전진 vs 후진 박스플롯
    ax4 = axes[1, 1]
    forward_errors = forward_df['error'] * 1000
    backward_errors = backward_df['error'] * 1000

    bp = ax4.boxplot([forward_errors, backward_errors],
                      labels=['Forward', 'Backward'],
                      patch_artist=True,
                      widths=0.6,
                      boxprops=dict(facecolor='lightblue', alpha=0.7, linewidth=1.5),
                      medianprops=dict(color='red', linewidth=2),
                      whiskerprops=dict(linewidth=1.5),
                      capprops=dict(linewidth=1.5))

    # 목표 범위 표시
    ax4.axhline(y=10, color='green', linestyle=':', linewidth=1.5, alpha=0.7,
                label='Target: ±10 mm')

    ax4.set_ylabel('Position Error (mm)', fontsize=11, fontweight='bold')
    ax4.set_title('Forward vs Backward Error', fontsize=12, fontweight='bold')
    ax4.legend(loc='best', fontsize=9)
    ax4.grid(True, alpha=0.3, linestyle=':', axis='y')

    # 통계 텍스트 추가
    stats_text = f"""Statistics Summary:
━━━━━━━━━━━━━━━━━━━━━━━━━━
Total Tests:  {len(df)}
Mean Error:   {errors_mm.mean():.2f} mm
Std Dev:      {errors_mm.std():.2f} mm
Max Error:    {errors_mm.max():.2f} mm
Min Error:    {errors_mm.min():.2f} mm
━━━━━━━━━━━━━━━━━━━━━━━━━━
Forward:      {forward_errors.mean():.2f} ± {forward_errors.std():.2f} mm
Backward:     {backward_errors.mean():.2f} ± {backward_errors.std():.2f} mm
━━━━━━━━━━━━━━━━━━━━━━━━━━
Target:       ± 10 mm
Status:       {'✅ PASS' if errors_mm.mean() <= 10 else '❌ FAIL'}
"""

    fig.text(0.02, 0.02, stats_text, fontsize=9, family='monospace',
             verticalalignment='bottom',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8, pad=0.8))

    plt.tight_layout(rect=[0, 0.15, 1, 0.96])

    # 저장 및 표시
    output_png = csv_file.replace('.csv', '.png')
    plt.savefig(output_png, dpi=200, bbox_inches='tight')
    print(f"✅ 그래프 저장: {output_png}")

    print("\n통계 요약:")
    print("=" * 50)
    print(f"평균 오차:   {errors_mm.mean():.2f} mm")
    print(f"표준편차:    {errors_mm.std():.2f} mm")
    print(f"최대 오차:   {errors_mm.max():.2f} mm")
    print(f"최소 오차:   {errors_mm.min():.2f} mm")
    print("-" * 50)
    print(f"전진 평균:   {forward_errors.mean():.2f} ± {forward_errors.std():.2f} mm")
    print(f"후진 평균:   {backward_errors.mean():.2f} ± {backward_errors.std():.2f} mm")
    print("=" * 50)

    if errors_mm.mean() <= 10:
        print("✅ 테스트 통과: 평균 오차 ≤ 10mm")
    else:
        print("❌ 테스트 실패: 평균 오차 > 10mm")

    plt.show()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("사용법: python3 visualize_results.py <csv_file>")
        print("")
        print("예시:")
        print("  python3 visualize_results.py 20251223_143052_performance_test_results.csv")
        sys.exit(1)

    csv_file = sys.argv[1]
    visualize_results(csv_file)
