#!/usr/bin/env python3
"""
1200도 회전 속도 분석 그래프 생성 스크립트
X축: 시간 (초)
Y축: 거리 (mm)
"""

import csv
from collections import defaultdict

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False
    print("plotly가 설치되어 있지 않습니다. matplotlib를 시도합니다...")
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        MATPLOTLIB_AVAILABLE = True
    except ImportError:
        MATPLOTLIB_AVAILABLE = False
        print("그래프 라이브러리를 사용할 수 없습니다.")

def load_data(csv_file):
    """CSV 파일에서 데이터 로드"""
    data = defaultdict(lambda: {'left': {'times': [], 'positions_mm': [], 'velocity': None},
                                 'right': {'times': [], 'positions_mm': [], 'velocity': None}})
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rotation_id = int(row['Rotation_ID'])
            side = row['Side']
            time_sec = float(row['Time_sec'])
            position_mm = float(row['Position_mm'])
            velocity = float(row['Velocity_mm_per_sec'])
            
            data[rotation_id][side]['times'].append(time_sec)
            data[rotation_id][side]['positions_mm'].append(position_mm)
            data[rotation_id][side]['velocity'] = velocity
    
    return data

def create_plotly_plot(data, output_file):
    """plotly를 사용하여 그래프 생성"""
    fig = make_subplots(
        rows=2, cols=1,
        subplot_titles=('좌측 모터 (0x141) - 1200도 회전 (600mm 이동)', '우측 모터 (0x142) - 1200도 회전 (600mm 이동)'),
        vertical_spacing=0.1
    )
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
              '#aec7e8', '#ffbb78', '#98df8a']
    
    for rotation_id in sorted(data.keys()):
        color = colors[(rotation_id - 1) % len(colors)]
        
        # 좌측 모터
        left = data[rotation_id]['left']
        if len(left['times']) > 0:
            velocity_text = f"{left['velocity']:.1f}mm/s" if left['velocity'] else ""
            fig.add_trace(
                go.Scatter(
                    x=left['times'],
                    y=left['positions_mm'],
                    mode='lines+markers',
                    name=f'ID={rotation_id} ({velocity_text})',
                    line=dict(color=color, width=2),
                    marker=dict(size=4),
                    legendgroup=f'id{rotation_id}',
                    showlegend=True
                ),
                row=1, col=1
            )
        
        # 우측 모터
        right = data[rotation_id]['right']
        if len(right['times']) > 0:
            velocity_text = f"{right['velocity']:.1f}mm/s" if right['velocity'] else ""
            fig.add_trace(
                go.Scatter(
                    x=right['times'],
                    y=right['positions_mm'],
                    mode='lines+markers',
                    name=f'ID={rotation_id} ({velocity_text})',
                    line=dict(color=color, width=2),
                    marker=dict(size=4),
                    legendgroup=f'id{rotation_id}',
                    showlegend=False
                ),
                row=2, col=1
            )
    
    fig.update_xaxes(title_text="시간 (초)", row=1, col=1)
    fig.update_xaxes(title_text="시간 (초)", row=2, col=1)
    fig.update_yaxes(title_text="거리 (mm)", row=1, col=1)
    fig.update_yaxes(title_text="거리 (mm)", row=2, col=1)
    
    fig.update_layout(
        title_text='1200도 회전 속도 분석 (시간-거리 그래프) - 600mm 이동',
        title_x=0.5,
        height=1000,
        showlegend=True
    )
    
    fig.write_html(output_file)
    print(f"그래프 저장 완료: {output_file} (HTML 형식)")

def create_matplotlib_plot(data, output_file):
    """matplotlib를 사용하여 그래프 생성"""
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle('1200도 회전 속도 분석 (시간-거리 그래프) - 600mm 이동', fontsize=16, fontweight='bold')
    
    colors = plt.cm.tab20(np.linspace(0, 1, 20))
    
    for rotation_id in sorted(data.keys()):
        color = colors[(rotation_id - 1) % len(colors)]
        
        # 좌측 모터
        left = data[rotation_id]['left']
        if len(left['times']) > 0:
            velocity_text = f"{left['velocity']:.1f}mm/s" if left['velocity'] else ""
            axes[0].plot(left['times'], left['positions_mm'], 'o-', 
                        label=f'ID={rotation_id} ({velocity_text})',
                        color=color, markersize=4, linewidth=2, alpha=0.7)
        
        # 우측 모터
        right = data[rotation_id]['right']
        if len(right['times']) > 0:
            velocity_text = f"{right['velocity']:.1f}mm/s" if right['velocity'] else ""
            axes[1].plot(right['times'], right['positions_mm'], 'o-',
                        label=f'ID={rotation_id} ({velocity_text})',
                        color=color, markersize=4, linewidth=2, alpha=0.7)
    
    axes[0].set_title('좌측 모터 (0x141) - 1200도 회전 (600mm 이동)', fontsize=12, fontweight='bold')
    axes[0].set_xlabel('시간 (초)', fontsize=11)
    axes[0].set_ylabel('거리 (mm)', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best', fontsize=8)
    
    axes[1].set_title('우측 모터 (0x142) - 1200도 회전 (600mm 이동)', fontsize=12, fontweight='bold')
    axes[1].set_xlabel('시간 (초)', fontsize=11)
    axes[1].set_ylabel('거리 (mm)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"그래프 저장 완료: {output_file} (PNG 형식)")

def main():
    csv_file = '/tmp/rotation_1200deg_data.csv'
    output_file = '/tmp/rotation_1200deg_speed_analysis'
    
    # 데이터 로드
    data = load_data(csv_file)
    
    if not data:
        print("데이터를 찾을 수 없습니다.")
        return
    
    # 통계 출력
    print("\n1200도 회전 (600mm 이동) 속도 통계:")
    print("=" * 60)
    velocities = []
    for rotation_id in sorted(data.keys()):
        left = data[rotation_id]['left']
        if left['velocity']:
            velocities.append(left['velocity'])
            print(f"ID={rotation_id:2d}: {left['velocity']:6.2f} mm/sec")
    
    if velocities:
        print("-" * 60)
        print(f"평균 속도: {sum(velocities)/len(velocities):.2f} mm/sec")
        print(f"최소 속도: {min(velocities):.2f} mm/sec")
        print(f"최대 속도: {max(velocities):.2f} mm/sec")
        print("=" * 60)
    
    # 그래프 생성
    if PLOTLY_AVAILABLE:
        create_plotly_plot(data, output_file + '.html')
    elif MATPLOTLIB_AVAILABLE:
        create_matplotlib_plot(data, output_file + '.png')
    else:
        print("그래프 라이브러리를 사용할 수 없습니다.")
        print("CSV 파일을 사용하여 다른 도구로 그래프를 그릴 수 있습니다:", csv_file)

if __name__ == '__main__':
    main()



