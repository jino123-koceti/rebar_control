#!/usr/bin/env python3
"""
회전 속도 분석 그래프 생성 스크립트
X축: 시간 (초)
Y축: 각도 (도)
기울기 = 속도 (mm/sec)
"""

import csv
import sys
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
        matplotlib.use('Agg')  # GUI 없이 사용
        import matplotlib.pyplot as plt
        import numpy as np
        MATPLOTLIB_AVAILABLE = True
    except ImportError:
        MATPLOTLIB_AVAILABLE = False
        print("matplotlib도 사용할 수 없습니다. 텍스트 그래프를 생성합니다.")

def load_data(csv_file):
    """CSV 파일에서 데이터 로드"""
    data = defaultdict(lambda: {'left': {'times': [], 'positions': [], 'velocity': None},
                                 'right': {'times': [], 'positions': [], 'velocity': None}})
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rotation_id = int(row['Rotation_ID'])
            side = row['Side']
            time_sec = float(row['Time_sec'])
            position_deg = float(row['Position_deg'])
            velocity = float(row['Velocity_mm_per_sec'])
            
            data[rotation_id][side]['times'].append(time_sec)
            data[rotation_id][side]['positions'].append(position_deg)
            data[rotation_id][side]['velocity'] = velocity
    
    return data

def create_plotly_plot(data, output_file):
    """plotly를 사용하여 그래프 생성"""
    fig = make_subplots(
        rows=2, cols=1,
        subplot_titles=('좌측 모터 (0x141)', '우측 모터 (0x142)'),
        vertical_spacing=0.1
    )
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
              '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    
    for rotation_id in sorted(data.keys()):
        if rotation_id > 10:
            continue
        
        color = colors[(rotation_id - 1) % len(colors)]
        
        # 좌측 모터
        left = data[rotation_id]['left']
        if len(left['times']) > 0:
            velocity_text = f"{left['velocity']:.1f}mm/s" if left['velocity'] else ""
            fig.add_trace(
                go.Scatter(
                    x=left['times'],
                    y=left['positions'],
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
                    y=right['positions'],
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
    fig.update_yaxes(title_text="각도 (도)", row=1, col=1)
    fig.update_yaxes(title_text="각도 (도)", row=2, col=1)
    
    fig.update_layout(
        title_text='360도 회전 속도 분석 (시간-각도 그래프)',
        title_x=0.5,
        height=1000,
        showlegend=True
    )
    
    fig.write_html(output_file)
    print(f"그래프 저장 완료: {output_file} (HTML 형식)")

def create_matplotlib_plot(data, output_file):
    """matplotlib를 사용하여 그래프 생성"""
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle('360도 회전 속도 분석 (시간-각도 그래프)', fontsize=16, fontweight='bold')
    
    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    
    for rotation_id in sorted(data.keys()):
        if rotation_id > 10:
            continue
        
        color = colors[(rotation_id - 1) % len(colors)]
        
        # 좌측 모터
        left = data[rotation_id]['left']
        if len(left['times']) > 0:
            velocity_text = f"{left['velocity']:.1f}mm/s" if left['velocity'] else ""
            axes[0].plot(left['times'], left['positions'], 'o-', 
                        label=f'ID={rotation_id} ({velocity_text})',
                        color=color, markersize=4, linewidth=2, alpha=0.7)
        
        # 우측 모터
        right = data[rotation_id]['right']
        if len(right['times']) > 0:
            velocity_text = f"{right['velocity']:.1f}mm/s" if right['velocity'] else ""
            axes[1].plot(right['times'], right['positions'], 'o-',
                        label=f'ID={rotation_id} ({velocity_text})',
                        color=color, markersize=4, linewidth=2, alpha=0.7)
    
    axes[0].set_title('좌측 모터 (0x141)', fontsize=12, fontweight='bold')
    axes[0].set_xlabel('시간 (초)', fontsize=11)
    axes[0].set_ylabel('각도 (도)', fontsize=11)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best', fontsize=8)
    
    axes[1].set_title('우측 모터 (0x142)', fontsize=12, fontweight='bold')
    axes[1].set_xlabel('시간 (초)', fontsize=11)
    axes[1].set_ylabel('각도 (도)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best', fontsize=8)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"그래프 저장 완료: {output_file} (PNG 형식)")

def main():
    csv_file = '/tmp/rotation_data.csv'
    output_file = '/tmp/rotation_speed_analysis'
    
    # 데이터 로드
    data = load_data(csv_file)
    
    if not data:
        print("데이터를 찾을 수 없습니다.")
        return
    
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



