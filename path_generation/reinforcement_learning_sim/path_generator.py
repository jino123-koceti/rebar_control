#!/usr/bin/env python3
"""
RL Path Generator - 학습된 CNN 모델로 경로 생성 (추론 전용)

사용법:
    generator = RLPathGenerator('models/cnn_ppo_agent.pth')
    waypoints = generator.generate(work_area, obstacles_mm)

    # work_area: navigator의 work_area dict
    # obstacles_mm: [{'x': mm, 'y': mm}, ...] 또는 None
    # waypoints: [{'x': mm, 'y': mm}, ...]
"""

import sys
import os
import math
import numpy as np
import torch
from typing import List, Dict, Tuple, Optional

# 같은 디렉토리의 모듈 import
_dir = os.path.dirname(os.path.abspath(__file__))
if _dir not in sys.path:
    sys.path.insert(0, _dir)

from state_encoder import grid_to_cnn_input, get_aux_input
from agents_cnn import (
    CNNPPOAgent, CNNDQNAgent, CNNFeatureExtractor,
    ACTION_MOVE_X_POS, ACTION_MOVE_X_NEG,
    ACTION_MOVE_Y_POS, ACTION_MOVE_Y_NEG,
    ACTION_TIE, N_ACTIONS,
)

X_STEP_MM = 600.0
Y_STEP_MM = 396.0  # 9회전 x 44mm/회전 (실측)


class RLPathGenerator:
    """학습된 CNN 모델로 커버리지 경로 생성"""

    def __init__(self, model_path: str, algorithm: str = 'ppo', device: str = 'cpu'):
        """
        Args:
            model_path: .pth 파일 경로
            algorithm: 'ppo' 또는 'dqn'
            device: 'cpu' 또는 'cuda'
        """
        self.device = device
        self.algorithm = algorithm
        self.model_path = model_path
        self.agent = None

        if os.path.exists(model_path):
            self._load_model(model_path, algorithm)

    def _load_model(self, path: str, algorithm: str):
        """모델 로드"""
        if algorithm == 'ppo':
            self.agent = CNNPPOAgent(device=self.device)
            self.agent.load(path)
            self.agent.feature_extractor.eval()
            self.agent.policy_head.eval()
            self.agent.value_head.eval()
        else:
            self.agent = CNNDQNAgent(device=self.device)
            self.agent.load(path)
            self.agent.feature_extractor.eval()
            self.agent.q_head.eval()

    def generate(
        self,
        work_area: dict,
        obstacles_mm: Optional[List[Dict]] = None,
    ) -> List[Dict]:
        """
        작업영역에 대한 커버리지 경로 생성 (결정론적)

        직선 영역(type='line'): X축 직선 경로 (600mm 간격)
        직사각형 영역: 지그재그 경로 (전/후진 600mm, 횡이동 400mm)

        Args:
            work_area: navigator work_area dict
                       (min_x, max_x, min_y, max_y, width, height, type)
            obstacles_mm: [{'x': mm, 'y': mm}, ...] 장애물 좌표 리스트

        Returns:
            [{'x': mm, 'y': mm}, ...] 웨이포인트 리스트 (mm 단위)
        """
        area_type = work_area.get('type', 'rectangle')
        if area_type == 'line':
            return self._generate_line(work_area)
        return self._generate_zigzag(work_area, obstacles_mm)

    def _generate_line(self, work_area: dict) -> List[Dict]:
        """
        직선 영역: X축 600mm 간격 웨이포인트 생성

        Y는 영역 중심(center_y)으로 고정, X는 min_x부터 max_x까지 X_STEP_MM 간격
        """
        min_x = work_area['min_x']
        max_x = work_area['max_x']
        width = work_area['width']
        center_y = (work_area['min_y'] + work_area['max_y']) / 2.0

        waypoints = []
        grid_cols = max(1, math.ceil(width / X_STEP_MM))

        for col in range(grid_cols):
            x = min_x + col * X_STEP_MM
            if x > max_x:
                x = max_x
            waypoints.append({'x': x, 'y': center_y})

        # 마지막 포인트가 max_x에 못 미치면 추가
        if waypoints and waypoints[-1]['x'] < max_x - 1.0:
            waypoints.append({'x': max_x, 'y': center_y})

        return waypoints

    def _generate_zigzag(
        self,
        work_area: dict,
        obstacles_mm: Optional[List[Dict]] = None,
    ) -> List[Dict]:
        """
        모델 없을 때 fallback: 결정론적 지그재그 경로

        X축 우선 이동 (전/후진) → Y축 횡이동 400mm → 반대 방향 X축 이동 → 반복
        """
        min_x = work_area['min_x']
        min_y = work_area['min_y']
        width = work_area['width']
        height = work_area['height']

        grid_cols = max(1, math.ceil(width / X_STEP_MM))
        grid_rows = max(1, math.ceil(height / Y_STEP_MM))

        # 장애물 셋
        obstacle_grid = set()
        if obstacles_mm:
            for obs in obstacles_mm:
                col = int((obs['x'] - min_x) / X_STEP_MM)
                row = int((obs['y'] - min_y) / Y_STEP_MM)
                col = max(0, min(col, grid_cols - 1))
                row = max(0, min(row, grid_rows - 1))
                obstacle_grid.add((col, row))

        waypoints = []
        forward = True  # X 진행 방향

        for row in range(grid_rows):
            cols_range = range(grid_cols) if forward else range(grid_cols - 1, -1, -1)

            for col in cols_range:
                if (col, row) in obstacle_grid:
                    continue
                waypoints.append({
                    'x': min_x + col * X_STEP_MM,
                    'y': min_y + row * Y_STEP_MM,
                })

            forward = not forward

        return waypoints


def main():
    """테스트: 결정론적 경로 생성 확인"""
    generator = RLPathGenerator('models/cnn_ppo_agent.pth')

    # 직사각형 영역 테스트
    work_area_rect = {
        'min_x': 0.0, 'max_x': 3000.0,
        'min_y': 0.0, 'max_y': 1600.0,
        'width': 3000.0, 'height': 1600.0,
        'type': 'rectangle',
    }

    waypoints = generator.generate(work_area_rect)
    print(f"[Rectangle] 웨이포인트: {len(waypoints)}개")
    print(f"  그리드: {math.ceil(3000/600)}x{math.ceil(1600/400)} = "
          f"{math.ceil(3000/600) * math.ceil(1600/400)} 셀")
    for i, wp in enumerate(waypoints):
        print(f"  [{i:2d}] x={wp['x']:7.1f}, y={wp['y']:7.1f} mm")

    # 직선 영역 테스트
    print()
    work_area_line = {
        'min_x': 0.0, 'max_x': 2500.0,
        'min_y': 800.0, 'max_y': 810.0,
        'width': 2500.0, 'height': 10.0,
        'type': 'line',
    }

    waypoints_line = generator.generate(work_area_line)
    print(f"[Line] 웨이포인트: {len(waypoints_line)}개")
    for i, wp in enumerate(waypoints_line):
        print(f"  [{i:2d}] x={wp['x']:7.1f}, y={wp['y']:7.1f} mm")


if __name__ == '__main__':
    main()
