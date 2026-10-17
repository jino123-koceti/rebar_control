"""
path_generator.py
추론 전용 (Jetson 배포용)
navigator.py의 PATH_GEN 커맨드를 처리하여 웨이포인트 리스트 반환
"""

import math
import os
from typing import Dict, List, Optional

import numpy as np
import torch

from agents_cnn import CNNPPOAgent, CNNDQNAgent
from environment_cnn import RebarGridCNNEnv

# 로봇 물리 상수
X_STEP_MM = 600.0   # X축(전/후진) 1스텝 = 600mm
Y_STEP_MM = 400.0   # Y축(횡이동) 1스텝 = 400mm


class RLPathGenerator:
    """
    학습된 CNN RL 모델을 이용한 경로 생성기.
    navigator.py에서 PATH_GEN 커맨드 수신 시 호출.
    CPU 추론 (Jetson 배포 최적화)
    """

    def __init__(self, model_path: str, algorithm: str = 'ppo'):
        """
        학습된 모델 로드 (CPU 추론)

        Args:
            model_path: .pth 파일 경로
            algorithm: 'ppo' 또는 'dqn'
        """
        self.model_path = model_path
        self.algorithm = algorithm
        self._env = RebarGridCNNEnv()

        if algorithm == 'ppo':
            self._agent = CNNPPOAgent()
        else:
            self._agent = CNNDQNAgent(epsilon_start=0.0)

        if not os.path.exists(model_path):
            raise FileNotFoundError(f'모델 파일을 찾을 수 없습니다: {model_path}')

        self._agent.load(model_path)
        print(f'[RLPathGenerator] 모델 로드 완료: {model_path}')

    def generate(
        self,
        work_area: dict,
        obstacles_mm: Optional[List[dict]] = None,
    ) -> List[dict]:
        """
        작업 영역 및 장애물 정보를 받아 웨이포인트 리스트 반환.

        Args:
            work_area: navigator의 work_area dict
                       예: {'min_x': 0.0, 'min_y': 0.0, 'max_x': 3000.0, 'max_y': 1600.0}
            obstacles_mm: 장애물 좌표 리스트 (mm 단위)
                          예: [{'x': 1500.0, 'y': 800.0}, ...]

        Returns:
            웨이포인트 리스트:
            [{'x': 600.0, 'y': 0.0}, {'x': 1200.0, 'y': 0.0}, ...]
            (mm 단위, TIE 액션 위치만 포함 — 이동 경로 포함 버전은 include_moves=True)
        """
        # 1. work_area → grid 크기 계산
        work_area_width = work_area['max_x'] - work_area['min_x']
        work_area_height = work_area['max_y'] - work_area['min_y']

        grid_cols = math.ceil(work_area_width / X_STEP_MM)
        grid_rows = math.ceil(work_area_height / Y_STEP_MM)

        grid_cols = max(grid_cols, 1)
        grid_rows = max(grid_rows, 1)

        # 2. obstacles_mm → grid 좌표 변환
        obstacle_grid = []
        if obstacles_mm:
            for obs in obstacles_mm:
                obs_x = obs.get('x', 0.0) - work_area['min_x']
                obs_y = obs.get('y', 0.0) - work_area['min_y']
                obs_col = int(obs_x / X_STEP_MM)
                obs_row = int(obs_y / Y_STEP_MM)
                if 0 <= obs_col < grid_cols and 0 <= obs_row < grid_rows:
                    obstacle_grid.append((obs_col, obs_row))

        # 3. 환경 초기화 및 CNN 모델 추론
        obs, _ = self._env.reset(
            grid_rows=grid_rows,
            grid_cols=grid_cols,
            obstacles=obstacle_grid,
            start_pos=(0, 0),
        )

        waypoints = []
        done = False
        max_steps = grid_cols * grid_rows * 6  # 안전 여유

        step = 0
        while not done and step < max_steps:
            aux = self._env.get_aux_input()
            action = self._agent.predict(obs, aux)
            obs, _, terminated, truncated, info = self._env.step(action)
            done = terminated or truncated
            step += 1

        # 4. grid path → mm 웨이포인트 변환
        path = self._env.extract_path()
        waypoints = self._path_to_waypoints(path, work_area)

        print(f'[RLPathGenerator] 경로 생성 완료: '
              f'{grid_rows}x{grid_cols} 그리드, '
              f'{len(waypoints)} 웨이포인트, '
              f'커버리지: {info["completion_rate"]:.1%}')

        return waypoints

    def _path_to_waypoints(
        self,
        path: List[tuple],
        work_area: dict,
    ) -> List[dict]:
        """
        grid (col, row) 경로 → mm 단위 웨이포인트 리스트 변환

        변환 공식:
          waypoint_x = work_area['min_x'] + col * X_STEP_MM
          waypoint_y = work_area['min_y'] + row * Y_STEP_MM
        """
        waypoints = []
        for col, row in path:
            wx = work_area['min_x'] + col * X_STEP_MM
            wy = work_area['min_y'] + row * Y_STEP_MM
            waypoints.append({'x': float(wx), 'y': float(wy)})
        return waypoints

    def generate_from_command(self, command: dict) -> List[dict]:
        """
        navigator.py의 PATH_GEN 커맨드 딕트를 직접 받아 처리.

        Args:
            command: {"command": "PATH_GEN"} 또는
                     {"command": "PATH_GEN", "obstacles": [{"x": 1500, "y": 800}]}

        Returns:
            웨이포인트 리스트
        """
        if not hasattr(self, '_work_area'):
            raise RuntimeError('work_area가 설정되지 않았습니다. '
                               'set_work_area()를 먼저 호출하세요.')

        obstacles = command.get('obstacles', None)
        return self.generate(self._work_area, obstacles)

    def set_work_area(self, work_area: dict):
        """navigator의 work_area 저장 (PATH_GEN 커맨드 처리 전 호출)"""
        self._work_area = work_area

    @staticmethod
    def waypoints_to_load_format(waypoints: List[dict]) -> List[dict]:
        """
        반환된 웨이포인트를 navigator의 load_waypoints_from_json() 형식으로 변환.
        navigator.py의 형식에 맞게 필요 시 수정.

        현재 가정: {'x': float, 'y': float} 형식 그대로 사용
        """
        return waypoints
