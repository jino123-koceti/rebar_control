"""
environment_cnn.py
gymnasium 기반 RebarGridCNNEnv
철근결속 자동화 로봇의 커버리지 경로 생성 환경
"""

from typing import List, Optional, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from state_encoder import grid_to_cnn_input

# 액션 정의 (TIE 제거 — 상위 제어기 담당)
MOVE_X_POS = 0  # X+1 (전진 600mm)
MOVE_X_NEG = 1  # X-1 (후진 600mm)
MOVE_Y_POS = 2  # Y+1 (횡이동 +400mm)
MOVE_Y_NEG = 3  # Y-1 (횡이동 -400mm)

FIXED_SIZE = 32
N_CHANNELS = 8  # +1: frontier 채널 추가 (tied 교체), y_axis_gradient 추가
AUX_DIM = 4


class RebarGridCNNEnv(gym.Env):
    """
    철근 결속 커버리지 환경.
    - 가변 그리드 크기 지원 (reset 시 지정)
    - 7채널 CNN 관측 + 4개 보조 스칼라
    - Discrete(5) 액션 공간
    """

    metadata = {'render_modes': ['human', 'rgb_array']}

    observation_space = spaces.Box(
        low=0, high=1,
        shape=(N_CHANNELS, FIXED_SIZE, FIXED_SIZE),
        dtype=np.float32,
    )
    action_space = spaces.Discrete(4)  # TIE 제거: X+, X-, Y+, Y-

    def __init__(self, render_mode=None):
        super().__init__()
        self.render_mode = render_mode

        # 환경 상태 (reset에서 초기화)
        self.grid_rows = 4
        self.grid_cols = 5
        self.robot_pos = (0, 0)  # (col, row)

        self.visited = None
        self.tied = None
        self.obstacles = None
        self.valid_area = None

        self.step_count = 0
        self.max_steps = 0

        self.path_history = []  # [(col, row), ...]

        # 커버리지 마일스톤 추적
        self._milestone_25 = False
        self._milestone_50 = False
        self._milestone_75 = False


    # ------------------------------------------------------------------
    # reset
    # ------------------------------------------------------------------
    def reset(
        self,
        grid_rows: int = 4,
        grid_cols: int = 5,
        obstacles: Optional[List[Tuple[int, int]]] = None,  # [(col, row), ...]
        start_pos: Tuple[int, int] = (0, 0),                # (col, row)
        seed: Optional[int] = None,
        options=None,
    ) -> Tuple[np.ndarray, dict]:
        super().reset(seed=seed)

        self.grid_rows = grid_rows
        self.grid_cols = grid_cols
        self.robot_pos = start_pos
        self.step_count = 0
        self.max_steps = grid_cols * grid_rows * 3  # 7th: ×4→×3 (실패 에피소드 배회 억제)
        self.path_history = [start_pos]

        # 그리드 초기화
        self.visited = np.zeros((grid_rows, grid_cols), dtype=bool)
        self.tied = np.zeros((grid_rows, grid_cols), dtype=bool)
        self.obstacles = np.zeros((grid_rows, grid_cols), dtype=bool)
        self.valid_area = np.ones((grid_rows, grid_cols), dtype=bool)

        # 장애물 설정
        if obstacles:
            for (c, r) in obstacles:
                if 0 <= r < grid_rows and 0 <= c < grid_cols:
                    self.obstacles[r, c] = True
                    self.valid_area[r, c] = False

        # 시작 위치 방문/커버 처리
        col, row = start_pos
        if 0 <= row < grid_rows and 0 <= col < grid_cols:
            self.visited[row, col] = True
            self.tied[row, col] = True  # visited == tied (TIE 제거로 자동 커버)

        # 마일스톤 초기화
        self._milestone_25 = False
        self._milestone_50 = False
        self._milestone_75 = False

        obs = self._get_obs()
        info = self._get_info()
        return obs, info

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------
    def _move(self, new_col: int, new_row: int) -> float:
        if not (0 <= new_row < self.grid_rows and 0 <= new_col < self.grid_cols):
            return -5.0  # boundary_collision

        if self.obstacles[new_row, new_col]:
            return -5.0  # obstacle_collision

        old_col, _ = self.robot_pos
        is_x = (new_col != old_col)  # col이 바뀌면 X 이동, row가 바뀌면 Y 이동
        self.robot_pos = (new_col, new_row)
        self.path_history.append(self.robot_pos)

        move_cost = -0.5 if is_x else -2.0  # 8th: X(전후진)=-0.5, Y(횡이동)=-2.0 (4배)
        # 로봇 실제 특성 반영: 전후진=무한궤도 빠름, 횡이동=모터1회전/느림 → serpentine 유도

        if not self.visited[new_row, new_col]:
            self.visited[new_row, new_col] = True
            self.tied[new_row, new_col] = True  # 방문 즉시 커버 (TIE 제거)
            return move_cost + 3.0  # new_cell_visit
        else:
            return move_cost - 1.5  # revisit 패널티 강화 (왕복 억제)

    def _check_milestones(self) -> float:
        ratio = self._coverage_ratio()
        bonus = 0.0
        if not self._milestone_25 and ratio >= 0.25:
            self._milestone_25 = True
            bonus += 10.0
        if not self._milestone_50 and ratio >= 0.50:
            self._milestone_50 = True
            bonus += 20.0
        if not self._milestone_75 and ratio >= 0.75:
            self._milestone_75 = True
            bonus += 30.0
        return bonus

    def _is_done(self) -> bool:
        valid_cells = int(self.valid_area.sum())
        tied_cells = int(self.tied.sum())
        if valid_cells == 0:
            return True
        if tied_cells >= valid_cells:
            return True  # mission_complete → 보너스는 info에서 처리

        # mission_complete 보너스는 terminated 직후 reward에 반영 안됨
        # → step() 반환 전에 check하여 reward에 추가하는 방식으로 수정 필요시 별도 처리
        return False

    def _coverage_ratio(self) -> float:
        valid_cells = int(self.valid_area.sum())
        if valid_cells == 0:
            return 1.0
        tied_cells = int(self.tied.sum())
        return tied_cells / valid_cells

    def _total_valid_cells(self) -> int:
        return int(self.valid_area.sum())

    def _get_obs(self) -> np.ndarray:
        tensor = grid_to_cnn_input(
            grid_rows=self.grid_rows,
            grid_cols=self.grid_cols,
            robot_pos=self.robot_pos,
            visited=self.visited,
            tied=self.tied,
            obstacles=self.obstacles,
            valid_area=self.valid_area,
            fixed_size=FIXED_SIZE,
        )
        return tensor.numpy()

    def get_aux_input(self) -> np.ndarray:
        """보조 입력 벡터 (4,) 반환"""
        valid_cells = self._total_valid_cells()
        tied_cells = int(self.tied.sum())

        coverage_ratio = tied_cells / valid_cells if valid_cells > 0 else 1.0
        step_ratio = self.step_count / max(self.max_steps, 1)
        col, row = self.robot_pos
        x_progress = col / max(self.grid_cols - 1, 1)
        y_progress = row / max(self.grid_rows - 1, 1)

        return np.array([coverage_ratio, step_ratio, x_progress, y_progress],
                        dtype=np.float32)

    def _get_info(self) -> dict:
        return {
            'completion_rate': self._coverage_ratio(),
            'step_count': self.step_count,
            'path': list(self.path_history),
            'tied_cells': int(self.tied.sum()),
            'valid_cells': self._total_valid_cells(),
        }

    def extract_path(self) -> List[Tuple[int, int]]:
        """에피소드 동안의 이동 경로를 (col, row) 리스트로 반환"""
        return list(self.path_history)

    # ------------------------------------------------------------------
    # step
    # ------------------------------------------------------------------
    def step(self, action: int) -> Tuple[np.ndarray, float, bool, bool, dict]:
        """
        action: 0=X+, 1=X-, 2=Y+, 3=Y-
        returns: obs, reward, terminated, truncated, info
        """
        self.step_count += 1
        col, row = self.robot_pos
        reward = 0.0
        prev_covered = int(self.visited.sum())  # coverage shaping 기준점

        if action == MOVE_X_POS:
            reward += self._move(col + 1, row)
        elif action == MOVE_X_NEG:
            reward += self._move(col - 1, row)
        elif action == MOVE_Y_POS:
            reward += self._move(col, row + 1)
        elif action == MOVE_Y_NEG:
            reward += self._move(col, row - 1)

        # Dense coverage shaping: 새 셀 방문 시 추가 보상
        new_covered = int(self.visited.sum())
        valid_cells = self._total_valid_cells()
        if valid_cells > 0 and new_covered > prev_covered:
            reward += (new_covered - prev_covered) / valid_cells * 50.0

        reward += -0.05  # step_penalty 완화 (탐색 여유)
        reward += self._check_milestones()

        terminated = self._is_done()
        if terminated:
            reward += 100.0  # mission_complete

        truncated = self.step_count >= self.max_steps

        obs = self._get_obs()
        info = self._get_info()
        return obs, reward, terminated, truncated, info

    def render(self):
        if self.render_mode == 'human':
            self._render_text()
        elif self.render_mode == 'rgb_array':
            return self._render_rgb()

    def _render_text(self):
        col, row = self.robot_pos
        print(f"Step: {self.step_count} | Pos: ({col},{row}) | "
              f"Coverage: {self._coverage_ratio():.1%}")
        for r in range(self.grid_rows):
            line = ''
            for c in range(self.grid_cols):
                if not self.valid_area[r, c]:
                    line += '[X]'
                elif r == row and c == col:
                    line += '[R]'
                elif self.tied[r, c]:
                    line += '[T]'
                elif self.visited[r, c]:
                    line += '[v]'
                else:
                    line += '[ ]'
            print(line)
        print()

    def _render_rgb(self) -> np.ndarray:
        h, w = self.grid_rows * 40, self.grid_cols * 40
        img = np.ones((h, w, 3), dtype=np.uint8) * 200
        col, row = self.robot_pos
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                y1, y2 = r * 40, (r + 1) * 40
                x1, x2 = c * 40, (c + 1) * 40
                if not self.valid_area[r, c]:
                    img[y1:y2, x1:x2] = [80, 80, 80]
                elif self.tied[r, c]:
                    img[y1:y2, x1:x2] = [100, 200, 100]
                elif self.visited[r, c]:
                    img[y1:y2, x1:x2] = [150, 200, 255]
                if r == row and c == col:
                    img[y1+10:y2-10, x1+10:x2-10] = [255, 50, 50]
        return img
