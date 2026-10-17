"""
state_encoder.py
그리드 상태 → 8채널 32x32 CNN 입력 변환

채널 구성:
  0: robot_position
  1: visited
  2: frontier        (방문 셀과 인접한 미방문 셀 — tied 채널 교체)
  3: obstacles
  4: valid_area
  5: distance_field  (미방문 셀까지 BFS 거리)
  6: x_axis_gradient (열 위치 0~1)
  7: y_axis_gradient (행 위치 0~1)  ← 신규
"""

from collections import deque
import numpy as np
import torch
import torch.nn.functional as F


def compute_distance_field(
    tied: np.ndarray,       # shape (rows, cols), bool
    valid_area: np.ndarray, # shape (rows, cols), bool
) -> np.ndarray:            # shape (rows, cols), float 0~1
    """BFS로 미결속 셀까지 정규화 거리장 계산"""
    rows, cols = tied.shape

    # 미결속 유효 셀 위치 수집
    untied_cells = []
    for r in range(rows):
        for c in range(cols):
            if valid_area[r, c] and not tied[r, c]:
                untied_cells.append((r, c))

    dist = np.full((rows, cols), float('inf'))

    if not untied_cells:
        return np.zeros((rows, cols), dtype=np.float32)

    queue = deque()
    for r, c in untied_cells:
        dist[r, c] = 0.0
        queue.append((r, c))

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    while queue:
        r, c = queue.popleft()
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and valid_area[nr, nc]:
                if dist[nr, nc] == float('inf'):
                    dist[nr, nc] = dist[r, c] + 1.0
                    queue.append((nr, nc))

    # 유효 영역 내 최대 거리로 정규화
    valid_dists = dist[valid_area & (dist != float('inf'))]
    max_dist = valid_dists.max() if len(valid_dists) > 0 else 1.0
    if max_dist == 0.0:
        max_dist = 1.0

    normalized = np.where(dist == float('inf'), 1.0, dist / max_dist)
    return normalized.astype(np.float32)


def compute_frontier(
    visited: np.ndarray,    # shape (rows, cols), bool
    valid_area: np.ndarray, # shape (rows, cols), bool
) -> np.ndarray:            # shape (rows, cols), float 0 or 1
    """미방문 셀 중 방문 셀과 인접한 셀(frontier) 계산"""
    rows, cols = visited.shape
    frontier = np.zeros((rows, cols), dtype=np.float32)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for r in range(rows):
        for c in range(cols):
            if valid_area[r, c] and not visited[r, c]:
                for dr, dc in directions:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and visited[nr, nc]:
                        frontier[r, c] = 1.0
                        break
    return frontier


def grid_to_cnn_input(
    grid_rows: int,
    grid_cols: int,
    robot_pos: tuple,           # (col, row)
    visited: np.ndarray,        # shape (rows, cols), bool
    tied: np.ndarray,           # shape (rows, cols), bool  (사용 안 함 — frontier로 교체)
    obstacles: np.ndarray,      # shape (rows, cols), bool
    valid_area: np.ndarray,     # shape (rows, cols), bool
    fixed_size: int = 32,
) -> torch.Tensor:              # shape (8, 32, 32), float32
    """가변 크기 그리드를 고정 크기 CNN 입력으로 변환 (8채널)"""

    col, row = robot_pos

    # 채널 0: robot_position
    robot_ch = np.zeros((grid_rows, grid_cols), dtype=np.float32)
    if 0 <= row < grid_rows and 0 <= col < grid_cols:
        robot_ch[row, col] = 1.0

    # 채널 1: visited
    visited_ch = visited.astype(np.float32)

    # 채널 2: frontier (방문 셀과 인접한 미방문 셀 — 구 tied 채널 교체)
    frontier_ch = compute_frontier(visited, valid_area)

    # 채널 3: obstacles
    obstacles_ch = obstacles.astype(np.float32)

    # 채널 4: valid_area
    valid_ch = valid_area.astype(np.float32)

    # 채널 5: distance_field (BFS)
    dist_ch = compute_distance_field(tied, valid_area)

    # 채널 6: x_axis_gradient (col / max_col)
    max_col = max(grid_cols - 1, 1)
    x_grad_ch = np.zeros((grid_rows, grid_cols), dtype=np.float32)
    for c in range(grid_cols):
        x_grad_ch[:, c] = c / max_col

    # 채널 7: y_axis_gradient (row / max_row)
    max_row = max(grid_rows - 1, 1)
    y_grad_ch = np.zeros((grid_rows, grid_cols), dtype=np.float32)
    for r in range(grid_rows):
        y_grad_ch[r, :] = r / max_row

    # 스택 → (8, rows, cols)
    grid_tensor = torch.from_numpy(
        np.stack([robot_ch, visited_ch, frontier_ch, obstacles_ch,
                  valid_ch, dist_ch, x_grad_ch, y_grad_ch], axis=0)
    )  # (8, rows, cols)

    # (1, 8, rows, cols)으로 배치 차원 추가
    grid_tensor = grid_tensor.unsqueeze(0)

    # 이미 fixed_size면 리사이즈 불필요
    if grid_rows == fixed_size and grid_cols == fixed_size:
        return grid_tensor.squeeze(0)

    # 채널별 interpolation 방식 분리
    # 채널 0~4 (binary): nearest
    binary_channels = grid_tensor[:, :5, :, :]
    binary_resized = F.interpolate(
        binary_channels,
        size=(fixed_size, fixed_size),
        mode='nearest',
    )

    # 채널 5~7 (float): bilinear
    float_channels = grid_tensor[:, 5:, :, :]
    float_resized = F.interpolate(
        float_channels,
        size=(fixed_size, fixed_size),
        mode='bilinear',
        align_corners=False,
    )

    result = torch.cat([binary_resized, float_resized], dim=1)
    return result.squeeze(0).to(torch.float32)  # (8, 32, 32)
