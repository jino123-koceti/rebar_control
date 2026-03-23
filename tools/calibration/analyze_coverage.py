#!/usr/bin/env python3
"""
Calibration data coverage analysis for left and right cameras.
Analyzes spatial distribution of calibration points and identifies gaps.
"""

import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.patches import Rectangle
import os

# Paths
DATA_DIR = '/home/koceti/ros2_ws/data/calibration'
RIGHT_JSON = os.path.join(DATA_DIR, 'calibration_data_right.json')
LEFT_JSON = os.path.join(DATA_DIR, 'calibration_data_left.json')

# Stage specs from tying_orchestrator.yaml
STAGE_X_MAX = 411.0  # mm
STAGE_Y_MAX = 288.0  # mm

# Camera-specific Y working ranges
CAM_RANGES = {
    'right': {'x_min': 0, 'x_max': STAGE_X_MAX, 'y_min': 0.0, 'y_max': 142.0},
    'left':  {'x_min': 0, 'x_max': STAGE_X_MAX, 'y_min': 124.0, 'y_max': 288.0},
}

# Grid resolution for gap analysis
GRID_X_CELLS = 8
GRID_Y_CELLS = 6


def load_data(path):
    with open(path, 'r') as f:
        data = json.load(f)
    return data


def print_stats(name, data):
    px_u = np.array([d['pixel_u'] for d in data])
    px_v = np.array([d['pixel_v'] for d in data])
    ax = np.array([d['actual_x_mm'] for d in data])
    ay = np.array([d['actual_y_mm'] for d in data])
    depth = np.array([d['depth_mm'] for d in data])

    print(f"\n{'='*60}")
    print(f"  {name.upper()} CAMERA - {len(data)} data points")
    print(f"{'='*60}")
    print(f"  {'Field':<15} {'Min':>8} {'Max':>8} {'Mean':>8} {'Std':>8}")
    print(f"  {'-'*47}")
    for label, arr in [('pixel_u', px_u), ('pixel_v', px_v),
                        ('actual_x_mm', ax), ('actual_y_mm', ay),
                        ('depth_mm', depth)]:
        print(f"  {label:<15} {arr.min():8.1f} {arr.max():8.1f} {arr.mean():8.1f} {arr.std():8.1f}")

    return ax, ay, px_u, px_v


def grid_density(ax, ay, cam_name):
    """Compute data density per grid cell within the working range."""
    rng = CAM_RANGES[cam_name]
    x_edges = np.linspace(rng['x_min'], rng['x_max'], GRID_X_CELLS + 1)
    y_edges = np.linspace(rng['y_min'], rng['y_max'], GRID_Y_CELLS + 1)

    counts = np.zeros((GRID_Y_CELLS, GRID_X_CELLS), dtype=int)
    for x, y in zip(ax, ay):
        xi = np.searchsorted(x_edges, x, side='right') - 1
        yi = np.searchsorted(y_edges, y, side='right') - 1
        xi = np.clip(xi, 0, GRID_X_CELLS - 1)
        yi = np.clip(yi, 0, GRID_Y_CELLS - 1)
        counts[yi, xi] += 1

    print(f"\n  Grid density ({GRID_X_CELLS}x{GRID_Y_CELLS} cells):")
    print(f"  X edges: {[f'{e:.0f}' for e in x_edges]}")
    print(f"  Y edges: {[f'{e:.0f}' for e in y_edges]}")
    print(f"  Cell counts (rows=Y ascending, cols=X ascending):")
    for yi in range(GRID_Y_CELLS - 1, -1, -1):
        row_str = '  '.join(f'{counts[yi, xi]:3d}' for xi in range(GRID_X_CELLS))
        y_lo = y_edges[yi]
        y_hi = y_edges[yi + 1]
        print(f"    Y[{y_lo:5.0f}-{y_hi:5.0f}]: {row_str}")

    # Identify sparse cells
    sparse_cells = []
    for yi in range(GRID_Y_CELLS):
        for xi in range(GRID_X_CELLS):
            if counts[yi, xi] < 2:
                x_center = (x_edges[xi] + x_edges[xi + 1]) / 2
                y_center = (y_edges[yi] + y_edges[yi + 1]) / 2
                sparse_cells.append((x_center, y_center, counts[yi, xi]))

    if sparse_cells:
        print(f"\n  SPARSE CELLS (< 2 points):")
        for xc, yc, cnt in sparse_cells:
            print(f"    X~{xc:.0f}mm, Y~{yc:.0f}mm : {cnt} points")
    else:
        print(f"\n  All cells have >= 2 points. Good coverage!")

    return counts, x_edges, y_edges, sparse_cells


def plot_camera(cam_name, data, output_path):
    ax_data = np.array([d['actual_x_mm'] for d in data])
    ay_data = np.array([d['actual_y_mm'] for d in data])
    px_u = np.array([d['pixel_u'] for d in data])
    px_v = np.array([d['pixel_v'] for d in data])

    rng = CAM_RANGES[cam_name]

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(f'{cam_name.upper()} Camera Calibration Coverage ({len(data)} points)',
                 fontsize=16, fontweight='bold')

    # --- Top-left: Pixel space ---
    ax1 = axes[0, 0]
    ax1.scatter(px_u, px_v, c='steelblue', s=40, alpha=0.7, edgecolors='navy', linewidths=0.5)
    ax1.set_xlabel('pixel_u')
    ax1.set_ylabel('pixel_v')
    ax1.set_title('Pixel Space Coverage')
    ax1.invert_yaxis()
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # --- Top-right: Robot coordinate scatter ---
    ax2 = axes[0, 1]
    ax2.scatter(ax_data, ay_data, c='crimson', s=50, alpha=0.7, edgecolors='darkred', linewidths=0.5)
    # Draw working envelope
    rect = Rectangle((rng['x_min'], rng['y_min']),
                      rng['x_max'] - rng['x_min'],
                      rng['y_max'] - rng['y_min'],
                      linewidth=2, edgecolor='green', facecolor='lightgreen', alpha=0.15,
                      linestyle='--', label='Working envelope')
    ax2.add_patch(rect)
    ax2.set_xlabel('actual_x_mm')
    ax2.set_ylabel('actual_y_mm')
    ax2.set_title('Robot Coordinate Coverage')
    ax2.set_xlim(-20, STAGE_X_MAX + 20)
    ax2.set_ylim(rng['y_min'] - 20, rng['y_max'] + 20)
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    # --- Bottom-left: 2D histogram heatmap ---
    ax3 = axes[1, 0]
    x_edges = np.linspace(rng['x_min'], rng['x_max'], GRID_X_CELLS + 1)
    y_edges = np.linspace(rng['y_min'], rng['y_max'], GRID_Y_CELLS + 1)
    h, xedge, yedge, im = ax3.hist2d(ax_data, ay_data, bins=[x_edges, y_edges],
                                       cmap='YlOrRd', cmin=0)
    fig.colorbar(im, ax=ax3, label='Point count')
    ax3.set_xlabel('actual_x_mm')
    ax3.set_ylabel('actual_y_mm')
    ax3.set_title('Data Density Heatmap')
    ax3.set_xlim(rng['x_min'], rng['x_max'])
    ax3.set_ylim(rng['y_min'], rng['y_max'])

    # --- Bottom-right: Gap analysis with grid ---
    ax4 = axes[1, 1]
    counts = np.zeros((GRID_Y_CELLS, GRID_X_CELLS), dtype=int)
    for x, y in zip(ax_data, ay_data):
        xi = np.searchsorted(x_edges, x, side='right') - 1
        yi = np.searchsorted(y_edges, y, side='right') - 1
        xi = np.clip(xi, 0, GRID_X_CELLS - 1)
        yi = np.clip(yi, 0, GRID_Y_CELLS - 1)
        counts[yi, xi] += 1

    # Plot grid cells colored by count
    for yi in range(GRID_Y_CELLS):
        for xi in range(GRID_X_CELLS):
            cnt = counts[yi, xi]
            if cnt == 0:
                color = '#ff4444'  # red = no data
                alpha = 0.6
            elif cnt < 2:
                color = '#ffaa00'  # orange = sparse
                alpha = 0.5
            elif cnt < 4:
                color = '#ffff44'  # yellow = low
                alpha = 0.4
            else:
                color = '#44cc44'  # green = good
                alpha = 0.4

            rect = Rectangle((x_edges[xi], y_edges[yi]),
                              x_edges[xi+1] - x_edges[xi],
                              y_edges[yi+1] - y_edges[yi],
                              facecolor=color, alpha=alpha,
                              edgecolor='gray', linewidth=1)
            ax4.add_patch(rect)
            ax4.text((x_edges[xi] + x_edges[xi+1]) / 2,
                     (y_edges[yi] + y_edges[yi+1]) / 2,
                     str(cnt), ha='center', va='center',
                     fontsize=11, fontweight='bold')

    ax4.scatter(ax_data, ay_data, c='black', s=15, zorder=5, alpha=0.6)
    ax4.set_xlabel('actual_x_mm')
    ax4.set_ylabel('actual_y_mm')
    ax4.set_title('Gap Analysis (red=0, orange=1, yellow=2-3, green=4+)')
    ax4.set_xlim(rng['x_min'], rng['x_max'])
    ax4.set_ylim(rng['y_min'], rng['y_max'])
    ax4.set_aspect('equal')

    plt.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n  Saved: {output_path}")
    plt.close(fig)


def main():
    print("Calibration Coverage Analysis")
    print("Stage: X=[0, 411]mm, Y=[0, 288]mm")
    print(f"Right cam Y range: [{CAM_RANGES['right']['y_min']}, {CAM_RANGES['right']['y_max']}]mm")
    print(f"Left  cam Y range: [{CAM_RANGES['left']['y_min']}, {CAM_RANGES['left']['y_max']}]mm")

    for cam_name, json_path in [('right', RIGHT_JSON), ('left', LEFT_JSON)]:
        data = load_data(json_path)
        ax, ay, px_u, px_v = print_stats(cam_name, data)
        counts, x_edges, y_edges, sparse = grid_density(ax, ay, cam_name)

        output_path = os.path.join(DATA_DIR, f'coverage_analysis_{cam_name}.png')
        plot_camera(cam_name, data, output_path)

        # Coverage summary
        total_cells = GRID_X_CELLS * GRID_Y_CELLS
        empty_cells = np.sum(counts == 0)
        sparse_cells = np.sum(counts < 2)
        good_cells = np.sum(counts >= 4)
        print(f"\n  Coverage summary:")
        print(f"    Total cells: {total_cells}")
        print(f"    Empty (0 pts): {empty_cells} ({100*empty_cells/total_cells:.0f}%)")
        print(f"    Sparse (<2 pts): {sparse_cells} ({100*sparse_cells/total_cells:.0f}%)")
        print(f"    Good (>=4 pts): {good_cells} ({100*good_cells/total_cells:.0f}%)")

        # Recommendations
        if sparse:
            print(f"\n  RECOMMENDATIONS for {cam_name} camera:")
            print(f"    Collect more data at these robot coordinates:")
            for xc, yc, cnt in sparse:
                print(f"      - X~{xc:.0f}mm, Y~{yc:.0f}mm (currently {cnt} point{'s' if cnt != 1 else ''})")

    print(f"\n{'='*60}")
    print("Analysis complete.")


if __name__ == '__main__':
    main()
