#!/usr/bin/env python3
"""
캘리브레이션 모델 성능 비교 (오프라인)

calibration_data_{camera}.json을 읽어서
다양한 ML 모델 + 다중선형회귀를 5-fold CV로 비교

사용법:
    python3 compare_models.py              # left (기본)
    python3 compare_models.py --camera right
    python3 compare_models.py --camera both
"""

import argparse
import json
import os
import warnings
import numpy as np

warnings.filterwarnings('ignore')

DATA_DIR = '/home/koceti/ros2_ws/data/calibration'


def load_data(camera):
    path = os.path.join(DATA_DIR, f'calibration_data_{camera}.json')
    with open(path) as f:
        data = json.load(f)
    points = data if isinstance(data, list) else data.get('points', [])
    return points


def build_features(points):
    """공통 feature 추출"""
    us = np.array([p['pixel_u'] for p in points], dtype=np.float64)
    vs = np.array([p['pixel_v'] for p in points], dtype=np.float64)
    ds = np.array([p['depth_mm'] for p in points], dtype=np.float64)
    ax = np.array([p['actual_x_mm'] for p in points], dtype=np.float64)
    ay = np.array([p['actual_y_mm'] for p in points], dtype=np.float64)
    return us, vs, ds, ax, ay


def compare(camera):
    from sklearn.linear_model import LinearRegression, Ridge, Lasso, ElasticNet
    from sklearn.ensemble import (
        GradientBoostingRegressor, RandomForestRegressor,
        AdaBoostRegressor, ExtraTreesRegressor
    )
    from sklearn.svm import SVR
    from sklearn.neighbors import KNeighborsRegressor
    from sklearn.tree import DecisionTreeRegressor
    from sklearn.neural_network import MLPRegressor
    from sklearn.model_selection import KFold
    from sklearn.base import clone
    from sklearn.preprocessing import StandardScaler
    from sklearn.pipeline import Pipeline

    points = load_data(camera)
    n = len(points)
    print(f'\n{"="*65}')
    print(f'[{camera}] 캘리브레이션 모델 비교 ({n}쌍)')
    print(f'{"="*65}')

    us, vs, ds, ax, ay = build_features(points)

    # Feature 셋 정의
    X_uvd = np.column_stack([us, vs, ds])
    X_poly = np.column_stack([
        us, vs, ds, us*ds, vs*ds, us**2, vs**2, np.ones(n)
    ])
    X_uv_only = np.column_stack([
        us, vs, us*vs, us**2, vs**2, np.ones(n)
    ])
    X_full_poly = np.column_stack([
        us, vs, ds, us*ds, vs*ds, us**2, vs**2,
        us*vs, ds**2, np.ones(n)
    ])

    kf = KFold(n_splits=5, shuffle=True, random_state=42)

    def cv_mae(model, X, y):
        scores = []
        for tr, te in kf.split(X):
            m = clone(model)
            m.fit(X[tr], y[tr])
            scores.append(np.mean(np.abs(m.predict(X[te]) - y[te])))
        return np.mean(scores)

    def cv_max(model, X, y):
        scores = []
        for tr, te in kf.split(X):
            m = clone(model)
            m.fit(X[tr], y[tr])
            scores.append(np.max(np.abs(m.predict(X[te]) - y[te])))
        return np.mean(scores)

    # 모델 정의
    models = [
        # --- 선형 모델 ---
        ('Linear(u,v,d)', LinearRegression(), X_uvd),
        ('Linear+poly(8계수)', LinearRegression(), X_poly),
        ('Linear+fullpoly(10계수)', LinearRegression(), X_full_poly),
        ('Linear(no depth)', LinearRegression(), X_uv_only),
        ('Ridge(poly)', Ridge(alpha=1.0), X_poly),
        ('Lasso(poly)', Lasso(alpha=0.1, max_iter=5000), X_poly),
        ('ElasticNet(poly)', ElasticNet(alpha=0.1, l1_ratio=0.5, max_iter=5000), X_poly),

        # --- 트리 기반 ---
        ('DecisionTree', DecisionTreeRegressor(max_depth=6), X_uvd),
        ('RandomForest', RandomForestRegressor(
            n_estimators=200, max_depth=6, random_state=42), X_uvd),
        ('ExtraTrees', ExtraTreesRegressor(
            n_estimators=200, max_depth=6, random_state=42), X_uvd),
        ('GBR(기본)', GradientBoostingRegressor(
            n_estimators=300, max_depth=4, learning_rate=0.05,
            subsample=0.8, random_state=42), X_uvd),
        ('GBR(깊은)', GradientBoostingRegressor(
            n_estimators=500, max_depth=6, learning_rate=0.03,
            subsample=0.8, random_state=42), X_uvd),
        ('AdaBoost', AdaBoostRegressor(
            n_estimators=200, learning_rate=0.05, random_state=42), X_uvd),

        # --- KNN ---
        ('KNN(k=5)', KNeighborsRegressor(n_neighbors=5, weights='distance'), X_uvd),
        ('KNN(k=10)', KNeighborsRegressor(n_neighbors=10, weights='distance'), X_uvd),

        # --- SVR ---
        ('SVR(rbf)', Pipeline([
            ('scaler', StandardScaler()),
            ('svr', SVR(kernel='rbf', C=100, epsilon=1.0))
        ]), X_uvd),

        # --- MLP ---
        ('MLP(100,50)', Pipeline([
            ('scaler', StandardScaler()),
            ('mlp', MLPRegressor(
                hidden_layer_sizes=(100, 50), max_iter=2000,
                learning_rate='adaptive', random_state=42))
        ]), X_uvd),
        ('MLP(200,100,50)', Pipeline([
            ('scaler', StandardScaler()),
            ('mlp', MLPRegressor(
                hidden_layer_sizes=(200, 100, 50), max_iter=3000,
                learning_rate='adaptive', random_state=42))
        ]), X_uvd),
    ]

    # 비교 실행
    results = []
    print(f'\n  {"모델":<25} {"X MAE":>7} {"Y MAE":>7} {"평균":>7} {"X max":>7} {"Y max":>7}')
    print(f'  {"-"*62}')

    for name, model, X in models:
        try:
            mx = cv_mae(model, X, ax)
            my = cv_mae(model, X, ay)
            mxmax = cv_max(model, X, ax)
            mymax = cv_max(model, X, ay)
            avg = (mx + my) / 2
            results.append((name, mx, my, avg, mxmax, mymax))
            print(f'  {name:<25} {mx:7.2f} {my:7.2f} {avg:7.2f} {mxmax:7.1f} {mymax:7.1f}')
        except Exception as e:
            print(f'  {name:<25} FAILED: {e}')

    # 정렬된 순위
    results.sort(key=lambda r: r[3])
    print(f'\n  === 순위 (평균 MAE 기준) ===')
    for i, (name, mx, my, avg, mxmax, mymax) in enumerate(results):
        marker = ' ★' if i == 0 else ''
        print(f'  {i+1:2d}. {name:<25} 평균={avg:.2f}mm  '
              f'(X={mx:.2f}, Y={my:.2f}, Xmax={mxmax:.1f}, Ymax={mymax:.1f}){marker}')

    print()

    # 플롯 생성
    plot_results(results, camera, n)


def plot_results(results, camera, n_points):
    """비교 결과 플롯 생성"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    names = [r[0] for r in results]
    x_mae = [r[1] for r in results]
    y_mae = [r[2] for r in results]
    avg_mae = [r[3] for r in results]
    x_max = [r[4] for r in results]
    y_max = [r[5] for r in results]

    fig, axes = plt.subplots(1, 2, figsize=(16, max(6, len(names) * 0.4)))

    # (1) MAE 비교 (수평 바 차트)
    y_pos = np.arange(len(names))
    bar_h = 0.35

    axes[0].barh(y_pos + bar_h/2, x_mae, bar_h, label='X MAE', color='steelblue', alpha=0.8)
    axes[0].barh(y_pos - bar_h/2, y_mae, bar_h, label='Y MAE', color='coral', alpha=0.8)

    # 평균 MAE 점 표시
    axes[0].scatter(avg_mae, y_pos, color='black', zorder=5, s=30, label='Avg MAE')

    axes[0].set_yticks(y_pos)
    axes[0].set_yticklabels(names, fontsize=9)
    axes[0].set_xlabel('MAE (mm)')
    axes[0].set_title(f'{camera} camera - 5-fold CV MAE ({n_points} points)')
    axes[0].legend(loc='lower right')
    axes[0].invert_yaxis()

    # 1위 하이라이트
    axes[0].get_yticklabels()[0].set_fontweight('bold')
    axes[0].get_yticklabels()[0].set_color('red')

    # (2) Max error 비교
    axes[1].barh(y_pos + bar_h/2, x_max, bar_h, label='X Max', color='steelblue', alpha=0.8)
    axes[1].barh(y_pos - bar_h/2, y_max, bar_h, label='Y Max', color='coral', alpha=0.8)
    axes[1].set_yticks(y_pos)
    axes[1].set_yticklabels(names, fontsize=9)
    axes[1].set_xlabel('Max Error (mm)')
    axes[1].set_title(f'{camera} camera - 5-fold CV Max Error ({n_points} points)')
    axes[1].legend(loc='lower right')
    axes[1].invert_yaxis()

    axes[1].get_yticklabels()[0].set_fontweight('bold')
    axes[1].get_yticklabels()[0].set_color('red')

    plt.tight_layout()
    out_path = os.path.join(DATA_DIR, f'model_comparison_{camera}.png')
    plt.savefig(out_path, dpi=120)
    plt.close()
    print(f'  플롯 저장: {out_path}')


def main():
    parser = argparse.ArgumentParser(description='캘리브레이션 모델 성능 비교')
    parser.add_argument('--camera', choices=['left', 'right', 'both'],
                        default='left')
    args = parser.parse_args()

    cameras = ['left', 'right'] if args.camera == 'both' else [args.camera]
    for cam in cameras:
        compare(cam)


if __name__ == '__main__':
    main()
