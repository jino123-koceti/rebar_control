#!/usr/bin/env python3
"""배근 pitch 추정 + 다음 이동거리 계산 — **자가테스트 진입점**.

⚠ 로직 정본은 `rebar_vision/coverage_planner.py`(패키지)다. ROS 노드가 tools/를
   import할 수 없어 패키지로 옮겼고, 여기서는 그걸 불러다 검증만 한다.

  python3 tools/drive/coverage_planner.py
"""
import os
import sys

ROOT = '/home/koceti/ros2_ws'
# install/ 빌드본이 앞서면 소스 수정이 안 보이므로 소스를 먼저 보게 고정
sys.path.insert(0, os.path.join(ROOT, 'src/rebar_vision'))
from rebar_vision.coverage_planner import (                       # noqa: E402
    compute_next_move, cluster_1d, estimate_pitch, _fmt)

__all__ = ['compute_next_move', 'cluster_1d', 'estimate_pitch', '_fmt']


if __name__ == '__main__':
    import json
    import glob

    print("=== 사용자 예시 4점 (10,50)(210,50)(10,250)(210,250) ===")
    print(_fmt(compute_next_move([(10, 50), (210, 50), (10, 250), (210, 250)])))
    print("  기대: pitch(200,200), 전진 400, 측면 400\n")

    print("=== 결측 열 테스트: 열 10,210,610 (410 빠짐) ===")
    print(_fmt(compute_next_move([(10, 50), (210, 50), (610, 50)])))
    print("  기대: pitch_x=200 (410결측 보정), n_cols=3\n")

    print("=== 실제 캘리브 데이터(=실배근 교차점 로봇XY)로 pitch 추정 검증 ===")
    base = os.path.join(ROOT, 'data/calibration')
    files = [os.path.join(base, 'calib3d_data_orbbec.json')]
    files += sorted(glob.glob(os.path.join(base, 'calib3d_data_orbbec.json.*')))
    for f in files:
        if not os.path.exists(f):
            continue
        try:
            d = json.load(open(f))
        except Exception:
            continue
        pts = [(p['ptool'][0], p['ptool'][1]) for p in d if 'ptool' in p]
        if len(pts) < 4:
            continue
        print(f"\n{os.path.basename(f)} ({len(pts)}점):")
        print(_fmt(compute_next_move(pts)))
