#!/usr/bin/env python3
"""orbbec 실검출 → 배근 pitch/다음이동 end-to-end 테스트.
OrbbecLocalizer로 실제 교차점을 로봇XY로 검출 → coverage_planner로 다음 이동거리 산출.
  python3 tools/drive/test_orbbec_coverage.py [--pose r|l]
"""
import os
import sys
import time
import argparse
import rclpy
from rclpy.node import Node

ROOT = '/home/koceti/ros2_ws'
sys.path.insert(0, os.path.join(ROOT, 'src/rebar_vision/rebar_vision'))
sys.path.insert(0, os.path.join(ROOT, 'tools/drive'))
from orbbec_detector import OrbbecLocalizer          # noqa: E402
from coverage_planner import compute_next_move, _fmt  # noqa: E402

MODEL = os.path.join(ROOT, 'src/rebar_vision/model/orbbec_crossing.pt')
HOMO = os.path.join(ROOT, 'data/calibration/homography_orbbec.yaml')
YRANGE = {'r': (0.0, 142.0), 'l': (124.0, 288.0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', default='r')
    args = ap.parse_args()
    rclpy.init()
    node = Node('orbbec_cov_test')
    loc = OrbbecLocalizer(node, MODEL, HOMO, '/camera/color/image_raw',
                          YRANGE, x_min=30.0, x_max=345.0)
    t = time.time()
    while rclpy.ok() and not loc.ready() and time.time() - t < 12:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(15):                       # 버퍼 채우기
        rclpy.spin_once(node, timeout_sec=0.1)
    if not loc.ready():
        print('⚠ orbbec 준비 안됨 (컬러토픽/호모그래피 확인)')
        node.destroy_node(); rclpy.shutdown(); return

    sets = loc.localize(args.pose)
    # ⚠ localize()는 (x,y,u,v,cls,conf) 6튜플을 준다 — 앞 2개만 쓴다(언패킹 금지).
    pts = [(c[0], c[1]) for c in sets['r']] + \
          [(c[0], c[1]) for c in sets['l']]
    print(f"\n검출 교차점 {len(pts)}개 (로봇XY mm):")
    for x, y in sorted(pts, key=lambda p: (round(p[1]/50), p[0])):
        print(f"   ({x:6.1f}, {y:6.1f})")
    print("\n=== 다음 이동 계산 ===")
    print(_fmt(compute_next_move(pts)))
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
