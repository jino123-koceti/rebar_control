#!/usr/bin/env python3
"""`tying_planner` 를 가짜 장비로 시험한다 — 검출부터 순회까지.

가짜 검출 노드 + 가짜 stage_node 를 띄우고 `/plan/start` 를 보낸다. 실제
`tying_planner` 와 `tying_sequence` 는 **진짜 그대로** 쓴다 — 계획·자세 선택·
순서·실패 건너뛰기가 맞는지 보려는 것이다.

교차점은 **실제 스냅샷**(`stage_camera_pairs.json` 의 `frozen`)을 쓴다. 손으로
만든 좌표는 모델이 어디로 보내는지 가늠이 안 돼서 도달 가능 여부가 비현실적이다.

⚠ `ROS_DOMAIN_ID` 를 따로 쓴다 (기본 77). 실장비(33)와 섞이면 가짜 검출이
진짜 플래너를 움직인다.

    ROS_DOMAIN_ID=77 ros2 run rmd_robot_control tying_sequence &
    ROS_DOMAIN_ID=77 ros2 run rmd_robot_control tying_planner &
    ROS_DOMAIN_ID=77 python3 tools/test/plan_sim.py --pose 1
"""

import argparse
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Point            # noqa: F401  (FakeStage 가 쓴다)
from std_msgs.msg import Empty, String
from rebar_base_interfaces.msg import RebarDetection, RebarGrid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tying_sim import FakeStage                # noqa: E402

PAIRS = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/stage_camera_pairs.json')


def load_crossings(path, limit):
    """`frozen` = [x, y, z, 신뢰도, 픽셀u, 픽셀v] (카메라 mm)."""
    d = json.load(open(path, encoding='utf-8'))
    out = []
    for c in (d.get('frozen') or [])[:limit]:
        r = RebarDetection()
        r.x, r.y, r.z = float(c[0]), float(c[1]), float(c[2])
        r.confidence = float(c[3])
        r.depth_mm = float(c[2])
        r.pixel_u, r.pixel_v = int(c[4]), int(c[5])
        r.camera_id = 0
        out.append(r)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', type=int, default=1, help='시작 자세')
    ap.add_argument('--x', type=float, default=30.0)
    ap.add_argument('--y', type=float, default=100.0)
    ap.add_argument('--z', type=float, default=0.0)
    ap.add_argument('--limit', type=int, default=11, help='교차점 몇 개까지')
    ap.add_argument('--timeout', type=float, default=400.0)
    a = ap.parse_args()

    rclpy.init()
    fake = FakeStage(None if a.pose < 0 else a.pose, a.x, a.y, a.z)
    dets = load_crossings(PAIRS, a.limit)

    grid_pub = fake.create_publisher(RebarGrid, '/rebar/crossings', 10)
    start_pub = fake.create_publisher(Empty, '/plan/start', 10)
    seen = []
    fake.create_subscription(String, '/plan/status',
                             lambda m: seen.append(json.loads(m.data)), 10)

    def send_grid():
        g = RebarGrid()
        g.detections = dets
        g.grid_rows = g.grid_cols = 0
        g.total_detected = len(dets)
        g.valid = True
        g.error_message = 'frame=camera'       # 플래너가 이 표시를 본다
        grid_pub.publish(g)
        print(f"    [가짜 검출] 교차점 {len(dets)}개 발행")

    fake.create_subscription(Empty, '/rebar/detect', lambda m: send_grid(), 10)

    def pump(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(fake, timeout_sec=0.02)

    pump(3.0)
    print(f"시작  자세 {a.pose}  X {a.x:.1f}  Y {a.y:.1f}  Z {a.z:.1f}")
    print(f"교차점 {len(dets)}개 (실제 스냅샷)\n")
    start_pub.publish(Empty())

    last, t0 = None, time.time()
    while time.time() - t0 < a.timeout:
        pump(0.2)
        if not seen:
            continue
        s = seen[-1]
        key = (s['phase'], s['current'], s['detail'])
        if key != last:
            last = key
            print(f"  {time.time()-t0:6.1f}s  [{s['phase']:<7}] {s['detail']}")
        if s['phase'] in ('done', 'failed'):
            break

    if not seen:
        print('\n⚠ /plan/status 가 오지 않았다 — tying_planner 가 떠 있는가')
        return 1
    s = seen[-1]
    print(f"\n결과: {s['phase']} — {s['detail']}")
    print('계획된 점:')
    for p in s['points']:
        print(f"  {p['idx']:2d}번  자세 {p['pose']}  "
              f"{p['stage_mm']}  {p['state']}")
    bad = [w for k, w in fake.log if k == '거부']
    if bad:
        print(f"\n⚠ 가짜 stage 거부 {len(bad)}건 — 순서가 틀렸다:")
        for w in bad:
            print(f"    {w}")
    return 0 if s['phase'] == 'done' and not bad else 1


if __name__ == '__main__':
    sys.exit(main())
