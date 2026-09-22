#!/usr/bin/env python3
"""heading 실시간 표시 — 손으로 로봇을 돌리며 목표값에 맞출 때 쓴다.

진행방향 카메라만 활성이므로 `/travel_direction`을 계속 발행해 고정한다.
⚠ rebar_drive_node가 도는 중에는 쓰지 말 것(같은 토픽을 서로 덮어쓴다).

표시:
  현재값 / 목표까지 남은 양 / **어느 쪽으로 돌려야 하는지**
  회전 방향은 실측으로 확정: **CW로 돌리면 heading이 감소**한다
  (2026-08-19: CW 회전에 front -1.34° → -3.44°, Δ-2.10°).

사용:
    python3 tools/vision_test/heading_live.py                 # front, 목표 0
    python3 tools/vision_test/heading_live.py --cam back --target -0.9
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', default='front', choices=['front', 'back'])
    ap.add_argument('--target', type=float, default=0.0,
                    help='목표 heading(원시값). --offset 쓰면 0으로 두면 된다')
    ap.add_argument('--offset', type=float, default=None,
                    help='영점(heading_offset_*_deg). 주면 **보정 후 오차**로 표시한다')
    ap.add_argument('--tol', type=float, default=0.15, help='도달 판정 허용범위(deg)')
    ap.add_argument('--sec', type=float, default=600.0)
    # 기본은 **계속 감시**한다. 돌렸다가 되돌아오는 반복 시험에 쓰므로
    # 한 번 도달했다고 끝내면 안 된다.
    ap.add_argument('--stop-on-reach', action='store_true',
                    help='목표 도달하면 종료 (기본: 계속 감시)')
    a = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('heading_live')
    dir_pub = n.create_publisher(String, '/travel_direction', 10)
    buf = []
    state = {'bars': 0}

    def cb(m):
        try:
            r = json.loads(m.data)
        except (ValueError, TypeError):
            return
        if r.get('cam') != a.cam:
            return
        h = r.get('heading_deg')
        if h is None:
            return
        buf.append(float(h))
        del buf[:-5]
        state['bars'] = int(r.get('heading_bars', 0))

    n.create_subscription(String, '/deck_edge_status', cb, 10)
    direction = 'forward' if a.cam == 'front' else 'backward'

    if a.offset is not None:
        a.target = a.target + a.offset
        print(f'■ {a.cam} heading 실시간  영점 {a.offset:+.2f}° 적용 '
              f'→ **보정 후 오차 0** 이 목표 (원시 {a.target:+.2f}°)')
    else:
        print(f'■ {a.cam} heading 실시간  (목표 {a.target:+.2f}°, 허용 ±{a.tol}°)')
    print(f'  ★ CW(시계)로 돌리면 heading **감소**, CCW로 돌리면 **증가**\n')

    t0 = time.time()
    last = 0.0
    hold = 0
    while time.time() - t0 < a.sec:
        m = String(); m.data = direction
        dir_pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.1)
        now = time.time()
        if now - last < 0.5 or not buf:
            continue
        last = now
        v = sorted(buf)
        med = v[len(v)//2] if len(v) % 2 else (v[len(v)//2-1]+v[len(v)//2])/2.0
        err = med - a.target
        if abs(err) <= a.tol:
            hold += 1
            bar = '  ✅ 도달' + (f' ({hold}회 유지)' if hold > 1 else '')
        else:
            hold = 0
            # heading을 올리려면 CCW, 내리려면 CW
            way = 'CCW(반시계)로' if err < 0 else 'CW(시계)로'
            bar = f'  → {way} {abs(err):.2f}° 더'
        shown = (f'오차 {med - a.target:+6.2f}°  (원시 {med:+.2f})'
                 if a.offset is not None else f'{med:+6.2f}°')
        print(f'  {shown}   (철근 {state["bars"]}개, n={len(buf)}){bar}', flush=True)
        if hold >= 6 and a.stop_on_reach:
            print(f'\n✅ 안정적으로 목표 도달. 현재 {med:+.2f}°')
            break

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
