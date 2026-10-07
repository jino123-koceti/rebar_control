#!/usr/bin/env python3
"""yaw 를 **지정한 간격으로** 돌며 (건 각도, 영상) 쌍을 모은다.

왜 (2026-10-07 사용자 제안): 1번(-16.77°)~4번(+15.18°) 사이를 조금씩 움직이며
데이터를 모아 두면 자세 판정의 **경계와 여유**를 실측으로 정할 수 있다. 지금
판정기는 자세 4점에서만 검증됐다.

⚠ **검증된 호밍 뒤에만 쓴다.** 라벨(건 각도)은 `/stage/status` 의 `gun_deg` 에서
  읽는데, 그 값은 **멀티턴 기준점**에서 나온다. 기준점이 추정이면 라벨이 통째로
  틀어진다 — 2026-10-07 오전에 눈대중 기준값으로 모았다가 그래서 버렸다.
  그래서 `yaw_anchor` 가 '호밍 완료' 인지 확인하고 아니면 거부한다.

⚠⚠ **기본은 모의 실행이다.** `--go` 없이는 목표 목록만 출력하고 아무것도 보내지
  않는다. 장비를 움직이는 명령은 사람이 먼저 보고 승인한다 (사용자 지시).

사용
  python3 yaw_sweep_collect.py --out-dir ~/yawsweep --step 2.0        # 계획만
  python3 yaw_sweep_collect.py --out-dir ~/yawsweep --step 2.0 --go
"""

import argparse
import csv
import json
import os
import sys
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', required=True)
    p.add_argument('--step', type=float, default=2.0, help='건 각도 간격(도)')
    p.add_argument('--lo', type=float, default=-16.5, help='시작 건 각도')
    p.add_argument('--hi', type=float, default=15.0, help='끝 건 각도')
    p.add_argument('--settle', type=float, default=0.8, help='촬영 전 정착 대기(초)')
    p.add_argument('--go', action='store_true', help='⚠ 실제로 움직인다')
    a = p.parse_args()

    n = int(abs(a.hi - a.lo) / a.step) + 1
    targets = [round(a.lo + a.step * i, 2) for i in range(n)]
    print('=' * 58)
    print(f"yaw 스윕 — 건 {a.lo:+.1f}° ~ {a.hi:+.1f}°, {a.step}° 간격, {len(targets)}점")
    print('  ' + ', '.join(f'{t:+.1f}' for t in targets))
    print('=' * 58)
    if not a.go:
        print("\n모의 실행입니다 — 아무것도 보내지 않았습니다.")
        print("사람이 계획을 확인한 뒤 `--go` 를 붙여 실행하세요.")
        return 0

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Empty, Float32, String

    rclpy.init()
    node = Node('yaw_sweep_collect')
    bridge = CvBridge()
    st = {'s': None, 'img': None}
    node.create_subscription(String, '/stage/status',
                             lambda m: st.__setitem__('s', json.loads(m.data)), 10)
    node.create_subscription(
        Image, '/camera/color/image_raw',
        lambda m: st.__setitem__('img', bridge.imgmsg_to_cv2(m, 'bgr8')), 1)
    deg_pub = node.create_publisher(Float32, '/stage/yaw_deg', 10)
    stop_pub = node.create_publisher(Empty, '/stage/stop', 10)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    # ⚠ 구독 직후 DDS 탐색에 몇 초 걸린다 — 짧게 기다렸다가 "안 온다" 고
    #   거부한 적이 있다 (2026-10-07). 넉넉히 기다리되 상한을 둔다.
    end = time.time() + 15.0
    while time.time() < end and (st['s'] is None or st['img'] is None):
        spin(0.2)
    if st['s'] is None:
        print("✗ /stage/status 가 오지 않는다 — 서비스 확인")
        rclpy.shutdown()
        return 1
    if st['img'] is None:
        print("✗ 카메라 프레임이 오지 않는다")
        rclpy.shutdown()
        return 1
    anchor = (st['s'] or {}).get('yaw_anchor')
    if anchor != '호밍 완료':
        print(f"✗ yaw 기준점이 '{anchor}' 다 — **검증된 호밍 뒤에만** 모은다. "
              f"기준점이 추정이면 각도 라벨이 통째로 틀어진다")
        rclpy.shutdown()
        return 1

    os.makedirs(a.out_dir, exist_ok=True)
    idx = os.path.join(a.out_dir, 'index.csv')
    new = not os.path.exists(idx)
    fh = open(idx, 'a', newline='')
    w = csv.writer(fh)
    if new:
        w.writerow(['file', 'gun_deg', 'pose', 'x_mm', 'y_mm', 'z_mm', 'detail'])
        fh.flush()

    saved = 0
    try:
        for i, t in enumerate(targets, 1):
            rej0 = (st['s'] or {}).get('rejects', 0)
            deg_pub.publish(Float32(data=float(t)))
            end = time.time() + 30.0
            ok = False
            while time.time() < end:
                spin(0.1)
                s = st['s'] or {}
                if s.get('rejects', 0) > rej0:
                    print(f"[{i}/{len(targets)}] 건 {t:+.1f}° — 거부: "
                          f"{str(s.get('detail'))[:60]}")
                    break
                g = s.get('gun_deg')
                if (not s.get('yaw_moving') and g is not None
                        and abs(g - t) < 0.6):
                    ok = True
                    break
            if not ok:
                continue
            spin(a.settle)                 # 기구 진동이 가라앉기를 기다린다
            s = st['s'] or {}
            c = s.get('current_mm') or {}
            g = s.get('gun_deg')
            name = f"gun{g:+07.2f}.png".replace('+', 'p').replace('-', 'm')
            cv2.imwrite(os.path.join(a.out_dir, name), st['img'])
            w.writerow([name, f"{g:.2f}", s.get('pose'), c.get('x'), c.get('y'),
                        c.get('z'), s.get('pose_detail')])
            fh.flush()                     # ⚠ 매 장 flush — 중단해도 남는다
            saved += 1
            print(f"[{i}/{len(targets)}] 건 {g:+.2f}°  자세 {s.get('pose')}  → {name}",
                  flush=True)
    except KeyboardInterrupt:
        print("\n중단 — 정지 보냄")
        stop_pub.publish(Empty())
        spin(0.5)
    finally:
        fh.close()
        rclpy.shutdown()
    print(f"\n{saved}장 수집 → {a.out_dir}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
