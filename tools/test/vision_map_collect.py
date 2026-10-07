#!/usr/bin/env python3
"""호밍 후 **아는 위치**로 스테이지를 옮기며 (영상, 위치) 쌍을 모은다.

왜 (2026-10-07 사용자 제안): 전원 재투입 후에는 단회전 엔코더가 **후보 여럿**만
주고 하나를 못 고른다. yaw 는 후보가 둘(건 28.8° 간격), X 는 다섯(104.6mm 간격)이다.
그래서 호밍이 거부되고 사람이 손으로 12시에 맞춰야 한다.

사용자 발상: **호밍 후에는 위치를 정확히 아니까, 그때 영상을 모아 매핑을 만들어
두면 호밍 전에도 영상으로 후보를 고를 수 있다.** 비전이 정밀할 필요는 없다 —
후보 간격의 절반만 맞히면 되고(X ±52mm, Y ±40mm, yaw ±14°), 정밀도는 엔코더가 준다.

⚠ **라벨이 추정이면 안 된다.** 2026-10-07 에 리모콘으로 먼저 모아봤는데, 그때
라벨의 바탕(`noon_single`·에지값)이 눈대중·잠정값이었다. 그래서 **검증된 호밍 뒤에만**
모은다. 이 도구가 `homed` 를 확인하고 아니면 거부하는 이유다.

⚠⚠ **기본은 모의 실행이다.** `--go` 없이는 목표 목록만 출력하고 아무것도 보내지
않는다. 장비를 움직이는 명령은 사람이 먼저 보고 승인한다는 약속 때문이다
(2026-10-07 사용자 지시). 계획을 보여준 뒤 `--go` 로 실행한다.

사용
  python3 vision_map_collect.py --out-dir ~/vmap --yaw-poses          # 계획만
  python3 vision_map_collect.py --out-dir ~/vmap --yaw-poses --go     # 실행
  python3 vision_map_collect.py --out-dir ~/vmap --grid x=0:450:25 --go
"""

import argparse
import csv
import json
import os
import sys
import time

AXES = ('x', 'y', 'z')


def parse_grid(specs):
    """`x=0:450:25` 꼴을 {축: [값…]} 으로."""
    out = {}
    for s in specs or []:
        name, _, rng = s.partition('=')
        a, b, step = (float(v) for v in rng.split(':'))
        n = int(abs(b - a) / step) + 1
        sign = 1.0 if b >= a else -1.0
        out[name] = [round(a + sign * step * i, 2) for i in range(n)]
    return out


class Collector:
    def __init__(self, node, out_dir):
        from cv_bridge import CvBridge
        from geometry_msgs.msg import Point
        from sensor_msgs.msg import Image
        from std_msgs.msg import Empty, Int32, String

        self.node = node
        self.bridge = CvBridge()
        self.img = None
        self.stage = None
        self.safety_estop = None
        self.out_dir = out_dir
        self.goal_pub = node.create_publisher(Point, '/stage/goal', 10)
        self.pose_pub = node.create_publisher(Int32, '/stage/yaw_pose', 10)
        self.stop_pub = node.create_publisher(Empty, '/stage/stop', 10)
        node.create_subscription(
            Image, '/camera/color/image_raw',
            lambda m: setattr(self, 'img',
                              self.bridge.imgmsg_to_cv2(m, 'bgr8')), 1)
        node.create_subscription(
            String, '/stage/status',
            lambda m: setattr(self, 'stage', _safe(m.data)), 10)

    def spin(self, sec):
        import rclpy
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def ready(self, need_axes, wait=15.0):
        """움직이기 전 전제. 하나라도 어긋나면 사유를 돌려준다.

        ⚠ 구독 직후 **DDS 탐색에 몇 초 걸린다.** 2초만 기다렸다가 "상태가 안 온다"
          로 거부한 적이 있다 (2026-10-07). 토픽은 멀쩡했고 기다림이 짧았다.
          그래서 둘 다 올 때까지 기다리되 상한을 둔다.
        """
        end = time.time() + wait
        while time.time() < end and (self.stage is None or self.img is None):
            self.spin(0.2)
        st = self.stage
        if st is None:
            return '/stage/status 가 오지 않는다 — 서비스 확인'
        if self.img is None:
            return '카메라 프레임이 오지 않는다 — gemini2L 확인'
        missing = [a for a in need_axes if a not in (st.get('homed') or [])]
        if missing:
            return (f"호밍되지 않은 축: {', '.join(missing)} — "
                    f"**검증된 호밍 뒤에만** 모은다 (라벨이 추정이면 안 된다)")
        return None

    def mm(self):
        return (self.stage or {}).get('current_mm') or {}

    def limits(self):
        return (self.stage or {}).get('limit_mm') or {}

    def in_range(self, want):
        """지금 자세의 가동범위 안인가. 밖이면 사유 문자열.

        ⚠ **가동범위는 자세마다 다르다.** 12시에서 X 는 -5.0~351.0mm 인데 다른
          자세에서는 더 좁거나 넓다. 그래서 계획 시점이 아니라 **보내기 직전에**
          그 순간의 `limit_mm` 과 대조한다 (자세를 바꿔가며 수집하기 때문이다).
        ⚠ `limit_mm` 이 없으면(범위 정보 없음) **보내지 않는다.** 모르는 채로
          보내는 것보다 건너뛰는 쪽이 안전하다 — 범위 밖 목표가 프레임을 친다.
        """
        lim = self.limits()
        if not lim:
            return '가동범위 정보(limit_mm)가 없다'
        for k, v in want.items():
            if v is None or k not in lim:
                continue
            lo, hi = lim[k]
            if lo is not None and v < lo:
                return f'{k}={v}mm < 하한 {lo}mm'
            if hi is not None and v > hi:
                return f'{k}={v}mm > 상한 {hi}mm'
        return None

    def goto(self, want, timeout=40.0, tol=2.0):
        """mm 목표. 도달하면 True.

        ⚠ **범위 밖이면 보내지 않는다** — 거부를 유발하지 않고 건너뛴다.
          `rejects` 는 상위 노드가 "방금 거부됐다" 를 보는 신호라, 수집 때문에
          올리면 그 신호가 흐려진다.
        ⚠ `moving` 이 내려가는 것만 보지 않는다 — 거부되면 애초에 안 움직이므로
          `rejects` 증가도 같이 본다. 거부를 도달로 세면 엉뚱한 위치의 영상을
          옳은 라벨로 저장하게 된다 (매핑이 통째로 오염된다).
        """
        from geometry_msgs.msg import Point
        bad = self.in_range(want)
        if bad:
            print(f"    · 범위 밖이라 보내지 않음 — {bad}")
            return False
        rej0 = (self.stage or {}).get('rejects', 0)
        pt = Point()
        for k in AXES:
            v = want.get(k)
            setattr(pt, k, float('nan') if v is None else float(v))
        self.goal_pub.publish(pt)
        end = time.time() + timeout
        while time.time() < end:
            self.spin(0.1)
            st = self.stage or {}
            if st.get('rejects', 0) > rej0:
                print(f"    ✗ 거부: {st.get('detail')}")
                return False
            now = self.mm()
            ok = all(now.get(k) is not None and abs(now[k] - v) <= tol
                     for k, v in want.items() if v is not None)
            if ok and not st.get('moving'):
                return True
        print("    ✗ 도달 타임아웃")
        return False

    def goto_pose(self, pose, timeout=40.0):
        from std_msgs.msg import Int32
        rej0 = (self.stage or {}).get('rejects', 0)
        self.pose_pub.publish(Int32(data=int(pose)))
        end = time.time() + timeout
        while time.time() < end:
            self.spin(0.1)
            st = self.stage or {}
            if st.get('rejects', 0) > rej0:
                print(f"    ✗ 거부: {st.get('detail')}")
                return False
            if st.get('pose') == pose and not st.get('yaw_moving'):
                return True
        print("    ✗ 자세 도달 타임아웃")
        return False

    def grab(self, writer, fh, tag):
        """정착을 기다린 뒤 한 장. 라벨은 **그 순간의 상태**에서 읽는다."""
        import cv2
        self.spin(0.6)                       # 기구 진동이 가라앉기를 기다린다
        st = self.stage or {}
        now = self.mm()
        name = f"{tag}.png"
        cv2.imwrite(os.path.join(self.out_dir, name), self.img)
        writer.writerow([name, now.get('x'), now.get('y'), now.get('z'),
                         st.get('pose'), st.get('gun_deg'), st.get('pose_detail')])
        fh.flush()                           # ⚠ 매 장 flush — 중단해도 남는다
        print(f"    저장 {name}  x={now.get('x')} y={now.get('y')} "
              f"z={now.get('z')} 자세={st.get('pose')}")


def _safe(s):
    try:
        return json.loads(s)
    except ValueError:
        return None


def plan_targets(args):
    """수집 지점 목록.

    `--serpentine` 은 X·Y 를 **지그재그**로 덮는다 — X 한 칸 전진 → Y 를 쭉 훑고
    → X 한 칸 → Y 를 **반대 방향**으로. 이동거리가 최소이고 긴 복귀가 없다.

    ⚠ 축을 따로 훑으면 **조합이 비어** 쓸 수 없다. 2026-10-07 에 X 를 훑고(Y 고정)
      Y 를 훑었더니(X=400 고정) 格子의 가장자리 두 줄만 덮였다. 건의 모습은
      **X·Y·yaw 에 모두 의존**하므로(카메라가 고정이고 건이 움직인다) 조합을
      덮어야 템플릿이 된다.
    """
    grid = parse_grid(args.grid)
    jobs = []
    if args.yaw_poses:
        for p in (1, 2, 3, 4):
            jobs.append(('pose', p))
    if args.serpentine and 'x' in grid and 'y' in grid:
        ys = list(grid['y'])
        for i, xv in enumerate(grid['x']):
            order = ys if i % 2 == 0 else list(reversed(ys))
            for yv in order:
                jobs.append(('xy', (xv, yv)))
        return jobs
    for ax, vals in grid.items():
        for v in vals:
            jobs.append((ax, v))
    return jobs


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', required=True)
    p.add_argument('--yaw-poses', action='store_true')
    p.add_argument('--grid', action='append',
                   help='예: x=0:450:25 (여러 번 줄 수 있다)')
    p.add_argument('--z-park', type=float, default=0.0,
                   help='수집 중 Z 를 여기에 둔다 (올린 상태. 건이 철근을 긁지 않게)')
    p.add_argument('--serpentine', action='store_true',
                   help='X·Y 를 지그재그로 덮는다 (조합이 비지 않게)')
    p.add_argument('--pose-first', type=int,
                   help='격자 수집 전에 이 자세로 먼저 돌린다. '
                        '가동범위가 자세마다 다르므로 격자를 잡은 자세와 맞춰야 한다')
    p.add_argument('--go', action='store_true',
                   help='⚠ 실제로 움직인다. 없으면 계획만 출력한다')
    a = p.parse_args()

    jobs = plan_targets(a)
    if not jobs:
        print("--yaw-poses 또는 --grid 가 필요합니다.")
        return 1

    print("=" * 62)
    print(f"수집 계획 — 지점 {len(jobs)}개, Z 는 {a.z_park}mm 로 올린 채")
    if a.pose_first:
        print(f"  [먼저] Z 를 올린 뒤 → {a.pose_first}번 자세로 회전")
    for kind, v in jobs:
        label = '자세' if kind == 'pose' else kind
        shown = f"x={v[0]}, y={v[1]}" if kind == 'xy' else v
        print(f"  {label:>4s} → {shown}")
    print("=" * 62)
    if not a.go:
        print("\n모의 실행입니다 — 아무것도 보내지 않았습니다.")
        print("⚠ 위 목록은 **요청 지점**이다. 실행 시 각 지점을 보내기 직전에")
        print("  그 자세의 가동범위(`limit_mm`)와 대조해, 범위 밖이면 보내지 않고")
        print("  건너뛴다. 가동범위는 자세마다 다르다.")
        print("사람이 계획을 확인한 뒤 `--go` 를 붙여 실행하세요.")
        return 0

    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node('vision_map_collect')
    col = Collector(node, a.out_dir)
    os.makedirs(a.out_dir, exist_ok=True)

    need = ['yaw'] if a.yaw_poses else []
    need += [k for k in parse_grid(a.grid)]
    bad = col.ready(sorted(set(need + ['z'])))
    if bad:
        print(f"✗ 전제 불충족 — {bad}")
        rclpy.shutdown()
        return 1

    idx = os.path.join(a.out_dir, 'index.csv')
    new = not os.path.exists(idx)
    fh = open(idx, 'a', newline='')
    w = csv.writer(fh)
    if new:
        w.writerow(['file', 'x_mm', 'y_mm', 'z_mm', 'pose', 'gun_deg', 'detail'])
        fh.flush()

    n = 0
    try:
        print(f"\n[0] Z 를 {a.z_park}mm 로 올린다")
        if not col.goto({'z': a.z_park}):
            raise SystemExit('Z 를 올리지 못했다 — 중단')
        if a.pose_first:
            # ⚠ Z 를 **먼저** 올린 뒤에 돌린다 — 회전하면 건이 원을 그리므로
            #   Z 가 내려가 있으면 철근을 친다 (stage_node 도 회전 중 Z 를 거부한다).
            print(f"[0b] {a.pose_first}번 자세로 회전")
            if not col.goto_pose(a.pose_first):
                raise SystemExit('자세 복귀 실패 — 중단')
        for i, (kind, v) in enumerate(jobs, 1):
            print(f"[{i}/{len(jobs)}] {kind} → {v}")
            if kind == 'pose':
                ok = col.goto_pose(v)
                tag = f"pose{v}"
            elif kind == 'xy':
                ok = col.goto({'x': v[0], 'y': v[1]})
                tag = (f"x{v[0]:+07.1f}_y{v[1]:+07.1f}"
                       .replace('+', 'p').replace('-', 'm'))
            else:
                ok = col.goto({kind: v})
                tag = f"{kind}{v:+08.2f}".replace('+', 'p').replace('-', 'm')
            if not ok:
                print("    건너뜀")
                continue
            col.grab(w, fh, tag)
            n += 1
    except KeyboardInterrupt:
        print("\n중단 요청 — 정지 보냄")
        from std_msgs.msg import Empty
        col.stop_pub.publish(Empty())
        col.spin(0.5)
    finally:
        fh.close()
        rclpy.shutdown()
    print(f"\n{n}장 수집 → {a.out_dir}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
