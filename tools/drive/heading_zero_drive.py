#!/usr/bin/env python3
"""heading 영점 확정용 직진 시험 — 실제 궤적으로 요 오차를 잰다.

## 왜 이게 필요한가
카메라가 읽는 가로철근 각도에는 **요 오차 + 카메라 장착 롤 + 추정기 편향 +
격자 자체의 비뚤어짐**이 전부 섞여 있고, 한 장의 각도만으로는 못 가른다.
전·후방 카메라로 교차검증도 안 된다 — 같은 요를 **같은 부호**로 읽기 때문
(이미지 180° 회전은 직선 각도를 보존, 2026-08-13·08-19 두 번 실측).
측정 2개 vs 미지수 5개(θ, b_f, b_b, g_f, g_b)라 원리적으로 풀 수 없다.

**주행 궤적은 카메라를 안 거친다.** 실제로 옆으로 밀렸는지는 장착 각도나
어떤 철근이 검출됐는지와 무관한 물리적 사실이다. 그래서 영점의 기준이 된다.

    이탈량 = 주행거리 × tan(θ)      →   2m에 35mm 밀리면 θ = 1.0°

## 절차
  1) 주행방향 철근(세로바) 하나를 기준으로 정하고, 궤도 바깥면까지 거리를 잰다
  2) 이 스크립트로 조향 없이 직진 (1m 지점에서 한 번 멈춰 중간 측정)
  3) 같은 철근까지 거리를 다시 잰다 → 이탈량
  4) θ = atan(이탈/거리) 만큼 로봇을 반대로 돌린다  → 이제 격자와 정렬됨
  5) heading_offset_calib.py 로 그 자세의 카메라 값을 받아적는다 = 영점

## ⚠ 이탈의 원인은 둘이다 — 반드시 구분할 것
    요 오차   : 궤적이 **직선**, 이탈이 거리에 **비례**   ← 우리가 재려는 것
    궤도 불일치: 궤적이 **원호**,  이탈이 거리의 **제곱**에 비례
  1m와 2m 두 지점에서 재면 갈린다(2배면 요, 4배면 궤도).
  주행 중 heading 값이 **변하는지**로도 갈린다 — 이 스크립트가 자동 기록한다.

## 안전
  · 조향 명령을 내지 않는다(angular.z = 0 고정)
  · deck_edge STOP이면 즉시 중단 (배근 밖으로 나가지 않는다)
  · 범퍼가 눌리면 즉시 중단
  · 거리 도달·시간 초과·Ctrl+C 어느 쪽이든 정지 명령을 낸다

사용:
    python3 tools/drive/heading_zero_drive.py --dist 2.0 --speed 0.10
    python3 tools/drive/heading_zero_drive.py --dist 2.0 --dry     # 명령 없이 관찰만
"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ZeroDrive(Node):
    def __init__(self, a):
        super().__init__('heading_zero_drive')
        self.a = a
        self.x0 = None
        self.pose = None
        self.hd = {}                 # cam -> [(dist, heading)]
        self.verdict = 'WAIT'
        self.bumper = False
        self.marks = {}              # 중간 기록 지점
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode = self.create_publisher(String, '/control_mode', 10)
        self.dir_pub = self.create_publisher(String, '/travel_direction', 10)
        self.create_subscription(PoseStamped, '/encoder_odom', self._odom, 10)
        self.create_subscription(String, '/deck_edge_status', self._status, 10)
        self.create_subscription(String, '/bumper_block', self._bump, 10)

    def _odom(self, m):
        # ⚠ `/encoder_odom`은 **PoseStamped**다(nav_msgs/Odometry 아님).
        #   2026-09-15 첫 실행에서 발견 — Odometry로 구독해 매칭이 안 돼
        #   "❌ /encoder_odom 수신 없음"으로 즉시 중단됐다. 퍼블리셔는 있는데
        #   타입이 달라 0건이었던 것. encoder_odom.py:89 / rebar_drive_node도 이 타입.
        p = m.pose.position
        q = m.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z), 1 - 2*(q.z*q.z)))
        if self.x0 is None:
            self.x0 = (p.x, p.y, yaw)
        self.pose = (p.x, p.y, yaw)

    def _status(self, m):
        try:
            r = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.verdict = r.get('verdict', '?')
        h, cam = r.get('heading_deg'), r.get('cam')
        if h is not None and cam and self.x0 is not None:
            self.hd.setdefault(cam, []).append((self.travelled(), float(h)))

    def _bump(self, m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.bumper = any(d.get(k) for k in ('forward', 'backward', 'left', 'right'))

    def travelled(self):
        if self.x0 is None or self.pose is None:
            return 0.0
        return math.hypot(self.pose[0]-self.x0[0], self.pose[1]-self.x0[1])

    def stop(self):
        for _ in range(5):
            self.cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dist', type=float, default=2.0, help='주행 거리(m)')
    ap.add_argument('--speed', type=float, default=0.10, help='속도(m/s)')
    ap.add_argument('--mark', type=float, default=1.0, help='중간 측정 지점(m)')
    ap.add_argument('--back', action='store_true', help='후진으로 시험')
    ap.add_argument('--dry', action='store_true', help='명령 없이 관찰만')
    ap.add_argument('--timeout', type=float, default=120.0)
    a = ap.parse_args()

    rclpy.init()
    n = ZeroDrive(a)
    sign = -1.0 if a.back else 1.0
    cam = 'back' if a.back else 'front'

    print(f"■ heading 영점 직진 시험  {'(후진)' if a.back else '(전진)'}"
          f"{'  ※ DRY' if a.dry else ''}")
    print(f"  목표 {a.dist:.1f}m,  속도 {a.speed:.2f}m/s,  중간표시 {a.mark:.1f}m")
    print(f"  ⚠ 조향 명령 없음(angular.z=0). deck_edge STOP·범퍼면 즉시 중단.\n")

    # 방향 알리고 auto 모드 진입
    #   ⚠ 3초는 이 장비에서 짧다 — 카메라 4대가 붙어 있으면 DDS 디스커버리가
    #     그 안에 안 끝나 첫 샘플을 놓친다. odom이 잡히면 바로 빠져나간다.
    t0 = time.time()
    while time.time() - t0 < 10.0:
        m = String(); m.data = 'backward' if a.back else 'forward'
        n.dir_pub.publish(m)
        m2 = String(); m2.data = 'auto'
        n.mode.publish(m2)
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.x0 is not None and time.time() - t0 > 3.0:
            break                      # 모드/방향은 3초는 알려두고 출발

    if n.x0 is None:
        print('❌ /encoder_odom 수신 없음 — 중단\n'
              '   확인:  ros2 topic hz /encoder_odom\n'
              '   (타입은 geometry_msgs/PoseStamped 여야 한다)')
        n.destroy_node(); rclpy.shutdown(); return

    print('  ▶ 기준 거리를 재두셨나요? 지금 출발합니다.\n')
    t0 = time.time()
    last = 0.0
    try:
        while time.time() - t0 < a.timeout:
            rclpy.spin_once(n, timeout_sec=0.05)
            d = n.travelled()

            if n.bumper:
                print('\n🛑 범퍼 감지 → 중단'); break
            if n.verdict == 'STOP':
                print(f'\n🛑 deck_edge STOP (배근 끝/장애물) → 중단  [{d:.3f}m]'); break
            if d >= a.dist:
                print(f'\n✅ 목표 도달 {d:.3f}m'); break

            if a.mark > 0 and d >= a.mark and 'mark' not in n.marks:
                n.marks['mark'] = d
                n.stop()
                print(f'\n⏸  중간 지점 {d:.3f}m — **여기서 이탈을 재세요.**')
                input('   측정 끝나면 Enter → 계속 주행: ')
                t0 = time.time()

            if not a.dry:
                t = Twist(); t.linear.x = sign * a.speed; t.angular.z = 0.0
                n.cmd.publish(t)
            if d - last >= 0.1:
                last = d
                hs = n.hd.get(cam, [])
                h = hs[-1][1] if hs else float('nan')
                print(f'   {d:5.3f}m   heading={h:+6.2f}°   {n.verdict}')
    except KeyboardInterrupt:
        print('\n중단(Ctrl+C)')
    finally:
        n.stop()
        m = String(); m.data = 'idle'; n.mode.publish(m)
        rclpy.spin_once(n, timeout_sec=0.1)

    # ── 결과 ────────────────────────────────────────────
    d = n.travelled()
    print(f'\n{"="*60}\n■ 결과')
    print(f'  주행거리(엔코더) {d:.3f}m')
    hs = n.hd.get(cam, [])
    if len(hs) >= 4:
        v = [x[1] for x in hs]
        half = len(hs)//2
        h1 = sorted(v[:half])[half//2]
        h2 = sorted(v[half:])[(len(v)-half)//2]
        print(f'  카메라 heading({cam}): 전반 {h1:+.2f}°  후반 {h2:+.2f}°  '
              f'변화 {h2-h1:+.2f}°')
        print(f'     (변화가 작으면 **요 오차**, 계속 커지면 **궤도가 휘는 것**)')
    # ⚠ 거의 못 간 주행으로 θ를 뽑으면 안 된다. `d*1000 or 1`로 0을 막아도
    #   1mm로 나눠 "이탈 10mm → θ=84°" 같은 헛값이 나온다(dry-run에서 실제로 봤다).
    #   이탈/거리는 거리가 짧을수록 오차가 증폭되므로 최소 주행거리를 요구한다.
    MIN_D = 0.3
    if d < MIN_D:
        print(f'\n  ⚠ 주행거리 {d:.3f}m — {MIN_D}m 미만이라 θ를 낼 수 없다.')
        print(f'     (이탈/거리는 거리가 짧을수록 오차가 증폭된다. 다시 주행할 것)')
        n.destroy_node(); rclpy.shutdown(); return
    print(f'\n  ▶ 이제 이탈량을 재세요. 그리고:')
    print(f'       θ = atan(이탈mm / {d*1000:.0f}mm)')
    for dev in (10, 20, 35, 50):
        print(f'         이탈 {dev:3d}mm → θ = {math.degrees(math.atan(dev/(d*1000))):.2f}°')
    print(f'  ▶ θ만큼 로봇을 반대로 돌린 뒤:')
    print(f'       python3 tools/vision_test/heading_offset_calib.py')
    print(f'     그때 나온 값이 **영점**입니다.')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
