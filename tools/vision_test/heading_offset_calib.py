#!/usr/bin/env python3
"""heading 영점(카메라 장착 롤) 캡처 — 전·후방 각각.

왜 필요한가:
  보정 없이 조향을 켜면 컨트롤러가 '로봇 정렬'이 아니라 '영상 수평'을 목표로 삼는다.
  정렬된 상태에서 읽히는 값(장착 롤 + 추정기 편향)을 빼줘야 한다.

★★ 모델 정정 (2026-08-19) — 예전 주석의 `front ≈ -back` 가정은 **틀렸다.**
  전·후방은 시선이 반대지만 **단일축 미러가 아니라 이미지 180° 회전**이고,
  180° 회전은 직선의 각도를 보존한다. 그래서 같은 요 틀어짐을 두 카메라가
  **같은 부호**로 읽는다. 실측 2회로 확인:
    2026-08-13  front +3.18→-0.05 (Δ-3.23),  back +1.49→-0.92 (Δ-2.41)
    2026-08-19  front -1.34→-3.44 (Δ-2.10),  back +1.28→-2.74 (Δ-4.02)
  두 번 다 **같은 방향**으로 움직였다.

  올바른 모델:  front = g_f·θ + b_f,   back = g_b·θ + b_b
    측정 2개 vs 미지수 5개(θ, b_f, b_b, g_f, g_b).
    ⚠ **로봇을 아무리 돌려가며 재도 절대 영점(θ=0)은 나오지 않는다.**
       두 카메라가 같은 부호라 서로를 견제하지 못한다.
    → θ=0은 **사람이 물리적으로 정해줘야만** 정의된다. 이 스크립트는 그 상태에서
       읽히는 값을 받아적을 뿐이고, 정렬 판단의 책임은 전적으로 사람에게 있다.

  ⚠ 이득(g_f, g_b)도 **추정기를 바꾸면 달라진다.** 2026-08-19 heading을 스팬 가중
     중앙값으로 바꿨으므로, 그 이전에 잡은 오프셋은 재사용할 수 없다.

★ 더 믿을 만한 영점 확인법: **직진 주행 후 횡방향 이탈 측정.**
  영점이 틀어져 있으면 로봇이 한쪽으로 밀린다. 이탈량/거리 = tan(오차각).
  카메라끼리 비교하는 것보다 이쪽이 실제로 원하는 것(격자 따라 곧게 가기)에 직결된다.

사용법:
  1) 로봇을 배근 격자에 **눈으로 봐서 반듯하게** 정렬시켜 세운다 (주행방향 ∥ 세로철근)
  2) deck_edge_node 실행 중인 상태에서:
       python3 tools/vision_test/heading_offset_calib.py
  3) 출력된 파라미터를 rebar_drive 실행 시 그대로 붙인다

⚠ 이 스크립트는 `/travel_direction`을 발행해 deck_edge의 활성 카메라를 전환한다
   (active_only 모드라 진행방향만 추론하기 때문). 주행 중에는 돌리지 말 것 —
   rebar_drive_node도 같은 토픽을 쏘므로 서로 덮어쓴다.
"""
import argparse
import json
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Collector(Node):
    def __init__(self):
        super().__init__('heading_offset_calib')
        self.samples = {'front': [], 'back': []}
        self.bars = {'front': [], 'back': []}
        self.dir_pub = self.create_publisher(String, '/travel_direction', 10)
        self.create_subscription(String, '/deck_edge_status', self._cb, 10)

    def _cb(self, msg):
        try:
            r = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        cam, hd = r.get('cam'), r.get('heading_deg')
        if cam in self.samples and hd is not None:
            self.samples[cam].append(float(hd))
            self.bars[cam].append(int(r.get('heading_bars', 0)))

    def collect(self, cam, n, timeout):
        """해당 방향으로 전환하고 n샘플 수집."""
        direction = 'forward' if cam == 'front' else 'backward'
        self.samples[cam].clear()
        self.bars[cam].clear()
        t0 = time.time()
        while time.time() - t0 < timeout and len(self.samples[cam]) < n:
            m = String(); m.data = direction
            self.dir_pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.3)
        # 전환 직후 1~2 샘플은 반대방향 카메라 잔상일 수 있어 앞을 버린다
        return self.samples[cam][2:], self.bars[cam][2:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=12, help='방향별 샘플 수')
    ap.add_argument('--timeout', type=float, default=25.0)
    a = ap.parse_args()

    rclpy.init()
    node = Collector()
    print('※ 로봇이 격자에 정렬돼 정지해 있어야 합니다. 수집 시작…\n')
    out = {}
    for cam in ('front', 'back'):
        vals, bars = node.collect(cam, a.n + 2, a.timeout)
        if len(vals) < 3:
            print(f'❌ {cam}: 샘플 부족({len(vals)}개). '
                  f'deck_edge_node와 카메라를 확인하세요.')
            out[cam] = None
            continue
        med = statistics.median(vals)
        out[cam] = med
        print(f'{cam:5s}: 중앙값 {med:+.2f}°  (n={len(vals)}, '
              f'범위 {min(vals):+.2f}~{max(vals):+.2f}, '
              f'가로철근 {min(bars)}~{max(bars)}개)')

    node.destroy_node()
    rclpy.shutdown()

    if out['front'] is None or out['back'] is None:
        return
    f, b = out['front'], out['back']
    print('\n⚠ front와 back은 **같은 부호**로 읽힌다(이미지 180° 회전).')
    print('   두 값을 더하거나 비교해서 정렬 여부를 판단할 수 없다 — 위 주석 참조.')
    print('   이 값들은 **지금 자세가 정렬이라는 사람의 판단**을 그대로 받아적은 것이다.')
    print(f'\n▶ rebar_drive 실행 시 붙일 파라미터:\n'
          f'  -p heading_offset_front_deg:={f:.2f} '
          f'-p heading_offset_back_deg:={b:.2f}')


if __name__ == '__main__':
    main()
