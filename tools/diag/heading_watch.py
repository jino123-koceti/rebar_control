#!/usr/bin/env python3
"""헤딩 제어 응답 실측 — 스텝마다 heading이 얼마나 줄었는지 잰다.

`/deck_edge_status`   heading_deg(raw), heading_bars
`/cmd_vel`            실제 나간 angular.z (조향 지령)
`/rebar_drive/state`  스텝 경계

★ 목표는 0이 아니라 **오프셋**이다 — raw heading이 오프셋과 같으면 물리적 직진.
  그래서 `--front` / `--back` 으로 그날 쓰는 오프셋을 넣어야 판정이 맞다.
  (2026-09-02: back 오프셋이 2.21° 틀려 후진마다 좌측으로 틀어졌다.
   2026-09-08: 카메라를 ZED X로 교체 → 이전 오프셋 전부 무효, 재측정 필요.)

## 사용
    python3 tools/diag/heading_watch.py                    # 오프셋 0/0
    python3 tools/diag/heading_watch.py --front -0.86 --back -0.69
"""
import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

DRIVING = ('FWD_DETECT', 'FWD_STEP', 'FWD_SETTLE',
           'REV_DETECT', 'REV_STEP', 'REV_SETTLE')
STEP = ('FWD_STEP', 'REV_STEP')


class W(Node):
    def __init__(self, target):
        super().__init__('heading_watch')
        self.target = target
        self.create_subscription(String, '/deck_edge_status', self.st, 10)
        self.create_subscription(String, '/rebar_drive/state', self.sm, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cv, 10)
        self.hd = None
        self.bars = 0
        self.state = 'IDLE'
        self.step_h0 = None
        self.wz = []
        self.n = 0
        print(f'▶ 헤딩 감시 시작 — 목표 FWD {target["FWD_STEP"]:+.2f}° / '
              f'REV {target["REV_STEP"]:+.2f}°   (S20 → S23으로 시작)', flush=True)

    def st(self, m):
        import json
        d = json.loads(m.data)
        if d.get('heading_deg') is not None:
            self.hd = d['heading_deg']
            self.bars = d.get('heading_bars', 0)

    def cv(self, m):
        if self.state in STEP:
            self.wz.append(m.angular.z)

    def sm(self, m):
        s = m.data
        if s == self.state:
            return
        old, self.state = self.state, s
        if s in STEP:
            self.step_h0 = self.hd
            self.wz = []
        elif old in STEP:
            h0, h1 = self.step_h0, self.hd
            if h0 is None or h1 is None:
                return
            self.n += 1
            w = statistics.mean(self.wz) if self.wz else 0.0
            wmax = max((abs(x) for x in self.wz), default=0.0)
            tg = self.target.get(old, 0.0)
            e0, e1 = h0 - tg, h1 - tg
            corr = abs(e0) - abs(e1)
            mark = '✅' if corr > 0.05 else ('—' if abs(corr) <= 0.05 else '❌ 악화')
            print(f'  #{self.n:2}  {old:10} raw {h0:+6.2f}→{h1:+6.2f}  '
                  f'오차(목표{tg:+.2f}) {e0:+6.2f}→{e1:+6.2f}  보정 {corr:+5.2f}°  '
                  f'angular 평균 {w:+.3f} 최대 {wmax:.3f}  bars {self.bars}  {mark}',
                  flush=True)
        if s in ('ABORT', 'DONE'):
            print(f'■ 상태 {s}', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--front', type=float, default=0.0, help='전진 heading 오프셋(°)')
    ap.add_argument('--back', type=float, default=0.0, help='후진 heading 오프셋(°)')
    a = ap.parse_args()
    rclpy.init()
    n = W({'FWD_STEP': a.front, 'REV_STEP': a.back})
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
