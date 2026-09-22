#!/usr/bin/env python3
"""리모콘 조종 중 **끊김 감시** — 수동주행 시뮬레이션/현장 이동용.

## 무엇을 보나 (1초마다 한 줄, 끊김은 즉시 경고)
  · /remote_control 수신 간격   : 0.3s 넘게 비면 drive_controller 워치독이 정지시키는 구간
  · can3 커널 카운터            : rx 패킷·오류·bus-off·재시작 (USB 수신기 자체 상태)
  · 벽시계 점프                 : 단조시계 대비 벽시계가 0.5s 이상 튀면 기록 (NTP step)
  · USB 끊김                    : /sys/class/net/can3 가 사라졌다 생기는지

    python3 tools/diag/remote_link_watch.py                 # Ctrl+C로 종료, 끝에 요약
    python3 tools/diag/remote_link_watch.py --minutes 30

로그: data/logs/remote_link/<시각>.csv
"""
import argparse
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import RemoteControl

GAP_WARN = 0.3          # drive_controller remote_timeout_sec 와 같게
IFACE = 'can3'


def can_stats():
    base = f'/sys/class/net/{IFACE}/statistics/'
    if not os.path.isdir(base):
        return None
    r = lambda k: int(open(base + k).read())
    return {'rx': r('rx_packets'), 'rx_err': r('rx_errors'), 'rx_drop': r('rx_dropped')}


class Watch(Node):
    def __init__(self):
        super().__init__('remote_link_watch')
        self.last = None
        self.gaps = []          # (시각, 길이)
        self.n = 0
        self.create_subscription(RemoteControl, '/remote_control', self._cb, 50)

    def _cb(self, _m):
        now = time.monotonic()
        if self.last is not None and now - self.last > GAP_WARN:
            g = now - self.last
            self.gaps.append((time.strftime('%H:%M:%S'), g))
            self.get_logger().warn(f'⚠ 리모콘 수신 공백 {g:.2f}s')
        self.last = now
        self.n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=0, help='0 = Ctrl+C까지')
    a = ap.parse_args()
    rclpy.init()
    w = Watch()
    os.makedirs('data/logs/remote_link', exist_ok=True)
    path = f"data/logs/remote_link/{time.strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(path, 'w', newline='')
    wr = csv.writer(f)
    wr.writerow(['time', 'hz', 'max_gap_s', 'can_rx_hz', 'can_rx_err', 'can_present', 'clock_jump_s'])
    t0 = time.monotonic()
    tick = t0
    wall0 = time.time() - t0
    prev = can_stats()
    prev_n = 0
    jumps, can_lost = [], 0
    print(f'감시 시작 → {path}  (공백 {GAP_WARN}s 초과 시 경고)')
    try:
        while rclpy.ok():
            rclpy.spin_once(w, timeout_sec=0.05)
            now = time.monotonic()
            if now - tick < 1.0:
                continue
            dt = now - tick
            tick = now
            # 벽시계 점프
            off = (time.time() - now) - wall0
            if abs(off) > 0.5:
                jumps.append((time.strftime('%H:%M:%S'), off))
                print(f'⚠ 벽시계 점프 {off:+.1f}s')
                wall0 += off
            st = can_stats()
            if st is None:
                can_lost += 1
                print(f'⚠ {IFACE} 없음 (USB 끊김?)')
            hz = (w.n - prev_n) / dt
            prev_n = w.n
            age = (now - w.last) if w.last else float('inf')
            crx = (st['rx'] - prev['rx']) / dt if (st and prev) else 0.0
            wr.writerow([time.strftime('%H:%M:%S'), f'{hz:.1f}', f'{age:.2f}', f'{crx:.1f}',
                         st['rx_err'] if st else '', int(st is not None), f'{off:+.2f}'])
            f.flush()
            if int(now - t0) % 10 == 0:
                print(f"{time.strftime('%H:%M:%S')} 리모콘 {hz:4.1f}Hz  can3 rx {crx:5.1f}/s  "
                      f"공백 누적 {len(w.gaps)}회")
            prev = st
            if a.minutes and now - t0 > a.minutes * 60:
                break
    except KeyboardInterrupt:
        pass
    dur = time.monotonic() - t0
    print('\n===== 요약 =====')
    print(f'감시 {dur/60:.1f}분, 리모콘 메시지 {w.n}개 (평균 {w.n/max(dur,1):.1f}Hz)')
    print(f'0.3s 초과 공백: {len(w.gaps)}회' + (f"  최대 {max(g for _, g in w.gaps):.2f}s" if w.gaps else ''))
    for t, g in w.gaps[:20]:
        print(f'   {t}  {g:.2f}s')
    print(f'{IFACE} 사라짐: {can_lost}초   벽시계 점프: {len(jumps)}회 {jumps[:5]}')
    print(f'로그: {path}')
    f.close()


if __name__ == '__main__':
    main()
