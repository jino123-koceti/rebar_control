#!/usr/bin/env python3
"""카메라 4대 + 모터 CAN + 횡이동을 **초 단위로 나란히** 기록 — "무엇이 무엇을 죽이나" 판별용.

## 왜 (2026-09-22)
횡이동 때 좌측 모노 카메라가 죽고(오전 11:05→11:07, 저녁 19:56→19:57), 같은 시간대에
스테레오 CORRUPTED FRAME·CAMERA REBOOTING, 모터 CAN 버스오프(1초 1회, 1,587회)가 겹쳤다.
제어기·캡처카드는 모터와 **배터리가 분리**돼 있다(실측: 모터 전원 OFF 후에도 스테레오 9분 정상).
→ 남는 후보: ① 케이블/커넥터 접촉(움직임) ② 접지·전자기 노이즈(큰 전류).
   흔들기 / 들어서 횡이동 / 바닥 횡이동을 하며 이 기록을 보면 갈린다.

## 한 줄 (1초마다)
  시각 | L R F B (카메라 Hz) | can2 상태·버스오프 증가 | 횡이동 0x143 활동·전류(iq 최대)

  · 카메라가 평소의 절반 밑으로 떨어지면 ⚠, 0이면 ❌
  · 횡이동 전류는 0xA9 응답의 iq(토크전류, 0.01A)에서 읽는다 (can_sender가 0x143만 0xA9 사용)

    python3 tools/diag/gmsl_can_watch.py                # Ctrl+C로 종료, 끝에 사건 요약
    python3 tools/diag/gmsl_can_watch.py --minutes 20

로그: data/logs/gmsl_can/<시각>.csv
"""
import argparse
import csv
import os
import struct
import threading
import time

import can
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

CAMS = {'L': 'zedxone_left', 'R': 'zedxone', 'F': 'zedxmini2', 'B': 'zedxmini1'}
NORMAL = {'L': 10.0, 'R': 10.0, 'F': 4.5, 'B': 4.5}   # 재부팅 직후 실측 근처
LAT_REPLY = 0x243


def can_stats(iface='can2'):
    base = f'/sys/class/net/{iface}/'
    try:
        st = os.popen(f'ip -d -s link show {iface}').read()
        state = st.split('can state ')[1].split()[0] if 'can state ' in st else '?'
        line = st.split('re-started')[1].splitlines()[1].split()
        return state, int(line[0]), int(line[5])        # 상태, 재시작, bus-off
    except Exception:
        return '?', -1, -1


class LatSniff(threading.Thread):
    """can2 수동 청취: 횡이동 모터 응답(0x243)과 전체 수신량."""
    def __init__(self):
        super().__init__(daemon=True)
        self.bus = can.Bus(interface='socketcan', channel='can2')
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        self.lat_frames = 0
        self.lat_iq_max = 0.0
        self.rx = 0

    def run(self):
        while True:
            m = self.bus.recv(0.5)
            if m is None:
                continue
            with self.lock:
                self.rx += 1
                if m.arbitration_id == LAT_REPLY and m.data[0] in (0xA9, 0xA4, 0x9C):
                    self.lat_frames += 1
                    iq = abs(struct.unpack('<h', bytes(m.data[2:4]))[0]) * 0.01
                    self.lat_iq_max = max(self.lat_iq_max, iq)

    def take(self):
        with self.lock:
            v = (self.lat_frames, self.lat_iq_max, self.rx)
            self.reset()
        return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=0)
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node('gmsl_can_watch')
    cnt = {k: 0 for k in CAMS}
    for k, ns in CAMS.items():
        n.create_subscription(CompressedImage, f'/{ns}/zed_node/rgb/color/rect/image/compressed',
                              lambda m, k=k: cnt.__setitem__(k, cnt[k] + 1), qos_profile_sensor_data)
    sn = LatSniff()
    sn.start()
    os.makedirs('data/logs/gmsl_can', exist_ok=True)
    path = f"data/logs/gmsl_can/{time.strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(path, 'w', newline='')
    wr = csv.writer(f)
    wr.writerow(['time', 'L', 'R', 'F', 'B', 'can_state', 'busoff_total', 'busoff_new',
                 'can_rx', 'lat_frames', 'lat_iq_max_A'])
    _, _, bo0 = can_stats()
    bo_prev = bo0
    events = []
    t0 = tick = time.monotonic()
    print(f'기록 시작 → {path}\n 시각      L    R    F    B   | CAN 상태      +버스오프 rx/s | 횡이동 iq')
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            now = time.monotonic()
            if now - tick < 1.0:
                continue
            dt = now - tick
            tick = now
            hz = {k: cnt[k] / dt for k in CAMS}
            for k in CAMS:
                cnt[k] = 0
            state, _, bo = can_stats()
            bnew = (bo - bo_prev) if bo >= 0 and bo_prev >= 0 else 0
            bo_prev = bo
            lf, iq, rx = sn.take()
            ts = time.strftime('%H:%M:%S')

            def mark(k):
                v = hz[k]
                return '❌' if v < 0.5 else ('⚠' if v < NORMAL[k] * 0.5 else ' ')
            lat = f'● {iq:4.1f}A' if lf else '  -'
            print(f'{ts} ' + ' '.join(f'{hz[k]:4.1f}{mark(k)}' for k in CAMS) +
                  f' | {state:13s} {"+"+str(bnew) if bnew else "  ":>4} {rx/dt:5.0f} | {lat}', flush=True)
            wr.writerow([ts] + [f'{hz[k]:.1f}' for k in CAMS] +
                        [state, bo, bnew, f'{rx/dt:.0f}', lf, f'{iq:.2f}'])
            f.flush()
            for k in CAMS:
                if hz[k] < 0.5:
                    events.append((ts, f'카메라 {k} 정지', lf > 0))
            if bnew:
                events.append((ts, f'CAN 버스오프 +{bnew}', lf > 0))
            if a.minutes and now - t0 > a.minutes * 60:
                break
    except KeyboardInterrupt:
        pass
    f.close()
    print('\n===== 사건 요약 (횡이동 중이었나) =====')
    seen = set()
    for ts, what, lat in events:
        key = (what.split()[0] + what.split()[1], ts[:5])
        if key in seen:
            continue
        seen.add(key)
        print(f'  {ts}  {what:18s} {"← 횡이동 중" if lat else ""}')
    print(f'버스오프 총 {bo_prev - bo0 if bo_prev >= 0 else "?"}회 · 로그 {path}')


if __name__ == '__main__':
    main()
