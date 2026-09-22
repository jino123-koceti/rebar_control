#!/usr/bin/env python3
"""GMSL 카메라 4대 **장시간 내구 시험** — 부팅 후 몇 분 만에 처음 무너지나.

## 왜 (2026-09-22)
9/14 5분 시험은 오류 0건으로 통과했는데, 같은 4대 구성이 오늘 가동 6~26분 뒤에 3번 무너졌다
(좌측 모노 NOT INITIALIZED → 스테레오 CORRUPTED/REBOOTING). 3대는 8시간 무사.

## 방법 (운용과 비슷한 부하로)
  · 1분마다 **5초만** 구독해 4대 수신 Hz를 잰다 (상시 구독은 인코딩 부하를 더해 조건이 달라진다).
    ZED 드라이버는 구독자와 무관하게 매 프레임 grab 하므로 GMSL/캡처 부하는 동일하다.
  · journalctl 을 계속 따라가며 카메라 오류 키워드를 **발생 시각·부팅 후 경과 분**으로 기록.
  · 로봇은 정지. 모터 전원 ON 상태에서 부팅.

    python3 tools/diag/gmsl4_endurance.py --minutes 90
로그: data/logs/gmsl4_endurance/<시각>.csv  (+ 같은 이름 _events.txt)
"""
import argparse
import os
import re
import subprocess
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

CAMS = {'L': 'zedxone_left', 'R': 'zedxone', 'F': 'zedxmini2', 'B': 'zedxmini1'}
PAT = re.compile(r'(zedxone_left|zedxone|zedxmini1|zedxmini2)\.zed_node\].*?'
                 r'(CORRUPTED FRAME|CAMERA REBOOTING|NOT INITIALIZED|FAILURE|process has died)')
DIED = re.compile(r'process has died.*(zedxone_left|zedxone|zedxmini)')


def uptime_min():
    return float(open('/proc/uptime').read().split()[0]) / 60.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=90)
    ap.add_argument('--every', type=float, default=60.0)
    a = ap.parse_args()
    os.makedirs('data/logs/gmsl4_endurance', exist_ok=True)
    stem = f"data/logs/gmsl4_endurance/{time.strftime('%Y%m%d_%H%M%S')}"
    csvf = open(stem + '.csv', 'w')
    csvf.write('time,uptime_min,L,R,F,B\n')
    evf = open(stem + '_events.txt', 'w')
    first = {}
    lock = threading.Lock()

    def follow():
        p = subprocess.Popen(['journalctl', '-u', 'robot-control', '-f', '-n', '0', '-o', 'short'],
                             stdout=subprocess.PIPE, text=True)
        for line in p.stdout:
            m = PAT.search(line) or DIED.search(line)
            if not m:
                continue
            cam = m.group(1)
            kind = m.group(2) if m.lastindex >= 2 else 'died'
            key = (cam, kind)
            with lock:
                if key not in first:
                    first[key] = uptime_min()
                    msg = (f"{time.strftime('%H:%M:%S')}  부팅 후 {first[key]:5.1f}분  "
                           f"{cam:13s} {kind}")
                    print('⚠ ' + msg, flush=True)
                    evf.write(msg + '\n')
                    evf.flush()
    threading.Thread(target=follow, daemon=True).start()

    rclpy.init()
    n = rclpy.create_node('gmsl4_endurance')
    t_end = time.monotonic() + a.minutes * 60
    print(f'시작: 부팅 후 {uptime_min():.1f}분 · {a.minutes:.0f}분 기록 → {stem}.csv', flush=True)
    while time.monotonic() < t_end:
        cnt = {k: 0 for k in CAMS}
        subs = [n.create_subscription(CompressedImage,
                                      f'/{ns}/zed_node/rgb/color/rect/image/compressed',
                                      lambda m, k=k: cnt.__setitem__(k, cnt[k] + 1),
                                      qos_profile_sensor_data) for k, ns in CAMS.items()]
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:          # 디스커버리
            rclpy.spin_once(n, timeout_sec=0.1)
        for k in cnt:
            cnt[k] = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        for s in subs:
            n.destroy_subscription(s)
        hz = {k: cnt[k] / 5.0 for k in CAMS}
        up = uptime_min()
        flag = ' '.join(f"{k}{'❌' if hz[k] < 0.5 else ''}" for k in CAMS if hz[k] < 0.5)
        print(f"{time.strftime('%H:%M:%S')} 부팅+{up:5.1f}분  " +
              '  '.join(f'{k} {hz[k]:4.1f}' for k in CAMS) + (f'   ← {flag}' if flag else ''),
              flush=True)
        csvf.write(f"{time.strftime('%H:%M:%S')},{up:.1f}," +
                   ','.join(f'{hz[k]:.1f}' for k in CAMS) + '\n')
        csvf.flush()
        while time.monotonic() - t0 < a.every and time.monotonic() < t_end:
            time.sleep(0.5)
    print('\n===== 처음 발생 (부팅 후 분) =====')
    for (cam, kind), m in sorted(first.items(), key=lambda kv: kv[1]):
        print(f'  {m:5.1f}분  {cam:13s} {kind}')
    if not first:
        print('  오류 없음')


if __name__ == '__main__':
    main()
