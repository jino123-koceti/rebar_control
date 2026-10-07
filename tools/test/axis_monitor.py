#!/usr/bin/env python3
"""상부 4축의 속도·전류·온도를 CSV 로 **흘려쓰며** 감시한다.

⚠ 끝날 때 한 번에 쓰지 않는 이유: 2026-10-07 에 그렇게 짰다가 300초를 기다려야
  했고, 중간에 끊으니 **데이터가 전부 날아갔다.** 긴 시험은 언제 끝날지 모르므로
  **언제든 읽고 언제든 끊을 수 있어야** 한다. 그래서 한 줄씩 flush 한다.

전류 해석에 필요한 것
  · X 는 2026-10-04 에 **X4-10 → X4-36** 으로 교체됐다. 감속비가 12.5 → 36 이라
    같은 힘에 전류가 2.88배 **덜** 흐르고 정격도 7.8A → 6.1A 다. 교체 전 값과
    직접 비교하면 안 된다.
  · 기준선 (2026-10-07 호밍 실측, 이동 중 최대/평균):
      X 2.04 / 1.38 A   Y 3.75 / 2.24 A   Z 1.45 / 0.46 A   yaw 2.67 / 1.36 A

사용
  python3 axis_monitor.py --out /tmp/axmon.csv --sec 600
  python3 axis_monitor.py --summary /tmp/axmon.csv      (언제든 집계)
"""

import argparse
import csv
import os
import socket
import struct
import sys
import time

FMT = "IB3x8s"
CMD_STATUS2 = 0x9C
# 축 → (이름, 데이터시트 정격 A, mm/출력축도)
AXES = {
    0x145: ('x', 6.1, 0.290617),      # RMD-X4-36 (2026-10-04 교체)
    0x146: ('y', 7.8, 0.222711),
    0x147: ('z', 7.8, 0.090606),
    0x148: ('yaw', 7.8, None),
}


def read_state(sock, cid):
    """0x9C 한 번. 응답 ID 가 cid+0x100 인 것만 인정한다.

    엄격히 걸러야 한다 — 제어 노드가 같은 버스에 **명령**을 쏘고 있어서, 느슨한
    필터는 그 명령 프레임을 응답으로 오인한다 (2026-10-06 에 그렇게 오진했다).
    """
    for _ in range(2):
        sock.settimeout(0.008)
        while True:
            try:
                sock.recv(16)
            except OSError:
                break
        sock.send(struct.pack(FMT, cid, 8, bytes([CMD_STATUS2, 0, 0, 0, 0, 0, 0, 0])))
        end = time.time() + 0.03
        while time.time() < end:
            sock.settimeout(max(0.003, end - time.time()))
            try:
                raw = sock.recv(16)
            except OSError:
                break
            rid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            d = raw[8:16]
            if rid == cid + 0x100 and d[0] == CMD_STATUS2:
                return (d[1], struct.unpack("<h", d[2:4])[0] * 0.01,
                        struct.unpack("<h", d[4:6])[0])
    return None


def run(path, seconds, interface):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['t_s', 'axis', 'current_a', 'speed_dps', 'temp_c'])
            f.flush()
        t0 = time.time()
        n = 0
        try:
            while time.time() - t0 < seconds:
                for cid, (nm, _r, _k) in AXES.items():
                    st = read_state(sock, cid)
                    if st is None:
                        continue
                    w.writerow([f"{time.time() - t0:.3f}", nm,
                                f"{st[1]:.2f}", st[2], st[0]])
                    n += 1
                f.flush()          # ⚠ 매 바퀴 flush — 끊겨도 남는다
                time.sleep(0.005)
        except KeyboardInterrupt:
            pass
    sock.close()
    print(f"{n}행 기록 → {path}")
    return 0


def summary(path):
    rows = {}
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            rows.setdefault(r['axis'], []).append(
                (float(r['t_s']), abs(float(r['current_a'])),
                 int(r['speed_dps']), int(r['temp_c'])))
    if not rows:
        print("데이터가 없습니다.")
        return 1
    rated = {nm: v[1] for nm, v in
             ((n, (c, ra, k)) for c, (n, ra, k) in AXES.items())}
    mmpd = {nm: k for _c, (nm, _ra, k) in AXES.items()}
    print(f"{'축':5s}{'샘플':>7s}{'이동중':>7s}{'최대속도':>11s}"
          f"{'최대전류':>9s}{'정격비':>8s}{'평균(이동중)':>13s}{'온도':>10s}")
    for nm in ('x', 'y', 'z', 'yaw'):
        rs = rows.get(nm)
        if not rs:
            continue
        mv = [r for r in rs if abs(r[2]) > 2]
        pk = max((r[1] for r in mv), default=0.0)
        sp = max((abs(r[2]) for r in mv), default=0)
        av = (sum(r[1] for r in mv) / len(mv)) if mv else 0.0
        k = mmpd[nm]
        spd = f"{sp}dps" + (f"/{sp * k:.0f}mm/s" if k else "")
        print(f"{nm:5s}{len(rs):>7d}{len(mv):>7d}{spd:>11s}"
              f"{pk:>8.2f}A{pk / rated[nm] * 100:>7.0f}%{av:>12.2f}A"
              f"{min(r[3] for r in rs):>6d}→{max(r[3] for r in rs)}C")
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out')
    p.add_argument('--sec', type=float, default=600.0)
    p.add_argument('--interface', default='can2')
    p.add_argument('--summary')
    a = p.parse_args()
    if a.summary:
        return summary(a.summary)
    if not a.out:
        print("--out 또는 --summary 가 필요합니다.")
        return 1
    return run(a.out, a.sec, a.interface)


if __name__ == '__main__':
    sys.exit(main())
