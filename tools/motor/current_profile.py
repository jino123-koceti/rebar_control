#!/usr/bin/env python3
"""
전류-각도 프로파일 스캔 — 걸리는 지점 특정 / 전류 상한 근거 확보 / 열화 비교 기준

0x9C 응답 한 번에 [온도][전류 i16][속도 i16][각도 i16] 이 모두 들어있어
질의 1회로 동시 샘플링한다.

판정:
  * 규칙적 주기 패턴  → 코깅 토크 (정상)
  * 특정 각도에만 피크 → 기계적 간섭 (해당 위치 점검 필요)

사용:
  python3 current_profile.py --motor 0x143 --dps 20 --turns 1
  python3 current_profile.py --motor 0x144 --dps 20 --turns 1 --out prof_144.json
"""

import argparse
import json
import math
import socket
import struct
import sys
import time

FMT = "IB3x8s"
CMD_STATUS2 = 0x9C
CMD_SPEED = 0xA2
CMD_STOP = 0x81
CMD_SHUTDOWN = 0x80


class Bus:
    def __init__(self, iface):
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((iface,))
        self.s.settimeout(0.15)

    def drain(self):
        self.s.settimeout(0.01)
        try:
            while True:
                self.s.recv(16)
        except OSError:
            pass
        self.s.settimeout(0.15)

    def send(self, cid, data8):
        self.s.send(struct.pack(FMT, cid, 8, bytes(data8)))

    def sample(self, cid):
        """0x9C 1회 → (전류 A, 속도 dps, 각도 deg, 온도 C)"""
        self.drain()
        self.send(cid, [CMD_STATUS2, 0, 0, 0, 0, 0, 0, 0])
        t0 = time.time()
        while time.time() - t0 < 0.15:
            try:
                raw = self.s.recv(16)
            except OSError:
                break
            rid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            d = raw[8:16]
            if rid == cid + 0x100 and d[0] == CMD_STATUS2:
                cur = int.from_bytes(d[2:4], "little", signed=True) / 100.0
                spd = int.from_bytes(d[4:6], "little", signed=True)
                ang = int.from_bytes(d[6:8], "little", signed=True)
                return cur, spd, ang, d[1]
        return None

    def speed(self, cid, dps):
        data = bytearray(8)
        data[0] = CMD_SPEED
        data[4:8] = int(round(dps * 100)).to_bytes(4, "little", signed=True)
        self.send(cid, data)

    def stop(self, cid, shutdown=False):
        self.speed(cid, 0)
        time.sleep(0.05)
        self.send(cid, [CMD_SHUTDOWN if shutdown else CMD_STOP, 0, 0, 0, 0, 0, 0, 0])


def bar(v, vmax, width=40):
    n = 0 if vmax <= 0 else int(round(v / vmax * width))
    return "█" * n


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interface", default="can2")
    p.add_argument("--motor", type=lambda x: int(x, 0), required=True)
    p.add_argument("--dps", type=float, default=20.0)
    p.add_argument("--turns", type=float, default=1.0)
    p.add_argument("--abort", type=float, default=3.5, help="전류 상한 A")
    p.add_argument("--bin", type=float, default=10.0, help="각도 구간 (deg)")
    p.add_argument("--out", help="원본 샘플 저장 경로 (JSON)")
    a = p.parse_args()

    bus = Bus(a.interface)
    cid = a.motor
    dur = abs(a.turns) * 360.0 / abs(a.dps)

    s0 = bus.sample(cid)
    if s0 is None:
        print(f"✗ 0x{cid:03X} 응답 없음")
        return 1
    print(f"0x{cid:03X} 전류 프로파일: {a.dps:+.0f} dps × {a.turns}회전 "
          f"(약 {dur:.1f}초), 상한 {a.abort}A")
    print(f"  시작 각도 {s0[2]}°, 온도 {s0[3]}°C")

    samples = []
    aborted = False
    bus.speed(cid, a.dps)
    t0 = time.time()
    try:
        while time.time() - t0 < dur:
            s = bus.sample(cid)
            if s is None:
                continue
            cur, spd, ang, temp = s
            samples.append({"t": round(time.time() - t0, 4), "cur": cur,
                            "spd": spd, "ang": ang, "temp": temp})
            if abs(cur) > a.abort:
                print(f"  ⚠️ 전류 {cur:+.2f}A > {a.abort}A — 각도 {ang}° 에서 중단")
                aborted = True
                break
    finally:
        bus.stop(cid)

    time.sleep(0.3)
    s1 = bus.sample(cid)
    print(f"  종료 각도 {s1[2] if s1 else '?'}°, 샘플 {len(samples)}개 "
          f"({len(samples)/max(dur,0.01):.0f} Hz)")

    if not samples:
        print("✗ 샘플 없음")
        return 1

    # 각도 구간별 집계 (시작각 기준 상대 각도)
    base = s0[2]
    bins = {}
    for s in samples:
        rel = (s["ang"] - base) % 360.0
        k = int(rel // a.bin)
        b = bins.setdefault(k, [])
        b.append(abs(s["cur"]))

    curs = [abs(s["cur"]) for s in samples]
    vmax = max(curs)
    vmean = sum(curs) / len(curs)
    vmed = sorted(curs)[len(curs) // 2]
    print(f"\n  전류: 평균 {vmean:.2f}A  중앙 {vmed:.2f}A  최대 {vmax:.2f}A")

    print(f"\n  {'구간(시작각 기준)':<18}{'최대':>7}{'평균':>7}  프로파일")
    peaks = []
    for k in sorted(bins):
        v = bins[k]
        mx, mn = max(v), sum(v) / len(v)
        lo = k * a.bin
        print(f"  {lo:5.0f}°~{lo+a.bin:5.0f}°  {mx:6.2f}A {mn:6.2f}A  {bar(mx, vmax)}")
        peaks.append((lo, mx, mn))

    # 판정: 국소 이상만 잡는다.
    # 구간 '평균'끼리 비교해야 한다 — 구간 '최대'를 전체 중앙값과 비교하면
    # 전류 리플 때문에 거의 모든 구간이 걸려서 판정이 무의미해진다.
    print()
    means = [mn for _, _, mn in peaks]
    m_med = sorted(means)[len(means) // 2]
    m_max = max(means)
    # 이웃 대비 튀는 구간만 국소 이상으로 본다
    hot = []
    n = len(peaks)
    for i, (lo, mx, mn) in enumerate(peaks):
        nb = [peaks[(i - 1) % n][2], peaks[(i + 1) % n][2]]
        if mn > 2.0 * (sum(nb) / 2) and mn > m_med * 1.5:
            hot.append((lo, mx, mn))

    print(f"  구간 평균: 중앙 {m_med:.2f}A, 최대 {m_max:.2f}A "
          f"(변동폭 {m_max - min(means):.2f}A)")
    if not hot:
        print(f"  판정: 국소 이상 없음 — 이웃 구간 대비 2배 넘게 튀는 지점이 없다.")
        print(f"        회전 1주기로 완만히 변하는 형태는 축 정렬 편심이나 무게중심")
        print(f"        변화로, 기계적 간섭이 아니다. 코깅은 이보다 훨씬 촘촘하다.")
        print(f"        → 전류 상한은 실측 최대 {vmax:.2f}A 에 여유를 둔 값으로 설정")
    else:
        print(f"  판정: 국소 이상 {len(hot)}개 — 해당 위치 점검 권장")
        for lo, mx, mn in hot:
            print(f"        {lo:.0f}°~{lo+a.bin:.0f}°  평균 {mn:.2f}A  최대 {mx:.2f}A")
    if aborted:
        print(f"\n  ⚠️ 상한 초과로 중단됨 — 프로파일이 전체 구간을 덮지 못함")

    if a.out:
        json.dump({"motor": hex(cid), "dps": a.dps, "turns": a.turns,
                   "start_angle": base, "aborted": aborted,
                   "samples": samples}, open(a.out, "w"))
        print(f"\n  원본 저장: {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
