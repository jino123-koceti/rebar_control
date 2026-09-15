#!/usr/bin/env python3
"""
횡이동 2축 (0x143 #1, 0x144 #2) 동기 제어 도구 — 3차년도 장비

두 모터는 서로 마주보게 설치되어 있어, 기구를 같은 방향으로 보내려면
회전 부호가 반대여야 한다 (2026-09-08 육안 확인). +방향 = 좌측 횡이동.

동작 방식 (2차년도 로직과 동일):
  1) 각 모터를 가장 가까운 12시 위치로 정렬 (최단 경로, 360° 이내)
  2) 12시 기준으로 ±N회전 (1회전 = 리드스크류 1바퀴 = 50mm)

12시 각도는 모터마다 다르므로 lateral_home.json 에 저장한다.
  0x143: 33.50° (2025-12-26 캘리브레이션 값)
  0x144: 미측정 — set-home 으로 기록해야 함

사용 예:
  python3 lateral_dual.py status
  python3 lateral_dual.py jog --motor 0x144 --dps 10 --sec 0.5   # 12시 찾기
  python3 lateral_dual.py set-home --motor 0x144                 # 현재 위치를 12시로 기록
  python3 lateral_dual.py align                                  # 양축 12시 정렬
  python3 lateral_dual.py rotate --turns +1                      # 동기 1회전
"""

import argparse
import json
import os
import socket
import struct
import sys
import time

FMT = "IB3x8s"
IFACE_DEFAULT = "can2"
CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lateral_home.json")

M1, M2 = 0x143, 0x144
DEG_PER_MM = 7.2          # 0x143 기준: 360° = 리드스크류 1바퀴 = 50mm
ENC_CPR = 262144          # 멀티턴 엔코더 1회전 counts (18bit) — 0x60/0x61 실측 확인
DEFAULT_SPEED = 100       # dps (기존 코드의 200 보다 보수적)
ABORT_CURRENT_A = 2.5
TOLERANCE_DEG = 1.0

# 부호 규약 (2026-09-08 실측 확정, 육안 확인):
#   두 모터에 동일하게 +20 dps 를 동시에 보내면 엔코더는 둘 다 +40° 로 증가하지만,
#   모터가 서로 마주보게 설치되어 있어 기구는 **서로 반대 방향**으로 움직인다.
#   따라서 기구를 같은 방향으로 보내려면 0x144 에 반대 부호를 보내야 한다.
#   → 0x143 = +1, 0x144 = -1.  이 부호 기준으로 +방향 = 좌측 횡이동.
#   주의: 두 축은 기계적으로 연결돼 있지 않아 반대로 움직여도 전류가 튀지 않는다.
#         전류만 보고 방향을 판단하지 말 것 — 반드시 육안 확인.
DEFAULTS = {
    "0x143": {"home_angle": 33.50, "sign": +1, "name": "횡이동 #1"},
    "0x144": {"home_angle": None,  "sign": -1, "name": "횡이동 #2"},
}


def load_cfg():
    cfg = {k: dict(v) for k, v in DEFAULTS.items()}
    if os.path.exists(CFG):
        try:
            saved = json.load(open(CFG))
            for k, v in saved.items():
                if k in cfg:
                    cfg[k].update(v)
        except Exception as e:
            print(f"  ⚠️ 설정 읽기 실패 ({e}) — 기본값 사용")
    return cfg


def save_cfg(cfg):
    json.dump(cfg, open(CFG, "w"), indent=2, ensure_ascii=False)
    print(f"  저장: {CFG}")


class Bus:
    def __init__(self, iface):
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((iface,))
        self.s.settimeout(0.25)

    def drain(self):
        self.s.settimeout(0.02)
        try:
            while True:
                self.s.recv(16)
        except OSError:
            pass
        self.s.settimeout(0.25)

    def send(self, cid, data8):
        self.s.send(struct.pack(FMT, cid, 8, bytes(data8)))

    def ask(self, cid, cmd, idx=0, window=0.25):
        """cmd 를 보내고 같은 cmd 의 응답 프레임을 돌려준다"""
        self.drain()
        self.send(cid, [cmd, idx, 0, 0, 0, 0, 0, 0])
        t0 = time.time()
        while time.time() - t0 < window:
            try:
                raw = self.s.recv(16)
            except OSError:
                break
            rid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            d = raw[8:16]
            if rid == cid + 0x100 and d[0] == cmd:
                return d
        return None

    # ---- 읽기 -------------------------------------------------------------
    def angle92(self, cid):
        d = self.ask(cid, 0x92)
        return None if d is None else int.from_bytes(d[4:8], "little", signed=True) / 100.0

    def angle94(self, cid):
        d = self.ask(cid, 0x94)
        return None if d is None else int.from_bytes(d[4:8], "little", signed=True) / 100.0

    def enc61(self, cid):
        """0x61 멀티턴 엔코더 원본. mod ENC_CPR 한 값이 단회전 절대 위치(전원 무관)."""
        d = self.ask(cid, 0x61)
        return None if d is None else int.from_bytes(d[4:8], "little", signed=True)

    def status(self, cid):
        d = self.ask(cid, 0x9A)
        if d is None:
            return None
        return {
            "temp": d[1],
            "volt": int.from_bytes(d[4:6], "little") / 10.0,
            "err": int.from_bytes(d[6:8], "little"),
        }

    def current(self, cid):
        d = self.ask(cid, 0x9C)
        return None if d is None else int.from_bytes(d[2:4], "little", signed=True) / 100.0

    # ---- 쓰기 -------------------------------------------------------------
    def goto(self, cid, angle_deg, speed_dps):
        """0xA4 절대 위치: [A4][00][maxSpeed u16][angle i32 0.01deg/LSB]"""
        data = bytearray(8)
        data[0] = 0xA4
        data[2:4] = int(speed_dps).to_bytes(2, "little")
        data[4:8] = int(round(angle_deg * 100)).to_bytes(4, "little", signed=True)
        self.send(cid, data)

    def speed(self, cid, dps):
        data = bytearray(8)
        data[0] = 0xA2
        data[4:8] = int(round(dps * 100)).to_bytes(4, "little", signed=True)
        self.send(cid, data)

    def stop(self, cid):
        self.speed(cid, 0)
        time.sleep(0.05)
        self.send(cid, [0x81, 0, 0, 0, 0, 0, 0, 0])


def nearest_home(current_angle, home_angle):
    """현재 각도에서 360° 이내 가장 가까운 12시 위치"""
    remainder = current_angle % 360.0
    delta = home_angle - remainder
    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0
    return current_angle + delta, delta


def wait_arrival(bus, targets, timeout=25.0):
    """targets = {cid: target_angle}. 도달 또는 타임아웃까지 감시 (전류 보호 포함)"""
    t0 = time.time()
    last = {}
    while time.time() - t0 < timeout:
        done = True
        for cid, tgt in targets.items():
            a = bus.angle92(cid)
            if a is None:
                done = False
                continue
            last[cid] = a
            cur = bus.current(cid)
            if cur is not None and abs(cur) > ABORT_CURRENT_A:
                print(f"  ⚠️ 0x{cid:03X} 전류 {cur:+.2f}A 초과 — 전체 중단")
                for c in targets:
                    bus.stop(c)
                return False, last
            if abs(a - tgt) > TOLERANCE_DEG:
                done = False
        if done:
            return True, last
        time.sleep(0.1)
    print(f"  ⚠️ 타임아웃 {timeout}s")
    return False, last


# =============================================================================
def cmd_status(bus, cfg, args):
    print("=" * 68)
    print("횡이동 2축 상태")
    print("=" * 68)
    for cid in (M1, M2):
        c = cfg[f"0x{cid:03x}"]
        st = bus.status(cid)
        a92 = bus.angle92(cid)
        a94 = bus.angle94(cid)
        print(f"\n0x{cid:03X} {c['name']}  (부호 {c['sign']:+d})")
        if st is None:
            print("  ✗ 응답 없음")
            continue
        print(f"  온도 {st['temp']}°C  전압 {st['volt']:.1f}V  "
              f"에러 0x{st['err']:04X}{' (정상)' if st['err'] == 0 else ' ← 에러!'}")
        print(f"  0x92 멀티턴 {a92:+.2f}°   0x94 단일턴 {a94:+.2f}°")
        e61 = bus.enc61(cid)
        if e61 is not None:
            cur_single = e61 % ENC_CPR
            print(f"  0x61 단회전 절대 {cur_single} ({cur_single/ENC_CPR*360:.2f}°)")
        hs = c.get("home_enc_single")
        if hs is not None and e61 is not None:
            d = (e61 % ENC_CPR) - hs
            if d > ENC_CPR / 2:
                d -= ENC_CPR
            elif d < -ENC_CPR / 2:
                d += ENC_CPR
            dd = d / ENC_CPR * 360.0
            print(f"  12시(절대) {hs} ({hs/ENC_CPR*360:.2f}°) — 현재와 {dd:+.2f}° ({dd/DEG_PER_MM:+.2f} mm)")
        elif c["home_angle"] is None:
            print("  12시: ⚠️ 미설정 — set-home 필요")
        if c["home_angle"] is not None:
            nh, delta = nearest_home(a92, c["home_angle"])
            print(f"  12시(0x94 기준, 참고) {c['home_angle']:.2f}° → {nh:+.2f}° ({delta:+.2f}°)")
    print("\n" + "=" * 68)
    return 0


def cmd_jog(bus, cfg, args):
    cid = args.motor
    a0 = bus.angle92(cid)
    print(f"0x{cid:03X} jog {args.dps:+.1f} dps × {args.sec}s   시작 {a0:+.2f}°")
    peak = 0.0
    bus.speed(cid, args.dps)
    t0 = time.time()
    try:
        while time.time() - t0 < args.sec:
            c = bus.current(cid)
            if c is not None:
                peak = max(peak, abs(c))
                if abs(c) > ABORT_CURRENT_A:
                    print(f"  ⚠️ 전류 {c:+.2f}A — 중단")
                    break
            time.sleep(0.03)
    finally:
        bus.stop(cid)
    time.sleep(0.4)
    a1 = bus.angle92(cid)
    d = a1 - a0
    print(f"  종료 {a1:+.2f}°   Δ={d:+.2f}° ({d/DEG_PER_MM:+.3f} mm)   최대전류 {peak:.2f}A")
    print(f"  0x94 = {bus.angle94(cid):+.2f}°")
    return 0


def cmd_set_home(bus, cfg, args):
    cid = args.motor
    a94 = bus.angle94(cid)
    e61 = bus.enc61(cid)
    if a94 is None or e61 is None:
        print("✗ 각도/엔코더 읽기 실패")
        return 1
    key = f"0x{cid:03x}"
    single = e61 % ENC_CPR
    old_a = cfg[key].get("home_angle")
    old_e = cfg[key].get("home_enc_single")
    cfg[key]["home_angle"] = round(a94, 2)           # 전원 인가 기준 (참고용)
    cfg[key]["home_enc_single"] = single             # 절대 단회전 (전원 무관, 이쪽이 기준)
    print(f"0x{cid:03X} 12시 기록")
    print(f"  home_angle      : {old_a} → {a94:.2f}°        (0x94, 전원 인가 기준 — 참고용)")
    print(f"  home_enc_single : {old_e} → {single}  ({single/ENC_CPR*360:.2f}°)  "
          f"(0x61 mod {ENC_CPR}, 전원 무관 — 기준값)")
    save_cfg(cfg)
    return 0


def cmd_align(bus, cfg, args):
    targets = {}
    print("=== 12시 정렬 ===")
    for cid in (M1, M2):
        c = cfg[f"0x{cid:03x}"]
        if c["home_angle"] is None:
            print(f"✗ 0x{cid:03X} 12시 각도 미설정 — set-home 먼저 실행")
            return 1
        a = bus.angle92(cid)
        nh, delta = nearest_home(a, c["home_angle"])
        print(f"  0x{cid:03X} {c['name']}: {a:+.2f}° → {nh:+.2f}° ({delta:+.2f}°, {delta/DEG_PER_MM:+.2f} mm)")
        targets[cid] = nh
    for cid, t in targets.items():
        bus.goto(cid, t, args.speed)
    ok, last = wait_arrival(bus, targets)
    for cid in targets:
        print(f"  0x{cid:03X} 도달 {last.get(cid, float('nan')):+.2f}° (목표 {targets[cid]:+.2f}°)")
    print("  ✅ 정렬 완료" if ok else "  ⚠️ 정렬 미완료")
    return 0 if ok else 1


def cmd_rotate(bus, cfg, args):
    turns = args.turns
    side = "좌측" if turns > 0 else "우측"
    print(f"=== 동기 {turns:+.0f}회전 ({side} 횡이동, 1회전 = 50mm) ===")
    targets = {}
    for cid in (M1, M2):
        c = cfg[f"0x{cid:03x}"]
        if c["home_angle"] is None:
            print(f"✗ 0x{cid:03X} 12시 각도 미설정 — set-home 먼저 실행")
            return 1
        a = bus.angle92(cid)
        nh, _ = nearest_home(a, c["home_angle"])
        tgt = nh + c["sign"] * turns * 360.0
        print(f"  0x{cid:03X} {c['name']} (부호 {c['sign']:+d}): "
              f"{a:+.2f}° → 12시 {nh:+.2f}° → 목표 {tgt:+.2f}°")
        targets[cid] = tgt
    for cid, t in targets.items():
        bus.goto(cid, t, args.speed)
    ok, last = wait_arrival(bus, targets, timeout=args.timeout)
    print()
    for cid in targets:
        a = last.get(cid)
        if a is not None:
            print(f"  0x{cid:03X} 도달 {a:+.2f}° (목표 {targets[cid]:+.2f}°, 오차 {a-targets[cid]:+.2f}°)")
    print("  ✅ 회전 완료" if ok else "  ⚠️ 회전 미완료")
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(description="횡이동 2축 동기 제어")
    p.add_argument("--interface", default=IFACE_DEFAULT)
    p.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="최대 속도 dps")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="양축 상태 및 12시까지 거리")

    j = sub.add_parser("jog", help="속도모드 미세 이동 (12시 찾기용)")
    j.add_argument("--motor", type=lambda x: int(x, 0), required=True)
    j.add_argument("--dps", type=float, default=10.0)
    j.add_argument("--sec", type=float, default=0.5)

    h = sub.add_parser("set-home", help="현재 위치를 그 모터의 12시로 기록")
    h.add_argument("--motor", type=lambda x: int(x, 0), required=True)

    sub.add_parser("align", help="양축 12시 정렬")

    r = sub.add_parser("rotate", help="12시 정렬 후 동기 ±N회전")
    r.add_argument("--turns", type=float, required=True)
    r.add_argument("--timeout", type=float, default=30.0)

    args = p.parse_args()
    cfg = load_cfg()
    bus = Bus(args.interface)
    fn = {"status": cmd_status, "jog": cmd_jog, "set-home": cmd_set_home,
          "align": cmd_align, "rotate": cmd_rotate}[args.cmd]
    try:
        return fn(bus, cfg, args)
    finally:
        for cid in (M1, M2):
            try:
                bus.stop(cid)
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
