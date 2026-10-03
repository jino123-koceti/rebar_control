#!/usr/bin/env python3
"""yaw_home 감지 구간의 **양쪽 에지**를 단회전값으로 잰다 (모터 구동).

## 왜 양쪽인가

감지판에 폭이 있어서 **접근 방향에 따라 켜지는 지점이 다르다.**

    건 각도 증가(1번→2번 방향)로 접근 → 판의 **아래** 경계에서 켜짐
    건 각도 감소(4번→3번 방향)로 접근 → 판의 **위** 경계에서 켜짐

그래서 "에지 → 12시" 오프셋이 방향마다 다르다. 이것을 빠뜨리면 호밍이 12시를
**판 폭만큼(건 2.07°) 비껴간다** — 2026-10-03 실장비 실행에서 1번 자세(증가 방향)로
접근했는데 위 경계 기준 오프셋(+57.70°)을 써서 그만큼 틀어졌다.

## 방법

감지 구간 안이나 근처에서 시작해 **위로 나갔다 → 내려오고 → 아래로 나갔다 → 올라온다.**
네 전이(켜짐·꺼짐 × 두 방향)를 모두 잡는다. 필요한 ON 지점은 두 개다:
  · 감소 방향 ON = 위 경계 → `edge_single_from_high`
  · 증가 방향 ON = 아래 경계 → `edge_single_from_low`

## 왜 모터로 하는가 (손이 아니라)

손으로 돌리면 전류는 안 쓰지만, 센서 콜백이 뜰 때 **그때 폴링된 마지막 위치**를
적게 되어 사람이 빠르게 돌리면 지연이 각도로 번진다(실측에서 건 10° 어긋났다).
모터는 속도가 일정해서 지연이 일정하고, 양방향 값을 비교하면 그 지연이 상쇄된다.

## 정지마찰

정지 상태에서 출발하면 **전류가 5A 넘게 올라가며 2초 정도 거의 안 움직인다**
(1번 자세 근처 실측: 0.22 → 5.41A, 2.4초 후 풀림, 그 뒤 1.2A). 그래서 전류 상한을
돌파 요구치보다 **위로** 두어야 한다 — 5~6A 로 잡으면 풀리기 직전에 끊긴다.

## 안전장치

전류·온도·이동량·시간 중 하나라도 넘으면 즉시 정지 → 0x78 → 0x80.
yaw 는 끝 리미트가 없다 — 이동량 상한이 유일한 보호다.
"""

import argparse
import socket
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

IFACE, YAW = 'can2', 0x148
CPR, GEAR = 262144, 12.5
CPD = CPR / 360.0
CPG = CPD * GEAR


def wrap(x):
    return (x + CPR // 2) % CPR - CPR // 2


class EdgeBoth(Node):
    def __init__(self, dps, i_stop, t_stop):
        super().__init__('yaw_edge_both')
        self.dps, self.i_stop, self.t_stop = dps, i_stop, t_stop
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((IFACE,))
        self.home = None
        self.create_subscription(Bool, '/limit_sensors/yaw_home', self._on_home, 10)
        self.hits = []            # (방향, 단회전, 켜짐여부)

    def _on_home(self, m):
        self.home = bool(m.data)

    # ---- CAN -----------------------------------------------------------
    def send(self, d8):
        self.s.send(struct.pack("IB3x8s", YAW, 8, d8))

    def flush(self, t=0.015):
        end = time.time() + t
        while time.time() < end:
            self.s.settimeout(max(0.003, end - time.time()))
            try:
                self.s.recv(16)
            except OSError:
                return

    def ask(self, cmd, tries=3):
        for _ in range(tries):
            self.flush()
            self.send(bytes([cmd, 0, 0, 0, 0, 0, 0, 0]))
            end = time.time() + 0.06
            while time.time() < end:
                self.s.settimeout(max(0.003, end - time.time()))
                try:
                    raw = self.s.recv(16)
                except OSError:
                    break
                cid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
                d = raw[8:16]
                if cid == YAW + 0x100 and d[0] == cmd:
                    return d
        return None

    def single(self):
        d = self.ask(0x61)
        return None if d is None else struct.unpack('<i', d[4:8])[0] % CPR

    def st2(self):
        d = self.ask(0x9C)
        if d is None:
            return None
        return (struct.unpack('<h', d[2:4])[0] * 0.01, d[1])

    def halt(self):
        self.send(struct.pack("<BB2xi", 0xA2, 0x64, 0))

    def park(self):
        self.halt()
        time.sleep(0.2)
        self.send(bytes([0x78, 0, 0, 0, 0, 0, 0, 0]))
        time.sleep(0.1)
        self.send(bytes([0x80, 0, 0, 0, 0, 0, 0, 0]))

    def release(self):
        for i in range(12):
            self.send(bytes([0x77, 0, 0, 0, 0, 0, 0, 0]))
            time.sleep(0.25)
            b = self.ask(0x9A)
            if b and b[3] == 1:
                print(f"  브레이크 해제 확인 ({(i + 1) * 0.25:.2f}초)")
                return True
        return False

    # ---- 한 구간 ---------------------------------------------------------
    def leg(self, sign, want_on, cap_gun, label):
        """want_on 상태가 될 때까지 sign 방향으로 간다. 이동량 상한 cap_gun(건°)."""
        s0 = self.single()
        arrow = '증가' if sign > 0 else '감소'
        print(f"\n[{label}]  {arrow} 방향, 센서 {'켜짐' if want_on else '꺼짐'} 까지 "
              f"(상한 건 {cap_gun:.1f}°)")
        self.send(struct.pack("<BB2xi", 0xA2, 0x64, int(sign * self.dps * 100)))
        t0, prev, peak = time.time(), self.home, 0.0
        why = '시간 상한'
        while time.time() - t0 < 40:
            rclpy.spin_once(self, timeout_sec=0.0)
            cur = self.single()
            if cur is None:
                continue
            moved = wrap(cur - s0) / CPG
            if self.home is not None and self.home != prev:
                self.hits.append((arrow, cur, self.home))
                print(f"  ★ {'켜짐' if self.home else '꺼짐'} — 단회전 {cur} "
                      f"(이동 건 {moved:+.2f}°)")
                prev = self.home
                if self.home == want_on:
                    why = '목표 전이'
                    break
            if abs(moved) > cap_gun:
                why = f"이동 상한 (건 {moved:+.2f}°)"
                break
            v = self.st2()
            if v:
                peak = max(peak, abs(v[0]))
                if abs(v[0]) > self.i_stop or v[1] >= self.t_stop:
                    why = f"전류 {v[0]:+.2f}A / 온도 {v[1]}°C"
                    break
        self.halt()
        time.sleep(0.4)
        print(f"  정지 — {why} (최대 {peak:.2f}A)")
        return why == '목표 전이'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dps', type=float, default=12.0, help='모터축 dps')
    ap.add_argument('--cap', type=float, default=4.0, help='구간당 이동 상한 (건°)')
    # 정지마찰 돌파에 5.4A 가 필요했다 — 상한을 그보다 넉넉히 둔다
    ap.add_argument('--i-stop', type=float, default=8.0)
    ap.add_argument('--t-stop', type=float, default=45.0)
    a = ap.parse_args()

    rclpy.init()
    n = EdgeBoth(a.dps, a.i_stop, a.t_stop)
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.home is not None:
            break
    print(f"센서 초기 상태: {n.home}   (감지 구간 안에서 시작하는 것이 좋다)")
    n.ask(0x9A)
    if not n.release():
        print("✗ 브레이크 해제 실패")
        n.park()
        return 1
    try:
        n.leg(+1, False, a.cap, '1/4 위로 나가기')      # 위 경계에서 꺼짐
        n.leg(-1, True, a.cap, '2/4 내려와 켜짐')        # 위 경계 ON ← from_high
        n.leg(-1, False, a.cap, '3/4 아래로 나가기')     # 아래 경계에서 꺼짐
        n.leg(+1, True, a.cap, '4/4 올라와 켜짐')        # 아래 경계 ON ← from_low
    except KeyboardInterrupt:
        pass
    finally:
        n.park()
        print("\n정지 + 0x78 + 0x80")

    print()
    print("=" * 64)
    ups = [c for d, c, on in n.hits if on and d == '증가']
    dns = [c for d, c, on in n.hits if on and d == '감소']
    print("=== axes.yaml 에 넣을 값 ===")
    if ups:
        print(f"  edge_single_from_low:  {ups[-1]}    # 건 각도 증가 접근 → 아래 경계")
    if dns:
        print(f"  edge_single_from_high: {dns[-1]}    # 건 각도 감소 접근 → 위 경계")
    if ups and dns:
        w = wrap(dns[-1] - ups[-1])
        print(f"  판 폭 {abs(w)} counts = 건 {abs(w) / CPG:.2f}° "
              f"= 모터축 {abs(w) / CPD:.2f}°")
        print(f"  → 방향별 오프셋 차이가 그만큼이다")
    else:
        print("  ⚠ 두 ON 지점을 다 못 잡았다 — --cap 을 늘려 다시 할 것")
    print("=" * 64)
    n.s.close()
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
