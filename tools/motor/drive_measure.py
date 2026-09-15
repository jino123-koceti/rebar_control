#!/usr/bin/env python3
"""
주행륜 정밀 회전 — wheel_radius 실측용

바퀴를 정확히 N회전 시키고 엔코더 실제 회전각을 보고한다.
사용자가 실제 이동거리를 재서 입력하면 wheel_radius 를 계산할 수 있다.

부호 규약 (2026-09-08 검증): 전진 시 좌측 0x142 = +, 우측 0x141 = -.
전류 보호는 횡이동과 동일한 Protection 을 사용한다.

  python3 drive_measure.py --turns 1 --dps 60
  python3 drive_measure.py --turns 1 --dps 60 --measured-mm 628   # 반지름 계산
"""
import argparse, math, socket, struct, sys, time
sys.path.insert(0, '/home/koceti/ros2_ws/install/rmd_robot_control/lib/python3.10/site-packages')
from rmd_robot_control.lateral_axes import Protection, I_HARD_A, SEV_HARD

FMT = "IB3x8s"
LEFT, RIGHT = 0x142, 0x141          # 좌측=+, 우측=-
SIGN = {LEFT: +1, RIGHT: -1}
NAME = {LEFT: "좌측 0x142", RIGHT: "우측 0x141"}


class Bus:
    def __init__(self, iface="can2"):
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((iface,)); self.s.settimeout(0.2)

    def drain(self):
        self.s.setblocking(False)
        try:
            while True: self.s.recv(16)
        except OSError: pass
        finally:
            self.s.setblocking(True); self.s.settimeout(0.2)

    def send(self, cid, d): self.s.send(struct.pack(FMT, cid, 8, bytes(d)))

    def ask(self, cid, cmd, window=0.15):
        self.drain(); self.s.settimeout(window)
        self.send(cid, [cmd,0,0,0,0,0,0,0])
        t0=time.time()
        while time.time()-t0 < window:
            try: raw=self.s.recv(16)
            except OSError: break
            rid=struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF; d=raw[8:16]
            if rid==cid+0x100 and d[0]==cmd: return d
        return None

    def angle(self, cid):
        d=self.ask(cid,0x92)
        return None if d is None else int.from_bytes(d[4:8],"little",signed=True)/100.0

    def st2(self, cid):
        d=self.ask(cid,0x9C,0.08)
        if d is None: return None
        return (int.from_bytes(d[2:4],"little",signed=True)/100.0,
                int.from_bytes(d[4:6],"little",signed=True), d[1])

    def goto(self, cid, deg, dps):
        b=bytearray(8); b[0]=0xA4
        b[2:4]=int(dps).to_bytes(2,"little")
        b[4:8]=int(round(deg*100)).to_bytes(4,"little",signed=True)
        self.send(cid,b)

    def stop(self, cid): self.send(cid,[0xA2,0,0,0,0,0,0,0]); self.send(cid,[0x81,0,0,0,0,0,0,0])
    def shutdown(self, cid): self.send(cid,[0x80,0,0,0,0,0,0,0])


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--turns", type=float, default=1.0)
    ap.add_argument("--dps", type=float, default=60.0)
    ap.add_argument("--timeout", type=float, default=25.0)
    ap.add_argument("--measured-mm", type=float, help="실측 이동거리 (mm)")
    a=ap.parse_args()

    bus=Bus(); prot={m:Protection(m,NAME[m]) for m in (LEFT,RIGHT)}
    a0={m:bus.angle(m) for m in (LEFT,RIGHT)}
    if any(v is None for v in a0.values()):
        print("✗ 엔코더 읽기 실패"); return 1

    tgt={m: a0[m] + SIGN[m]*a.turns*360.0 for m in (LEFT,RIGHT)}
    print(f"전진 {a.turns}회전 @ {a.dps:.0f} dps")
    for m in (LEFT,RIGHT):
        print(f"  {NAME[m]}: {a0[m]:+.2f}° → {tgt[m]:+.2f}° ({SIGN[m]*a.turns*360:+.0f}°)")

    for m in (LEFT,RIGHT): bus.goto(m, tgt[m], a.dps)

    t0=time.time(); n=0; tripped=None
    while time.time()-t0 < a.timeout:
        now=time.time()
        for m in (LEFT,RIGHT):
            st=bus.st2(m)
            if st is None: continue
            n+=1
            r,sev = prot[m].update(st[0], st[1], st[2], now)
            if r:
                tripped=(m,r,sev); break
        if tripped: break
        if now-t0 > 0.3:
            done=all(bus.angle(m) is not None and abs(bus.angle(m)-tgt[m])<1.0
                     for m in (LEFT,RIGHT))
            if done: break

    if tripped:
        m,r,sev = tripped
        print(f"\n⛔ 보호 발동: {NAME[m]} {r}")
        if sev==SEV_HARD:
            for x in (LEFT,RIGHT): bus.shutdown(x)
        else:
            for x in (LEFT,RIGHT): bus.stop(x)
    else:
        for m in (LEFT,RIGHT): bus.stop(m)

    time.sleep(0.5)
    print(f"\n감시 {n/max(time.time()-t0,0.01):.0f} Hz")
    deltas={}
    for m in (LEFT,RIGHT):
        a1=bus.angle(m); d=a1-a0[m]; deltas[m]=d
        print(f"  {NAME[m]}: {a0[m]:+.2f}° → {a1:+.2f}°  Δ={d:+.2f}° "
              f"(목표 {SIGN[m]*a.turns*360:+.0f}°, 오차 {d-SIGN[m]*a.turns*360:+.2f}°)")
        print(f"    {prot[m].summary()}")

    turns_actual = sum(abs(deltas[m]) for m in (LEFT,RIGHT))/2/360.0
    print(f"\n  실제 바퀴 회전수(좌우 평균): {turns_actual:.4f} 회전")
    if a.measured_mm:
        r_mm = a.measured_mm/(2*math.pi*turns_actual)
        print(f"  실측 이동 {a.measured_mm:.0f} mm → wheel_radius = {r_mm/1000:.5f} m ({r_mm:.1f} mm)")
        print(f"    현재 설정: rmd_robot_control 0.1 m / rebar_base_control 0.02865 m")
    else:
        print("  → 실제 이동거리를 재서 --measured-mm 로 다시 실행하면 반지름을 계산합니다")
    return 0

if __name__=="__main__":
    sys.exit(main())
