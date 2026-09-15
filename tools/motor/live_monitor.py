#!/usr/bin/env python3
"""
4개 모터 실시간 전류/온도 모니터 — 텔레옵 주행 중 관찰용

별도 CAN 소켓으로 0x9C 를 폴링한다. 노드와 동시에 떠 있어도 되며
읽기 전용이라 모터 동작에는 영향을 주지 않는다.

  python3 live_monitor.py --seconds 240 --out /tmp/drive_mon.json
"""
import argparse, json, math, socket, struct, sys, time

FMT="IB3x8s"
MOTORS=[(0x141,"우측주행"),(0x142,"좌측주행"),(0x143,"횡이동#1"),(0x144,"횡이동#2")]
I_RATED=6.1; I_HARD=18.0; I2T_BUDGET=600.0; COOL_TAU=20.0

class Bus:
    def __init__(self,iface="can2"):
        self.s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        self.s.bind((iface,)); self.s.settimeout(0.05)
    def drain(self):
        self.s.setblocking(False)
        try:
            while True: self.s.recv(16)
        except OSError: pass
        finally: self.s.setblocking(True); self.s.settimeout(0.05)
    def st2(self,cid):
        self.drain()
        self.s.send(struct.pack(FMT,cid,8,bytes([0x9C,0,0,0,0,0,0,0])))
        t0=time.time()
        while time.time()-t0<0.05:
            try: raw=self.s.recv(16)
            except OSError: break
            rid=struct.unpack("I",raw[:4])[0]&0x1FFFFFFF; d=raw[8:16]
            if rid==cid+0x100 and d[0]==0x9C:
                return (int.from_bytes(d[2:4],"little",signed=True)/100.0,
                        int.from_bytes(d[4:6],"little",signed=True), d[1])
        return None

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--seconds",type=float,default=240)
    ap.add_argument("--out",default="/tmp/drive_mon.json")
    ap.add_argument("--report",type=float,default=5.0)
    a=ap.parse_args()
    bus=Bus()
    st={m:{"peak":0.0,"i2t":0.0,"tmax":0,"n":0,"active_s":0.0,"last":None} for m,_ in MOTORS}
    samples={hex(m):[] for m,_ in MOTORS}
    t0=time.time(); nxt=t0+a.report
    print(f"모니터 시작 — {a.seconds:.0f}초. 정격 {I_RATED}A / 하드 {I_HARD}A / I²t {I2T_BUDGET:.0f}",flush=True)
    while time.time()-t0 < a.seconds:
        now=time.time()
        for m,nm in MOTORS:
            r=bus.st2(m)
            if r is None: continue
            cur,spd,tmp=r; i=abs(cur); s=st[m]
            dt=0.0 if s["last"] is None else max(0.0,now-s["last"]); s["last"]=now
            s["n"]+=1; s["peak"]=max(s["peak"],i); s["tmax"]=max(s["tmax"],tmp)
            if abs(spd)>2: s["active_s"]+=dt
            if i>I_RATED: s["i2t"]+=(i*i-I_RATED**2)*dt
            elif dt>0 and s["i2t"]>0: s["i2t"]*=math.exp(-dt/COOL_TAU)
            samples[hex(m)].append({"t":round(now-t0,3),"cur":cur,"spd":spd,"temp":tmp})
        if now>=nxt:
            nxt=now+a.report
            parts=[]
            for m,nm in MOTORS:
                s=st[m]
                last=samples[hex(m)][-1] if samples[hex(m)] else None
                c=abs(last["cur"]) if last else 0
                parts.append(f"{nm} {c:4.1f}A(최대{s['peak']:4.1f}) I²t{s['i2t']:5.0f} {s['tmax']}°C")
            print(f"[{now-t0:5.1f}s] " + " | ".join(parts),flush=True)
    print("\n===== 요약 =====",flush=True)
    for m,nm in MOTORS:
        s=st[m]
        print(f"  0x{m:03X} {nm}: 최대 {s['peak']:.2f}A  I²t {s['i2t']:.0f} A²·s  "
              f"최고 {s['tmax']}°C  구동시간 {s['active_s']:.1f}s  샘플 {s['n']}",flush=True)
        if s["peak"]>I_HARD: print(f"    ⚠️ 하드 리밋 {I_HARD}A 초과!",flush=True)
        elif s["peak"]>I_RATED: print(f"    정격 {I_RATED}A 초과 구간 있음 (정상일 수 있음)",flush=True)
    json.dump(samples,open(a.out,"w"))
    print(f"\n샘플 저장: {a.out}",flush=True)

if __name__=="__main__": sys.exit(main())
