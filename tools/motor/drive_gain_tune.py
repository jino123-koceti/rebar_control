#!/usr/bin/env python3
"""
주행 속도루프 게인 튜닝 — 동일 동작 반복 시험 + 지표 자동 산출

배경 (2026-09-08 실측):
  * 선회 시 27 Hz 전류 진동. 배음 거의 없고 속도와 무관한 고정 주파수,
    좌우 동일 → 스틱-슬립이 아니라 **속도루프 헌팅**.
  * 정지 명령 후 실측 감속률 133~145 dps/s. 0x43 설정값 5000 dps/s 의 1/37.
    속도루프가 제동 토크를 못 내고 있다 (KI 0.0001 이 KP 0.01 의 1/100).

PID 는 0x30(읽기)/0x31(RAM 쓰기)/0x32(ROM 쓰기), DATA[1]=인덱스, DATA[4:8]=float32 LE.
  idx 1/2 전류루프 KP/KI, 4/5 속도루프 KP/KI, 7/8/9 위치루프 KP/KI/KD
튜닝 중에는 **RAM(0x31)만** 쓴다. 전원 재투입으로 공장값 복귀.

  python3 drive_gain_tune.py --read
  python3 drive_gain_tune.py --test --dps 180
  python3 drive_gain_tune.py --speed-kp 0.01 --speed-ki 0.0005 --test --dps 180
  python3 drive_gain_tune.py --restore
"""
import argparse, json, math, socket, struct, sys, time

FMT="IB3x8s"
LEFT, RIGHT = 0x142, 0x141
FWD  = {LEFT:+1, RIGHT:-1}     # 전진 (2026-09-08 검증)
BACK = {LEFT:-1, RIGHT:+1}     # 후진
SPIN_A = {LEFT:+1, RIGHT:+1}   # 제자리 선회 (좌 전진 + 우 후진)
SPIN_B = {LEFT:-1, RIGHT:-1}   # 역방향 선회

# 시험 순서: 직진 → 선회 → 역선회 → 후진.
# 순 이동·순 회전이 모두 0 이라 매 시험이 제자리에서 끝난다.
# 좁은 공간에서 게인을 바꿔가며 반복 비교하기 위한 구성.
SEQUENCE = [("직진", FWD), ("선회", SPIN_A), ("역선회", SPIN_B), ("후진", BACK)]
NAME = {LEFT:"좌측0x142", RIGHT:"우측0x141"}
IDX = {"cur_kp":1,"cur_ki":2,"spd_kp":4,"spd_ki":5,"pos_kp":7,"pos_ki":8,"pos_kd":9}
FACTORY = {"cur_kp":1.5,"cur_ki":0.05,"spd_kp":0.01,"spd_ki":0.0001,
           "pos_kp":0.05,"pos_ki":0.0,"pos_kd":0.5}
I_HARD = 18.0

class Bus:
    def __init__(self, iface="can2"):
        self.s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        self.s.bind((iface,))
    def drain(self):
        self.s.setblocking(False)
        try:
            while True: self.s.recv(16)
        except OSError: pass
        finally: self.s.setblocking(True)
    def send(self,cid,d): self.s.send(struct.pack(FMT,cid,8,bytes(d)))
    def ask(self,cid,d8,expect,window=0.3):
        self.drain(); self.s.settimeout(window); self.send(cid,d8)
        t0=time.time()
        while time.time()-t0<window:
            try: raw=self.s.recv(16)
            except OSError: break
            rid=struct.unpack("I",raw[:4])[0]&0x1FFFFFFF; r=raw[8:16]
            if rid==cid+0x100 and r[0]==expect: return r
        return None
    def st2(self,cid):
        r=self.ask(cid,[0x9C,0,0,0,0,0,0,0],0x9C,0.06)
        if r is None: return None
        return (int.from_bytes(r[2:4],"little",signed=True)/100.0,
                int.from_bytes(r[4:6],"little",signed=True), r[1])
    def speed(self,cid,dps,max_torque=100):
        """0xA2 속도 제어.

        DATA[1] = maxTorque: **정격 전류의 백분율** (uint8, 1 LSB = 1%).
        0 으로 두면 힘 제어가 활성화되지 않아 가감속이 전류 출력 능력에
        묶인다 (프로토콜 2.20.1 주석 3). 이 바이트를 0 으로 보내던 것이
        "감속이 관성처럼 느리다"의 실제 원인이었다 (2026-09-08).
        """
        b=bytearray(8); b[0]=0xA2; b[1]=max(0,min(255,int(max_torque)))
        b[4:8]=int(round(dps*100)).to_bytes(4,"little",signed=True)
        self.send(cid,b)
    def stop(self,cid,mt=100): self.speed(cid,0,mt); self.send(cid,[0x81,0,0,0,0,0,0,0])
    def shutdown(self,cid): self.send(cid,[0x80,0,0,0,0,0,0,0])
    # ---- 가감속 (0x42 읽기 / 0x43 RAM+ROM 쓰기) ----
    # 인덱스 (프로토콜 2.4.4 / 2.5.4):
    #   0 위치계획 가속, 1 위치계획 감속, 2 속도계획 가속, 3 속도계획 감속
    # 주의: 0x43 은 RAM+ROM 동시 저장이라 전원을 꺼도 남는다.
    def accel_read(self,cid,idx):
        r=self.ask(cid,[0x42,idx,0,0,0,0,0,0],0x42)
        return None if r is None else int.from_bytes(r[4:8],"little",signed=True)
    def accel_write(self,cid,idx,val):
        b=bytearray(8); b[0]=0x43; b[1]=idx
        b[4:8]=int(val).to_bytes(4,"little",signed=True)
        r=self.ask(cid,list(b),0x43)
        return None if r is None else int.from_bytes(r[4:8],"little",signed=True)

    # ---- PID ----
    def pid_read(self,cid,idx):
        r=self.ask(cid,[0x30,idx,0,0,0,0,0,0],0x30)
        return None if r is None else struct.unpack('<f',r[4:8])[0]
    def pid_write_ram(self,cid,idx,val):
        b=bytearray(8); b[0]=0x31; b[1]=idx; b[4:8]=struct.pack('<f',float(val))
        r=self.ask(cid,list(b),0x31)
        return None if r is None else struct.unpack('<f',r[4:8])[0]

def read_all(bus):
    print(f"{'파라미터':<12}" + "".join(f"{NAME[c]:>14}" for c in (LEFT,RIGHT)))
    for k,i in IDX.items():
        vals=[bus.pid_read(c,i) for c in (LEFT,RIGHT)]
        print(f"{k:<12}" + "".join(f"{('—' if v is None else f'{v:g}'):>14}" for v in vals))

def ensure_stopped(bus):
    for c in (LEFT,RIGHT): bus.stop(c)
    time.sleep(0.6)
    for c in (LEFT,RIGHT):
        st=bus.st2(c)
        if st and abs(st[1])>5:
            print(f"  ✗ {NAME[c]} 아직 회전 중 ({st[1]} dps) — 중단"); return False
    return True

def run_move(bus, signs, dps, hold, label, max_torque=100):
    """지정 부호로 hold 초 구동 후 정지. 전 구간 샘플 반환."""
    samples={c:[] for c in (LEFT,RIGHT)}
    for c in (LEFT,RIGHT): bus.speed(c, signs[c]*dps, max_torque)
    t0=time.time(); cmd_zero_t=None
    while True:
        now=time.time()-t0
        if cmd_zero_t is None and now>=hold:
            for c in (LEFT,RIGHT): bus.speed(c,0,max_torque)
            cmd_zero_t=now
        for c in (LEFT,RIGHT):
            st=bus.st2(c)
            if st is None: continue
            samples[c].append({"t":round(time.time()-t0,4),"cur":st[0],"spd":st[1],"temp":st[2]})
            if abs(st[0])>I_HARD:
                print(f"  ⛔ {NAME[c]} 과전류 {st[0]:.1f}A — 즉시 차단")
                for x in (LEFT,RIGHT): bus.shutdown(x)
                return samples, cmd_zero_t, True
        if cmd_zero_t is not None:
            stopped=all(samples[c] and abs(samples[c][-1]["spd"])<5 for c in (LEFT,RIGHT))
            if stopped or now-cmd_zero_t>4.0: break
        if now>hold+6: break
    for c in (LEFT,RIGHT): bus.stop(c)
    return samples, cmd_zero_t, False

def metrics(samples, cmd_zero_t, dps, signs):
    out={}
    for c,rows in samples.items():
        if len(rows)<30: continue
        drive=[r for r in rows if r["t"]<cmd_zero_t-0.3 and abs(r["spd"])>5]
        dec=[r for r in rows if r["t"]>=cmd_zero_t]
        m={}
        if drive:
            sp=[abs(r["spd"]) for r in drive]; cu=[abs(r["cur"]) for r in drive]
            m["spd_mean"]=sum(sp)/len(sp); m["spd_err"]=dps-m["spd_mean"]
            m["cur_mean"]=sum(cu)/len(cu); m["cur_max"]=max(cu)
            # 27Hz 대역 진폭
            t=[r["t"] for r in drive]; y=[r["cur"] for r in drive]
            if len(y)>=64 and t[-1]-t[0]>0.5:
                import numpy as np
                fs=len(t)/(t[-1]-t[0])
                tu=np.arange(t[0],t[-1],1/fs); yu=np.interp(tu,t,y); yu=yu-yu.mean()
                w=np.hanning(len(yu)); Y=np.abs(np.fft.rfft(yu*w))*2/np.sum(w)
                f=np.fft.rfftfreq(len(yu),1/fs)
                band=(f>20)&(f<35)
                m["osc27"]=float(Y[band].max()) if band.any() else 0.0
                m["osc_f"]=float(f[band][Y[band].argmax()]) if band.any() else 0.0
        if dec:
            v0=abs(dec[0]["spd"])
            stop=next((r for r in dec if abs(r["spd"])<5), None)
            if stop and v0>50:
                dt=stop["t"]-dec[0]["t"]
                m["stop_s"]=dt; m["decel"]=v0/dt if dt>0 else 0
                m["coast_deg"]=v0*dt/2
        out[c]=m
    return out

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--interface",default="can2")
    ap.add_argument("--read",action="store_true")
    ap.add_argument("--restore",action="store_true")
    ap.add_argument("--read-accel",action="store_true")
    ap.add_argument("--set-accel",type=int,help="속도계획 가속/감속(idx 2,3) 값 dps/s")
    ap.add_argument("--speed-kp",type=float)
    ap.add_argument("--speed-ki",type=float)
    ap.add_argument("--test",action="store_true")
    ap.add_argument("--dps",type=float,default=180.0)
    ap.add_argument("--hold",type=float,default=3.0)
    ap.add_argument("--max-torque",type=int,default=100,
                    help="0xA2 DATA[1] 최대토크 (정격 전류의 %%, 0=제한없음이 아니라 힘제어 비활성)")
    ap.add_argument("--out")
    ap.add_argument("--repeat",type=int,default=1,help="시퀀스 반복 횟수 (평균/편차 산출)")
    ap.add_argument("--phases",default="all",
                    help="수행할 구간: all | spin (선회+역선회만) | straight (직진+후진만)")
    a=ap.parse_args()
    bus=Bus(a.interface)

    if a.read:
        print("=== 현재 PID (0x30) ===\n"); read_all(bus); return 0

    if a.read_accel:
        names={0:"위치계획 가속",1:"위치계획 감속",2:"속도계획 가속",3:"속도계획 감속"}
        print("=== 가감속 (0x42) ===")
        for i,n in names.items():
            v=[bus.accel_read(c,i) for c in (LEFT,RIGHT)]
            print(f"  idx{i} {n:<12} 좌측 {v[0]}  우측 {v[1]}  dps/s")
        return 0

    if a.set_accel is not None:
        print(f"=== 속도계획 가속/감속 ← {a.set_accel} dps/s (0x43, RAM+ROM) ===")
        if not ensure_stopped(bus): return 1
        for i in (2,3):
            for c in (LEFT,RIGHT):
                r=bus.accel_write(c,i,a.set_accel)
                print(f"  {NAME[c]} idx{i} ← {a.set_accel}  응답 {r}")
        time.sleep(0.3)
        print("  확인:")
        for i in (2,3):
            v=[bus.accel_read(c,i) for c in (LEFT,RIGHT)]
            print(f"    idx{i}: 좌측 {v[0]}  우측 {v[1]}")

    if a.restore:
        print("=== 공장값으로 복원 (RAM) ===")
        if not ensure_stopped(bus): return 1
        for k,v in FACTORY.items():
            for c in (LEFT,RIGHT):
                r=bus.pid_write_ram(c,IDX[k],v)
                print(f"  {NAME[c]} {k} ← {v:g}  (응답 {r})")
        return 0

    if a.speed_kp is not None or a.speed_ki is not None:
        print("=== 속도루프 게인 쓰기 (RAM, 전원 재투입 시 원복) ===")
        if not ensure_stopped(bus): return 1
        for k,v in (("spd_kp",a.speed_kp),("spd_ki",a.speed_ki)):
            if v is None: continue
            for c in (LEFT,RIGHT):
                r=bus.pid_write_ram(c,IDX[k],v)
                ok = r is not None and abs(r-v)<1e-6
                print(f"  {NAME[c]} {k} ← {v:g}   {'✓' if ok else f'✗ 응답 {r}'}")
        time.sleep(0.3)

    if not a.test: return 0

    seq = SEQUENCE
    if a.phases=="spin":     seq=[x for x in SEQUENCE if "선회" in x[0]]
    elif a.phases=="straight":seq=[x for x in SEQUENCE if "선회" not in x[0]]
    seq_txt=" → ".join(f"{n} {a.hold:.0f}s" for n,_ in seq)
    print(f"\n=== 반복 시험 @ {a.dps:.0f} dps, 최대토크 {a.max_torque}% ===")
    print(f"    {seq_txt} → 정지  (순 이동/회전 0 → 제자리 복귀)")
    if not ensure_stopped(bus): return 1
    results={}; acc={}
    for rep in range(a.repeat):
        if a.repeat>1: print(f"\n########## 반복 {rep+1}/{a.repeat} ##########")
        for label,signs in seq:
            print(f"\n--- {label} ---")
            smp,cz,abort=run_move(bus,signs,a.dps,a.hold,label,a.max_torque)
            if abort: return 1
            mt=metrics(smp,cz,a.dps,signs)
            results.setdefault(label,[]).append(
                {"metrics":{hex(k):v for k,v in mt.items()},
                 "samples":{hex(k):v for k,v in smp.items()}})
            for c in (LEFT,RIGHT):
                m=mt.get(c,{})
                if not m: continue
                acc.setdefault((label,c),[]).append(m)
                print(f"  {NAME[c]}: 진동 {m.get('osc27',0):.3f}A @ {m.get('osc_f',0):.1f}Hz | "
                      f"전류 평균 {m.get('cur_mean',0):.2f}A 최대 {m.get('cur_max',0):.2f}A | "
                      f"감속 {m.get('decel',0):.0f} dps/s, 미끄러짐 {m.get('coast_deg',0):.0f}°")
            time.sleep(1.5)

    import statistics as stx
    print(f"\n=== 요약 ({a.repeat}회 평균) ===")
    print(f"  {'구간':<8}{'축':<12}{'진동A':>16}{'전류평균':>10}{'감속dps/s':>12}{'미끄러짐':>11}")
    for label,_ in seq:
        for c in (LEFT,RIGHT):
            ms=acc.get((label,c))
            if not ms: continue
            def agg(k):
                v=[m.get(k,0) for m in ms]
                return stx.mean(v), (stx.stdev(v) if len(v)>1 else 0.0)
            o,osd=agg('osc27'); cu,_=agg('cur_mean'); de,_=agg('decel'); co,_=agg('coast_deg')
            print(f"  {label:<8}{NAME[c]:<12}{o:>8.3f}±{osd:<6.3f}{cu:>9.2f}"
                  f"{de:>12.0f}{co:>10.0f}°")
    if a.out:
        json.dump(results,open(a.out,"w")); print(f"\n샘플 저장: {a.out}")
    return 0

if __name__=="__main__": sys.exit(main())
