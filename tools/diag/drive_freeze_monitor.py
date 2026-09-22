#!/usr/bin/env python3
"""주행 프리즈 진단 모니터 — 0.5s마다 시스템 상태를 fsync로 디스크 기록.

프리즈→폭주→강제전원차단(cold) 시나리오에서 netconsole/serial 불가할 때,
**프리즈 직전 시스템 상태 추이**를 NVMe에 남겨(전원 내려도 보존) 원인을 좁힌다.
기록: load, 메모리, CAN2(주행모터버스) tx/rx/err, CPU 상위 프로세스(스핀 감지).

매 줄 fsync → 전원이 끊겨도 마지막 줄까지 디스크에 남음 (volt_monitor 방식).

  python3 tools/diag/drive_freeze_monitor.py [--out /var/log/robot_control/drive_freeze.csv] [--hz 2]
프리즈 후 재부팅:  tail -30 <out>   ← 마지막 줄들이 프리즈 직전 상태
"""
import os
import time
import argparse


def read_procstat_cpu():
    """{pid: (comm, utime+stime)} — 프로세스별 누적 CPU 틱."""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                parts = f.read().split()
            # comm은 괄호 안 (공백 포함 가능) → 뒤에서 인덱싱
            rp = parts[-1::-1]
            # utime=14, stime=15 (1-indexed) → 0-indexed 13,14. 하지만 comm 공백 대비 rsplit 사용
            comm = parts[1].strip('()')
            utime = int(parts[13]); stime = int(parts[14])
            out[pid] = (comm, utime + stime)
        except Exception:
            continue
    return out


def can_stats(dev='can2'):
    base = f'/sys/class/net/{dev}/statistics'
    def r(n):
        try:
            return int(open(f'{base}/{n}').read())
        except Exception:
            return -1
    return r('tx_packets'), r('rx_packets'), r('tx_errors'), r('rx_errors'), r('tx_dropped')


def mem_free_mb():
    try:
        for line in open('/proc/meminfo'):
            if line.startswith('MemAvailable:'):
                return int(line.split()[1]) // 1024
    except Exception:
        pass
    return -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/var/log/robot_control/drive_freeze.csv')
    ap.add_argument('--hz', type=float, default=2.0)
    ap.add_argument('--can', default='can2')
    ap.add_argument('--topn', type=int, default=4)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    f = open(args.out, 'a')
    f.write('# t, load1, memMB, can_tx, can_rx, can_txerr, can_rxerr, can_drop, top_cpu(comm:pct ...)\n')
    f.flush(); os.fsync(f.fileno())

    prev = read_procstat_cpu()
    prev_t = time.monotonic()
    clk = os.sysconf('SC_CLK_TCK')
    ncpu = os.cpu_count() or 1
    period = 1.0 / args.hz
    print(f'주행 프리즈 모니터 시작 → {args.out} ({args.hz}Hz, fsync). Ctrl+C 종료.')

    while True:
        time.sleep(period)
        now = time.monotonic()
        dt = now - prev_t
        cur = read_procstat_cpu()
        # 프로세스별 CPU% (delta ticks / dt / clk * 100), 상위 N
        deltas = []
        for pid, (comm, tot) in cur.items():
            if pid in prev:
                d = tot - prev[pid][1]
                if d > 0:
                    pct = 100.0 * d / (dt * clk)
                    deltas.append((pct, comm))
        deltas.sort(reverse=True)
        top = ' '.join(f'{c}:{p:.0f}' for p, c in deltas[:args.topn])
        prev, prev_t = cur, now

        la = open('/proc/loadavg').read().split()[0]
        txp, rxp, txe, rxe, drp = can_stats(args.can)
        line = (f'{time.strftime("%H:%M:%S")},{la},{mem_free_mb()},'
                f'{txp},{rxp},{txe},{rxe},{drp},{top}\n')
        f.write(line)
        f.flush(); os.fsync(f.fileno())     # ← 전원 끊겨도 이 줄까지 디스크에 남음


if __name__ == '__main__':
    main()
