#!/usr/bin/env python3
"""커널 메시지(/dev/kmsg) 실시간 fsync 로거 — 프리즈 원인 포획용.

netconsole/serial 불가 + 하드행+강제전원차단(cold) 환경에서, 커널이 프리즈 직전
찍는 Oops/WARNING/스택트레이스를 **매 메시지 fsync로 NVMe에 저장** → 전원 내려도 보존.
(journald는 버퍼링해 전원차단 전 flush 못 하면 소실됨 → 이건 메시지마다 즉시 fsync)

  sudo python3 tools/diag/kmsg_fsync_logger.py [--out /var/log/robot_control/kmsg_freeze.log]
프리즈 후 재부팅:  tail -50 <out>   ← 마지막 줄들 = 프리즈 직전 커널 메시지

⚠️ sudo 필요(/dev/kmsg 읽기). 완전 하드행(전CPU 정지)이면 마지막 메시지까지만,
   soft-lockup/hung-task/Oops면 스택트레이스 통째로 잡힘.
"""
import os
import argparse

# /dev/kmsg 레코드: "prio,seq,ts_usec,flag;message\n[ SUBSYS=.. ]"
PRIO = {0: 'EMERG', 1: 'ALERT', 2: 'CRIT', 3: 'ERR', 4: 'WARN',
        5: 'NOTICE', 6: 'INFO', 7: 'DEBUG'}


def parse(rec):
    try:
        meta, msg = rec.split(';', 1)
        fields = meta.split(',')
        prio = int(fields[0]) & 7
        ts = int(fields[2]) / 1e6
        return f'[{ts:12.6f}] {PRIO.get(prio, "?"):6} {msg.rstrip()}'
    except Exception:
        return rec.rstrip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/var/log/robot_control/kmsg_freeze.log')
    ap.add_argument('--from-start', action='store_true',
                    help='기존 버퍼부터 (기본: 지금부터)')
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    try:
        kf = os.open('/dev/kmsg', os.O_RDONLY | os.O_NONBLOCK)
    except PermissionError:
        print('⚠ sudo 필요: sudo python3 tools/diag/kmsg_fsync_logger.py'); return
    if not args.from_start:
        os.lseek(kf, 0, os.SEEK_END)     # 지금부터 (기존 버퍼 스킵)

    out = open(args.out, 'a')
    hdr = '=== kmsg fsync 로거 시작 (매 메시지 fsync → 전원차단에도 보존) ===\n'
    out.write(hdr); out.flush(); os.fsync(out.fileno())
    print(f'커널 로거 시작 → {args.out}. Ctrl+C 종료.')

    import select
    poller = select.poll()
    poller.register(kf, select.POLLIN)
    while True:
        if not poller.poll(1000):
            continue
        try:
            data = os.read(kf, 8192)
        except BlockingIOError:
            continue
        except OSError:
            continue
        for rec in data.decode(errors='replace').splitlines():
            if not rec:
                continue
            out.write(parse(rec) + '\n')
        out.flush()
        os.fsync(out.fileno())           # ← 전원 끊겨도 여기까지 디스크에 남음


if __name__ == '__main__':
    main()
