#!/usr/bin/env python3
"""Jetson 입력 전압 감시 — 주행 중 전압강하(브라운아웃) 포착용.

프리즈가 두 번 모두 "주행 중"에 발생했고 커널 로그에 아무 흔적이 없었다.
이는 소프트웨어 오류보다 전원 순단/전압강하를 시사한다. 이 스크립트는
INA3221로 입력 전압을 고속 샘플링하고 매 기록마다 flush + fsync 하므로,
전원이 끊겨도 직전까지의 전압 추이가 디스크에 남는다.

  python3 tools/diag/volt_monitor.py                    # 기본 감시
  python3 tools/diag/volt_monitor.py --sag-mv 17000     # 경보 임계 조정
  python3 tools/diag/volt_monitor.py --out /path/x.csv

출력 CSV: time,min_mv,max_mv,avg_mv,cur_ma,event
  - 매초 1줄(그 1초간의 min/max/avg)
  - 임계 미만 샘플이 나오면 즉시 SAG 줄 추가
"""
import argparse
import glob
import os
import time

# hwmon 번호(hwmon1)는 부팅마다 바뀔 수 있으므로 안정적인 i2c 장치경로(1-0040) 아래를
# glob으로 해석. 서비스가 조용히 실패하지 않도록 함.
_HWROOT = '/sys/bus/i2c/drivers/ina3221/1-0040/hwmon'
_cands = sorted(glob.glob(os.path.join(_HWROOT, 'hwmon*')))
HWMON = _cands[0] if _cands else os.path.join(_HWROOT, 'hwmon1')
VIN = os.path.join(HWMON, 'in1_input')      # 메인 입력 전압 (mV)
CIN = os.path.join(HWMON, 'curr1_input')    # 전류 (mA)
DEFAULT_OUT = '/var/log/robot_control/volt_monitor.csv'


def read_int(path):
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--hz', type=float, default=50.0, help='샘플링 주파수')
    ap.add_argument('--sag-mv', type=int, default=17000,
                    help='이 값 미만이면 즉시 SAG 기록 (기본 17000mV)')
    args = ap.parse_args()

    if not os.path.exists(VIN):
        print(f'전압 센서 없음: {VIN}')
        return 1

    out_dir = os.path.dirname(args.out)
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    period = 1.0 / args.hz
    f = open(args.out, 'a', buffering=1)     # line-buffered
    if f.tell() == 0:
        f.write('time,min_mv,max_mv,avg_mv,cur_ma,event\n')

    def emit(line):
        f.write(line)
        f.flush()
        os.fsync(f.fileno())                 # 전원이 끊겨도 남도록 강제 기록

    emit(f'{time.strftime("%Y-%m-%d %H:%M:%S")},,,,,START sag_threshold={args.sag_mv}mV\n')
    print(f'감시 시작 → {args.out}  (임계 {args.sag_mv}mV, {args.hz:.0f}Hz)')
    print('Ctrl+C 로 종료')

    samples = []
    bucket_start = time.time()
    try:
        while True:
            mv = read_int(VIN)
            if mv > 0:
                samples.append(mv)
                if mv < args.sag_mv:
                    emit(f'{time.strftime("%Y-%m-%d %H:%M:%S")},{mv},{mv},{mv},'
                         f'{read_int(CIN)},SAG\n')
            now = time.time()
            if now - bucket_start >= 1.0 and samples:
                emit(f'{time.strftime("%Y-%m-%d %H:%M:%S")},{min(samples)},'
                     f'{max(samples)},{sum(samples)//len(samples)},'
                     f'{read_int(CIN)},\n')
                samples = []
                bucket_start = now
            time.sleep(period)
    except KeyboardInterrupt:
        emit(f'{time.strftime("%Y-%m-%d %H:%M:%S")},,,,,STOP\n')
        print('\n종료')
    finally:
        f.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
