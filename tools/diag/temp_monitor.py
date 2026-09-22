#!/usr/bin/env python3
"""Jetson 온도/스로틀 감시 — 프리즈와 발열 상관관계 포착용.

프리즈 직전 커널 로그에 thermal hot-surface-alert 가 떴다. 4대 카메라 복귀로
발열이 오른 상태라, 온도 추이와 스로틀/팬 상태를 남겨 프리즈와의 상관을 본다.
volt_monitor 와 동일하게 매 기록마다 flush + fsync → 전원이 끊겨도 직전까지 보존.

  python3 tools/diag/temp_monitor.py                 # 기본 감시 (2Hz)
  python3 tools/diag/temp_monitor.py --hot-c 90      # 경보 임계 조정(°C)
  python3 tools/diag/temp_monitor.py --out /path/x.csv

출력 CSV: time,<zone별 °C...>,fan,hot_surf,throttle,event
  - 매 0.5초 1줄 (온도는 천천히 변하므로 고속 불필요)
  - 스로틀/핫서피스 cooling state가 0→양수로 뜨거나 임계온도 초과 시 event 표기
"""
import argparse
import glob
import os
import time


def _read_int(path):
    # 빈/에러 sysfs(cv0-thermal 등)는 OSError/ValueError뿐 아니라 codec TypeError도 냄 → 광범위 캐치
    try:
        with open(path) as f:
            return int(f.read().strip())
    except Exception:
        return None


def discover_zones():
    """{type: temp_path} — 값이 읽히는 thermal zone만 (cv0/1/2 등 빈 것 제외)."""
    zones = []
    for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        try:
            ztype = open(os.path.join(z, 'type')).read().strip()
        except OSError:
            continue
        tpath = os.path.join(z, 'temp')
        if _read_int(tpath) is not None:
            zones.append((ztype, tpath))
    return zones


def discover_cooling():
    """관심 cooling device 경로 묶음: 팬 / 핫서피스 / 스로틀알림."""
    fan, hot, throttle = [], [], []
    for c in sorted(glob.glob('/sys/class/thermal/cooling_device*')):
        try:
            ctype = open(os.path.join(c, 'type')).read().strip()
        except OSError:
            continue
        cur = os.path.join(c, 'cur_state')
        if 'fan' in ctype:
            fan.append(cur)
        elif 'hot-surface' in ctype:
            hot.append(cur)
        elif 'throttle-alert' in ctype:
            throttle.append(cur)
    return fan, hot, throttle


def _max_state(paths):
    vals = [v for v in (_read_int(p) for p in paths) if v is not None]
    return max(vals) if vals else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/var/log/robot_control/temp_monitor.csv')
    ap.add_argument('--hz', type=float, default=2.0, help='샘플링 주파수 (기본 2Hz)')
    ap.add_argument('--hot-c', type=float, default=90.0,
                    help='이 온도(°C) 초과하는 zone 있으면 HOT 기록 (기본 90)')
    args = ap.parse_args()

    zones = discover_zones()
    if not zones:
        print('thermal zone을 찾지 못함')
        return 1
    fan_p, hot_p, thr_p = discover_cooling()

    out_dir = os.path.dirname(args.out)
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    f = open(args.out, 'a', buffering=1)
    ztypes = [t for t, _ in zones]
    if f.tell() == 0:
        f.write('time,' + ','.join(ztypes) + ',fan,hot_surf,throttle,event\n')

    def emit(line):
        f.write(line)
        f.flush()
        os.fsync(f.fileno())                 # 전원이 끊겨도 남도록 강제 기록

    emit(f'{time.strftime("%Y-%m-%d %H:%M:%S")},'
         + ','.join('' for _ in ztypes)
         + f',,,,START hot_threshold={args.hot_c}C zones={"|".join(ztypes)}\n')
    print(f'온도 감시 시작 → {args.out}  (임계 {args.hot_c}°C, {args.hz:.0f}Hz)')
    print(f'  zones: {", ".join(ztypes)}')
    print('Ctrl+C 로 종료')

    period = 1.0 / args.hz
    prev_throttle = 0
    prev_hot = 0
    try:
        while True:
            temps_c = []
            hot_zone = None
            for ztype, tpath in zones:
                mc = _read_int(tpath)
                c = (mc / 1000.0) if mc is not None else None
                temps_c.append(c)
                if c is not None and c > args.hot_c:
                    hot_zone = f'{ztype}={c:.1f}'

            fan = _max_state(fan_p)
            hot = _max_state(hot_p)
            throttle = _max_state(thr_p)

            events = []
            if hot_zone:
                events.append(f'HOT:{hot_zone}')
            # 스로틀/핫서피스가 0→양수로 올라간 순간을 이벤트로 강조
            if throttle > 0 and prev_throttle == 0:
                events.append('THROTTLE_ON')
            if hot > 0 and prev_hot == 0:
                events.append('HOTSURF_ON')
            prev_throttle, prev_hot = throttle, hot

            row = ','.join(f'{c:.1f}' if c is not None else '' for c in temps_c)
            emit(f'{time.strftime("%Y-%m-%d %H:%M:%S")},{row},'
                 f'{fan},{hot},{throttle},{" ".join(events)}\n')
            time.sleep(period)
    except KeyboardInterrupt:
        emit(f'{time.strftime("%Y-%m-%d %H:%M:%S")},'
             + ','.join('' for _ in ztypes) + ',,,,STOP\n')
        print('\n종료')
    finally:
        f.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
