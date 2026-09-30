#!/usr/bin/env python3
"""축의 엔코더 값을 한 줄로 읽는다. `/encoder_probe` 를 쏘고 응답을 해독한다.

## 배경

`0x92` 멀티턴 각도는 **전원을 내리면 사라진다.** 반면 `0x61 mod 262144` 는 단회전
절대 위치라 전원과 무관하다 (횡이동이 `lateral_home.json` 에 이 값을 저장해 쓴다).
그래서 자세를 저장해 두려면 **구동범위가 모터 1회전(=262144 counts) 안에 들어야** 한다.
넘으면 `mod` 결과가 같은 값이 여러 자세에 겹쳐 구별이 안 된다.

실측으로 확인된 것 (2026-09-30, 0x148):
  · 엔코더 18비트 = 262144 counts / 모터 1회전 (728.18 counts/도)
  · `0x92` 가 돌려주는 각도는 **모터축** 도다 (0x60 counts 환산값과 일치)
  · `0x90`·`0x94` 는 이 모터가 지원하지 않는다 (0x92 페이로드를 되돌려준다)
  · `0x60 = 0x61 - 0x62` (위치 = 원위치 - 영점오프셋)

## 쓰는 법

    python3 tools/test/enc_read.py yaw
    python3 tools/test/enc_read.py yaw --label "12시"      # 기록에 이름을 붙인다
    python3 tools/test/enc_read.py yaw --compare 123080    # 기준과의 차이를 낸다

읽은 값은 `tools/test/enc_log.json` 에 쌓인다. 여러 자세를 찍어 두고 나중에 범위를 본다.
"""

import argparse
import json
import os
import re
import struct
import subprocess
import sys
import time

CPR = 262144                 # 18비트, 모터 1회전 counts
CNT_PER_DEG = CPR / 360.0    # 728.18
GEAR = 12.5                  # RMD-X4-10 (axes.yaml stage.gear)
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'enc_log.json')


def trigger(axis):
    subprocess.run(
        ['ros2', 'topic', 'pub', '--once', '/encoder_probe',
         'std_msgs/msg/String', '{data: %r}' % axis],
        capture_output=True, timeout=30)


def read_journal(since_sec=25):
    r = subprocess.run(
        ['journalctl', '-u', 'rebar-teleop.service',
         '--since', f'{since_sec} seconds ago', '--no-pager'],
        capture_output=True, text=True, timeout=30)
    return r.stdout


def parse(text, motor_hex):
    """마지막 PROBE 응답들에서 명령별 int32 값을 뽑는다."""
    out = {}
    pat = re.compile(r'\[PROBE\] ' + motor_hex + r' ← 0x([0-9A-F]{2})  ([0-9A-F]{16})')
    for m in pat.finditer(text):
        cmd, payload = int(m.group(1), 16), bytes.fromhex(m.group(2))
        # 응답의 명령 바이트가 요청과 다르면 그 명령은 지원되지 않는 것이다
        if payload[0] != cmd:
            continue
        out[cmd] = struct.unpack('<i', payload[4:8])[0]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('axis', help="축 이름(x/y/z/yaw) 또는 CAN ID(0x148)")
    ap.add_argument('--label', default='', help='이 측정에 붙일 이름 (예: 12시, 좌측끝)')
    ap.add_argument('--compare', type=int, help='이 counts 값과의 차이를 낸다')
    a = ap.parse_args()

    ids = {'x': 0x145, 'y': 0x146, 'z': 0x147, 'yaw': 0x148}
    mid = ids.get(a.axis.lower())
    if mid is None:
        mid = int(a.axis, 0)
    motor_hex = f'0x{mid:03X}'

    trigger(a.axis)
    time.sleep(3.0)
    vals = parse(read_journal(), motor_hex)
    if not vals:
        print("실패 — PROBE 응답을 못 읽었습니다.")
        print("  · 서비스가 떠 있나요?  systemctl is-active rebar-teleop.service")
        print("  · journalctl 권한이 있나요?")
        return 1

    orig = vals.get(0x61)
    pos = vals.get(0x60)
    off = vals.get(0x62)
    ang = vals.get(0x92)

    print(f"■ {motor_hex}" + (f"  [{a.label}]" if a.label else ''))
    if ang is not None:
        print(f"  0x92 멀티턴 각도 : {ang*0.01:+10.2f}° (모터축)"
              f"   = 건 {ang*0.01/GEAR:+.2f}°   ⚠ 전원 내리면 사라짐")
    if pos is not None:
        print(f"  0x60 멀티턴 위치 : {pos:+10d} counts  = {pos/CNT_PER_DEG:+.2f}° (모터축)")
    if off is not None:
        print(f"  0x62 영점 오프셋 : {off:+10d} counts")
    if orig is not None:
        single = orig % CPR
        print(f"  0x61 원위치      : {orig:+10d} counts")
        print(f"  → 단회전 절대    : {single:10d} counts  = {single/CPR*360:.2f}° (모터축)"
              f"   ✅ 전원 무관")
        if a.compare is not None:
            d = orig - a.compare
            print(f"  → 기준 대비 차이 : {d:+d} counts = {d/CNT_PER_DEG:+.2f}° (모터축)"
                  f" = 건 {d/CNT_PER_DEG/GEAR:+.2f}°")
            turns = abs(d) / CPR
            print(f"     모터 회전수   : {turns:.3f} 회"
                  f"  {'→ 1회전 안 ✅' if turns < 1 else '→ 1회전 초과 ❌ 단회전 절대값으로 구별 불가'}")

    rec = dict(t=time.strftime('%Y-%m-%d %H:%M:%S'), motor=motor_hex, label=a.label,
               enc_orig=orig, enc_pos=pos, enc_off=off, angle_deg=(ang*0.01 if ang else None))
    try:
        hist = json.load(open(LOG)) if os.path.exists(LOG) else []
    except ValueError:
        hist = []
    hist.append(rec)
    json.dump(hist, open(LOG, 'w'), indent=2, ensure_ascii=False)

    same = [h for h in hist if h['motor'] == motor_hex and h.get('enc_orig') is not None]
    if len(same) > 1:
        lo = min(h['enc_orig'] for h in same)
        hi = max(h['enc_orig'] for h in same)
        span = hi - lo
        print(f"\n■ {motor_hex} 지금까지 {len(same)}회 측정")
        for h in same:
            print(f"    {h['t'][11:]}  {h['enc_orig']:+10d}  {h.get('label') or ''}")
        print(f"  범위 {lo:+d} ~ {hi:+d}  폭 {span} counts"
              f" = {span/CNT_PER_DEG:.2f}° (모터축) = 건 {span/CNT_PER_DEG/GEAR:.2f}°")
        print(f"  모터 {span/CPR:.3f} 회전  "
              f"{'→ 1회전 안에 든다 ✅ 단회전 절대값으로 자세 구별 가능' if span < CPR else '→ 1회전 초과 ❌'}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
