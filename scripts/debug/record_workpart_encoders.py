#!/usr/bin/env python3
"""
상부 작업부 4축 멀티턴 절대 엔코더 기록 도구 (kinematics 캘리브레이션용)

목적
  멀티턴 저장(0x20 idx=0x04) 활성화 후, X/Y/Z/Yaw(0x144~0x147)의
  0x92 멀티턴 절대각을 "알고 있는 물리위치(x,y mm)" 또는 "라벨"과 함께
  누적 기록한다.
  - 각 리미트 센서 위치, yaw 한계 위치, 임의의 결속포인트 등에서 샘플 채취
  - 이렇게 모은 (x_mm, y_mm, enc_x, enc_y, enc_yaw) 표로부터
    base frame 정의 / deg_per_mm / 레버암·장착각 등 기구학 파라미터를 도출

읽는 CAN 명령 (test_rmd_multiturn.py snapshot 과 동일 파싱) — 4축 전부 풀세트
  - 0x60 multi-turn pos (raw-offset)  int32
  - 0x61 multi-turn raw position      int32
  - 0x62 multi-turn zero offset       int32
  - 0x90 single-turn encoder          int32 (16bit는 &0xFFFF)
  - 0x92 multi-turn angle             int32, 0.01°/LSB  ← 절대각 후보
  - 0x94 single-turn angle            uint16, 0.01°/LSB
  나중에 어떤 값이 진짜 절대값인지 분석으로 가릴 수 있게 전부 같이 기록한다.

CAN bus: can2 @ 1Mbps
  X=0x144, Y=0x145, Z=0x146, Yaw=0x147   (응답 ID = +0x100)

⚠️  실행 전 robot-control 서비스 중지 (can2 충돌 방지):
    sudo systemctl stop robot-control

사용법
  python3 scripts/debug/record_workpart_encoders.py
  python3 scripts/debug/record_workpart_encoders.py --out data/calibration/workpart_enc.csv

대화형 입력
  Enter(빈 입력)  → 현재 4축 엔코더를 한 줄 기록 (의미는 나중에 분석 시 부여)
  아무 텍스트     → 기록 + 그 텍스트를 note로 함께 저장 (선택)
  p               → 저장 없이 현재값만 미리보기
  u               → 마지막 샘플 취소(undo)
  l               → 지금까지 기록 목록 출력
  s               → 즉시 파일 저장
  q               → 저장 후 종료
"""
import argparse
import csv
import json
import os
import struct
import sys
import time
from datetime import datetime

import can


CAN_IFACE = 'can2'
BITRATE = 1_000_000

AXES = [
    (0x144, 'x'),
    (0x145, 'y'),
    (0x146, 'z'),
    (0x147, 'yaw'),
]


def send_cmd(bus, motor_id, payload):
    msg = can.Message(
        arbitration_id=motor_id,
        data=payload + bytes(8 - len(payload)),
        is_extended_id=False,
    )
    bus.send(msg)


def read_reply(bus, motor_id, expected_cmd, timeout=0.5):
    response_id = motor_id + 0x100
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=remaining)
        if msg is None:
            continue
        if msg.arbitration_id == response_id and msg.data[0] == expected_cmd:
            return bytes(msg.data)


# 읽을 CAN 명령 정의 (test_rmd_multiturn.py snapshot 과 동일 파싱)
#   key,  cmd,  파서종류,        스케일,  설명
#   파서종류: 'i32' = int32 @ data[4:8],  'u16t' = uint16 @ data[6:8]
READS = [
    ('0x60', 0x60, 'i32',  1.0,  'multi-turn pos (raw-offset)'),
    ('0x61', 0x61, 'i32',  1.0,  'multi-turn raw position'),
    ('0x62', 0x62, 'i32',  1.0,  'multi-turn zero offset'),
    ('0x90', 0x90, 'i32',  1.0,  'single-turn enc (16bit는 &0xFFFF)'),
    ('0x92', 0x92, 'i32',  0.01, 'multi-turn angle (deg)'),
    ('0x94', 0x94, 'u16t', 0.01, 'single-turn angle (deg)'),
]


def _parse(reply, kind):
    if kind == 'i32':
        return struct.unpack('<i', reply[4:8])[0]
    if kind == 'u16t':
        return struct.unpack('<H', reply[6:8])[0]
    return None


def read_value(bus, motor_id, cmd, kind, retries=3):
    """단일 명령 raw 값. 실패 시 None."""
    for _ in range(retries):
        send_cmd(bus, motor_id, bytes([cmd]))
        reply = read_reply(bus, motor_id, cmd, timeout=0.4)
        if reply is not None:
            return _parse(reply, kind)
        time.sleep(0.02)
    return None


def read_all_axes(bus):
    """4축 × (0x60/0x61/0x62/0x90/0x92/0x94) 전체를 dict로 반환.
    키 형식: '{axis}_{key}' (raw), 0x92/0x94는 '{axis}_{key}_deg' 추가."""
    data = {}
    for motor_id, name in AXES:
        for key, cmd, kind, scale, _desc in READS:
            raw = read_value(bus, motor_id, cmd, kind)
            data[f'{name}_{key}'] = raw
            if scale != 1.0:
                data[f'{name}_{key}_deg'] = (raw * scale) if raw is not None else None
    return data


def print_reading(data, idx=None, note=''):
    """0x92(deg)·0x90을 한눈에, 나머지 raw는 줄여서 표시."""
    def deg92(name):
        d = data.get(f'{name}_0x92_deg')
        return f'{d:9.2f}°' if d is not None else '  <none>'
    tag = f'#{idx}' if idx is not None else '(preview)'
    if note:
        tag += f'  note="{note}"'
    print(f"    {tag}")
    for _mid, name in AXES:
        print(f"      {name:>3s}: 0x92={deg92(name)}  "
              f"0x90={data.get(f'{name}_0x90')}  "
              f"0x60={data.get(f'{name}_0x60')}  "
              f"0x61={data.get(f'{name}_0x61')}  "
              f"0x62={data.get(f'{name}_0x62')}  "
              f"0x94deg={data.get(f'{name}_0x94_deg')}")


# CSV 컬럼: 메타 + 축별 전체 raw/deg
CSV_FIELDS = ['index', 'timestamp', 'note']
for _mid, _name in AXES:
    for _key, _cmd, _kind, _scale, _desc in READS:
        CSV_FIELDS.append(f'{_name}_{_key}')
        if _scale != 1.0:
            CSV_FIELDS.append(f'{_name}_{_key}_deg')


def save_samples(samples, out_path):
    if not samples:
        print("  (기록된 샘플 없음 — 저장 생략)")
        return
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for s in samples:
            writer.writerow({k: s.get(k, '') for k in CSV_FIELDS})
    json_path = os.path.splitext(out_path)[0] + '.json'
    with open(json_path, 'w') as f:
        json.dump(samples, f, indent=2, ensure_ascii=False)
    print(f"  ✅ {len(samples)}개 샘플 저장: {out_path}")
    print(f"                            {json_path}")


def parse_input(line):
    """입력 파싱 → ('record', note) 또는 단순 명령 튜플."""
    s = line.strip()
    if s in ('q', 'Q'):
        return ('quit',)
    if s in ('s', 'S'):
        return ('save',)
    if s in ('u', 'U'):
        return ('undo',)
    if s in ('l', 'L'):
        return ('list',)
    if s in ('p', 'P'):
        return ('preview',)
    # 빈 입력 = 기록, 그 외 텍스트 = 기록 + note
    return ('record', s)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--iface', default=CAN_IFACE,
                        help=f'CAN interface (default {CAN_IFACE})')
    parser.add_argument('--out', default='data/calibration/workpart_enc.csv',
                        help='저장 경로 (CSV, 동일 이름 .json 동시 생성)')
    args = parser.parse_args()

    print("=" * 70)
    print("상부 작업부 4축 멀티턴 엔코더 기록 (kinematics 캘리브레이션)")
    print("=" * 70)
    print(f"CAN: {args.iface} @ {BITRATE} bps   축: X=0x144 Y=0x145 Z=0x146 Yaw=0x147")
    print(f"저장: {args.out}")
    print()
    print("⚠️  robot-control 서비스가 떠 있으면 CAN 응답 경쟁 발생.")
    print("    먼저:  sudo systemctl stop robot-control")
    print()
    print("입력: Enter(기록) / 텍스트(기록+note) / p(미리보기) / u(취소) / l(목록) / s(저장) / q(종료)")
    print()

    bus = can.interface.Bus(channel=args.iface, interface='socketcan', bitrate=BITRATE)
    samples = []

    try:
        while True:
            try:
                line = input("기록(Enter)> ")
            except (EOFError, KeyboardInterrupt):
                print()
                break

            parsed = parse_input(line)
            cmd = parsed[0]

            if cmd == 'quit':
                break
            elif cmd == 'save':
                save_samples(samples, args.out)
            elif cmd == 'undo':
                if samples:
                    removed = samples.pop()
                    print(f"  ↩️  샘플 #{removed.get('index')} 취소")
                else:
                    print("  (취소할 샘플 없음)")
            elif cmd == 'list':
                if not samples:
                    print("  (아직 없음)")
                for s in samples:
                    print(f"  #{s.get('index')} {s.get('note',''):10s} "
                          f"X={s.get('x_0x92_deg')} Y={s.get('y_0x92_deg')} "
                          f"Z={s.get('z_0x92_deg')} Yaw={s.get('yaw_0x92_deg')}")
            elif cmd == 'preview':
                data = read_all_axes(bus)
                print_reading(data)
            elif cmd == 'record':
                note = parsed[1]
                data = read_all_axes(bus)
                idx = len(samples)
                print_reading(data, idx, note)
                missing = [n for _, n in AXES if data.get(f'{n}_0x92') is None]
                if missing:
                    print(f"  ⚠️  0x92 응답 없는 축: {missing} — 기록은 하되 값 확인 필요")
                sample = {
                    'index': idx,
                    'timestamp': datetime.now().isoformat(timespec='seconds'),
                    'note': note,
                }
                for k in CSV_FIELDS[3:]:
                    sample[k] = data.get(k)
                samples.append(sample)
                print(f"  ✔ #{idx} 기록됨 (총 {len(samples)}개)")

    finally:
        print()
        save_samples(samples, args.out)
        bus.shutdown()
        print("종료.")


if __name__ == '__main__':
    main()
