#!/usr/bin/env python3
"""주행모터 통신중단 보호시간(0xB3) 설정 — PC 프리즈/CAN 두절 시 자동 정지.

RMD 0xB3 "Communication Interruption Protection Time Setting":
  설정 시간(ms) 이상 CAN 명령이 끊기면 모터가 출력을 차단한다.
  0 = 비활성(기본값) → 통신이 끊겨도 마지막 명령을 계속 실행 (폭주 원인).
  값은 ROM에 저장되어 전원을 꺼도 유지된다.

대상은 속도명령(0xA2)으로 20Hz 연속 구동되는 주행모터 0x141/0x142 뿐이다.
스테이지(0x144~0x146)는 위치명령(0xA4)이라 평소 명령이 가지 않으므로
보호가 상시 발동한다 — 절대 적용하지 말 것.

  python3 tools/can/set_drive_comm_timeout.py --dry     # 보낼 프레임만 출력
  python3 tools/can/set_drive_comm_timeout.py           # 500ms 설정
  python3 tools/can/set_drive_comm_timeout.py --ms 0    # 보호 해제(원복)
"""
import argparse
import struct
import sys
import time

import can

CAN_IF = 'can2'
BITRATE = 1000000
DRIVE_MOTORS = (0x141, 0x142)
CMD_B3 = 0xB3
REPLY_OFFSET = 0x100      # 0x141 → 0x241


def build_frame(motor_id, ms):
    """0xB3 프레임: [0xB3,0,0,0, ms(int32 LE)]"""
    t = struct.pack('<i', ms)
    data = [CMD_B3, 0x00, 0x00, 0x00, t[0], t[1], t[2], t[3]]
    return can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ms', type=int, default=500,
                    help='통신중단 보호시간(ms). 0=비활성. 기본 500')
    ap.add_argument('--iface', default=CAN_IF)
    ap.add_argument('--motors', default=','.join(f'0x{m:03X}' for m in DRIVE_MOTORS),
                    help='대상 모터 CAN ID (쉼표구분)')
    ap.add_argument('--dry', action='store_true', help='전송 없이 프레임만 출력')
    args = ap.parse_args()

    motors = [int(m, 16) for m in args.motors.split(',')]
    if args.ms < 0:
        print('ms는 0 이상이어야 합니다'); return 1

    # 스테이지/요 모터 오적용 방지
    forbidden = [m for m in motors if m not in DRIVE_MOTORS]
    if forbidden and args.ms > 0:
        print('⚠️  주행모터(0x141/0x142) 외 대상이 지정되었습니다: '
              + ', '.join(f'0x{m:03X}' for m in forbidden))
        print('   위치명령 모터에 적용하면 보호가 상시 발동합니다.')
        if input('   그래도 진행하려면 "yes" 입력: ').strip() != 'yes':
            return 1

    print(f'대상: {", ".join(f"0x{m:03X}" for m in motors)}')
    print(f'보호시간: {args.ms} ms' + ('  (0 = 보호 해제)' if args.ms == 0 else ''))
    print(f'인터페이스: {args.iface}\n')

    for m in motors:
        f = build_frame(m, args.ms)
        print(f'  0x{m:03X} → ' + ' '.join(f'{b:02X}' for b in f.data))
    if args.dry:
        print('\n--dry: 전송하지 않음')
        return 0

    print()
    bus = can.Bus(args.iface, bustype='socketcan', bitrate=BITRATE)
    ok = 0
    try:
        for m in motors:
            # 이 모터의 응답만 남기기 위해 필터 설정
            bus.set_filters([{'can_id': m + REPLY_OFFSET,
                              'can_mask': 0x7FF, 'extended': False}])
            bus.send(build_frame(m, args.ms))
            # 서비스가 20Hz로 0xA2를 쏘고 있어 응답이 섞인다 → 0xB3 응답만 고른다
            got = None
            deadline = time.time() + 1.5
            while time.time() < deadline:
                reply = bus.recv(timeout=0.2)
                if reply is not None and reply.data and reply.data[0] == CMD_B3:
                    got = list(reply.data)
                    break
            if got is None:
                print(f'  ❌ 0x{m:03X}: 0xB3 응답 없음 (설정 실패 가능)')
                continue
            echoed = struct.unpack('<i', bytes(got[4:8]))[0] if len(got) >= 8 else None
            if got[0] == CMD_B3 and echoed == args.ms:
                print(f'  ✅ 0x{m:03X}: 응답 확인 — 보호시간 {echoed} ms (ROM 저장됨)')
                ok += 1
            else:
                print(f'  ⚠️  0x{m:03X}: 예상과 다른 응답 '
                      + ' '.join(f'{b:02X}' for b in got))
            time.sleep(0.05)
    finally:
        bus.shutdown()

    print(f'\n{ok}/{len(motors)} 설정 완료')
    if ok and args.ms > 0:
        print('\n⚠️  이제 CAN 명령이 '
              f'{args.ms}ms 이상 끊기면 주행모터가 자동 정지합니다.')
        print('   반드시 바퀴를 띄운 상태에서 동작 시험을 먼저 하세요.')
    return 0 if ok == len(motors) else 1


if __name__ == '__main__':
    sys.exit(main())
