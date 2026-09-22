#!/usr/bin/env python3
"""RMD 모터 가감속(0x42 읽기 / 0x43 쓰기) 조회·설정 — 전류 피크 완화용.

## 왜 (2026-08-14)
주행 중 모터 전류가 평균 4.8A인데 **피크 28.3A**(6배)까지 튀고, 그 고부하 과도구간에서
**시스템 프리즈가 재현**됐다([[robot_freeze_safety]]).
소프트웨어 거버너(drive_controller)는 외부 루프라 반응이 한 박자 늦어 피크를 못 잡았다
(6A 제한을 걸고도 15.98A까지 올라가 프리즈).
→ **모터 자체의 가감속을 낮춰** 토크 상승률을 근본에서 제한한다.

## 프로토콜 (CAN BUS Motor Motion Protocol V4.3)
  0x42 읽기 / 0x43 쓰기(RAM+ROM). 값은 int32, 단위 1 dps/s, 범위 **100~60000**.
  Function index:
    0x00 위치계획 가속   0x01 위치계획 감속
    0x02 **속도계획 가속**  0x03 **속도계획 감속**   ← 주행(0xA2 속도제어)에 쓰이는 값
  프레임: [cmd, index, 0, 0, accel_b0, b1, b2, b3]

⚠ 0x43은 **ROM에도 쓴다** — 전원을 껐다 켜도 유지된다. 되돌리려면 원래 값을 기록해둘 것.
⚠ 주행 중에 바꾸지 말 것. 정지 상태에서만.

사용:
    python3 tools/motor/rmd_accel.py                    # 읽기만 (0x141,0x142)
    python3 tools/motor/rmd_accel.py --set 2000         # 속도계획 가/감속을 2000으로
    python3 tools/motor/rmd_accel.py --ids 0x141 --set 2000 --index 2
"""
import argparse
import struct
import time

import can

IDX_NAME = {0x00: '위치계획 가속', 0x01: '위치계획 감속',
            0x02: '속도계획 가속', 0x03: '속도계획 감속'}


def txrx(bus, motor_id, data, want_cmd, want_idx, timeout=0.4, retries=5):
    """명령 전송 후 **해당 명령·인덱스의** 응답만 골라 받는다.

    ⚠ `can_sender`가 같은 버스를 20~100Hz로 폴링하고 있어 응답 프레임이 섞인다.
       ID만 보고 첫 프레임을 집으면 0x92/0xA2 응답을 잘못 읽는다(실측 2026-08-14:
       쓰기 확인값이 None/이전값으로 나옴). → **cmd와 index까지 일치**해야 채택하고,
       못 찾으면 재시도한다.
    """
    for _ in range(retries):
        bus.send(can.Message(arbitration_id=motor_id, data=data,
                             is_extended_id=False))
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = bus.recv(timeout=timeout)
            if (m is not None and m.arbitration_id == motor_id + 0x100
                    and m.data[0] == want_cmd and m.data[1] == want_idx):
                return m
        time.sleep(0.05)
    return None


def read_accel(bus, motor_id, index):
    m = txrx(bus, motor_id, [0x42, index, 0, 0, 0, 0, 0, 0], 0x42, index)
    return None if m is None else struct.unpack('<i', bytes(m.data[4:8]))[0]


def write_accel(bus, motor_id, index, accel):
    b = struct.pack('<i', accel)
    m = txrx(bus, motor_id, [0x43, index, 0, 0, b[0], b[1], b[2], b[3]],
             0x43, index)
    return None if m is None else struct.unpack('<i', bytes(m.data[4:8]))[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', default='can2')
    ap.add_argument('--ids', nargs='+', default=['0x141', '0x142'])
    ap.add_argument('--index', type=int, default=None,
                    help='특정 인덱스만. 미지정 시 0~3 전부 읽기')
    ap.add_argument('--set', type=int, metavar='DPS_S',
                    help='속도계획 가/감속(0x02,0x03)을 이 값으로 설정 (100~60000)')
    a = ap.parse_args()

    ids = [int(x, 0) for x in a.ids]
    bus = can.interface.Bus(channel=a.channel, bustype='socketcan')
    try:
        for mid in ids:
            print(f"\n=== 모터 0x{mid:03X} ===")
            idxs = [a.index] if a.index is not None else [0, 1, 2, 3]
            for i in idxs:
                v = read_accel(bus, mid, i)
                print(f"  [{i}] {IDX_NAME.get(i, '?'):12s} = "
                      f"{v if v is not None else '응답 없음'} dps/s")
            if a.set is None:
                continue
            if not (100 <= a.set <= 60000):
                print(f"  ⚠ 범위 밖({a.set}) — 100~60000 이어야 함. 건너뜀")
                continue
            # ⚠ `--index`를 주면 **그 인덱스만** 쓴다. 안 주면 속도계획(0x02/0x03).
            #   (2026-08-14 버그: --index를 읽기에만 반영하고 쓰기는 항상 0x02/0x03에
            #    해서, 위치제어 모터인 0x143의 위치계획 값을 못 바꿨다.)
            targets = [a.index] if a.index is not None else [0x02, 0x03]
            for i in targets:
                w = write_accel(bus, mid, i, a.set)
                v = read_accel(bus, mid, i)
                ok = '✅' if v == a.set else '❌'
                print(f"  {ok} [{i}] {IDX_NAME[i]} → {a.set} (확인값 {v})")
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
