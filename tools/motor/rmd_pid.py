#!/usr/bin/env python3
"""RMD PI 게인(0x30 읽기 / 0x31 RAM쓰기 / 0x32 ROM쓰기) 조회·설정.

## 왜 (2026-09-09)
0x141을 X4-36으로 교체해 좌우 모델이 달라졌다. 가감속(0x43)을 양쪽 2000으로
맞추고 토크 상한도 절대값을 맞췄는데도 **X4-36이 램프에서 뒤처진다**.
전류가 상한의 47%밖에 안 되니 **토크 부족이 아니다** → 남는 건 속도루프 게인.
모델별 공장 기본 PI가 다르면 같은 지령에도 응답이 다르다.

## ★ 2026-09-14 — **이 도구는 형식을 틀리게 읽고 있었다**
지금까지 "0x30이 전부 0으로 읽힌다 → 이 펌웨어는 0x30을 지원 안 한다"고 결론내고
속도루프 게인 조사를 접었다([[drive_motor_0x141_stall_2026-09-09]]). **틀렸다.**
실측해 보니 이 펌웨어(버전일자 2026042402)의 0x30은 **바이트 6개가 아니라
인덱스 1개 + float32** 다 — 0x42(가감속)와 같은 꼴이다:

    요청  [0x30, index, 0, 0, 0, 0, 0, 0]
    응답  [0x30, index, 0, 0, float32(LE)]      ← DATA[4:8]

옛 형식으로 index=0 자리에 아무것도 안 넣고 DATA[2:8]을 uint8 6개로 읽으니
**index 0의 값(0)과 패딩만 보여서 "전부 0"** 으로 보였던 것이다. 버스 점유 탓이
아니다 — 같은 순간 같은 필터로 0x42는 1200을 정확히 읽었다.

실측값 (0x141·0x142 **완전히 동일**, 서비스 가동 중에도 안정적으로 읽힘):
    index 0 = 0        index 1 = 1.5      index 2 = 0.05
    index 3 = 0        index 4 = 0.01     index 5 = 0.0001

⚠ **인덱스가 각각 무슨 게인인지는 아직 모른다.** 데이터시트의 index 표를 확인하기
   전에는 쓰지 말 것 — 그래서 이 도구의 쓰기 기능은 **막아뒀다**(아래).
   잘못 짚으면 구동 중인 모터의 루프 게인이 날아간다.

## 프로토콜
  0x30 읽기 / 0x31 RAM 쓰기 / 0x32 ROM 쓰기
⚠ RAM 쓰기(0x31)는 전원을 끄면 사라진다 — **시험은 RAM으로** 하고, 좋으면 ROM.
⚠ 게인을 올리면 발진할 수 있다. 조금씩 올리며 소리·진동을 확인할 것.
⚠ 정지 상태에서만.

## 사용
    python3 tools/motor/rmd_pid.py                          # 읽기 (index 0~5)
"""
import argparse
import struct
import time

import can

# ⚠ 이름을 아직 모른다. 옛 코드의 ['angleKp','angleKi','speedKp','speedKi','iqKp','iqKi']는
#   **바이트 6개 형식**을 전제한 것이라 지금 형식엔 대응이 보장되지 않는다.
#   데이터시트 index 표를 확인하면 여기에 채워 넣을 것.
N_INDEX = 6


def txrx(bus, mid, data, want, want_idx=None, timeout=0.5, retries=8):
    """해당 명령·인덱스의 응답만 골라 받는다 — can_sender가 같은 버스를 폴링한다.

    ⚠ 인덱스까지 봐야 한다. cmd만 보면 다른 인덱스의 응답을 집는다(0x42와 같은 함정).
    """
    for _ in range(retries):
        bus.send(can.Message(arbitration_id=mid, data=data, is_extended_id=False))
        t0 = time.time()
        while time.time() - t0 < timeout:
            m = bus.recv(timeout=0.1)
            if m and m.arbitration_id == mid + 0x100 and m.data[0] == want:
                if want_idx is not None and m.data[1] != want_idx:
                    continue
                return bytes(m.data)
    return None


def read_index(bus, mid, idx):
    """0x30 index → float32. 못 읽으면 None."""
    r = txrx(bus, mid, [0x30, idx, 0, 0, 0, 0, 0, 0], 0x30, idx)
    return struct.unpack('<f', r[4:8])[0] if r else None


def read(bus, mid):
    return [read_index(bus, mid, i) for i in range(N_INDEX)]


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__)
    ap.add_argument('--channel', default='can2')
    ap.add_argument('--ids', nargs='*', default=['0x141', '0x142'])
    # ⚠ 쓰기 옵션은 **일부러 남겨두고 막는다** — 예전 사용법을 기억하고 그대로 치면
    #   조용히 아무 일도 안 일어나는 것보다, 왜 막혔는지 읽고 가는 편이 낫다.
    ap.add_argument('--speed-kp', type=int, dest='skp', help='(사용 불가 — 아래 설명)')
    ap.add_argument('--speed-ki', type=int, dest='ski', help='(사용 불가)')
    ap.add_argument('--copy-from', dest='copy', help='(사용 불가)')
    ap.add_argument('--rom', action='store_true', help='(사용 불가)')
    a = ap.parse_args()
    ids = [int(x, 0) for x in a.ids]

    bus = can.interface.Bus(channel=a.channel, interface='socketcan')
    cur = {i: read(bus, i) for i in ids}
    bus.shutdown()

    print(f'\n{"":10}' + ''.join(f'{"idx " + str(i):>12}' for i in range(N_INDEX)))
    print('-' * (10 + 12 * N_INDEX))
    for i in ids:
        row = ''.join(f'{"응답없음" if v is None else f"{v:.6g}":>12}' for v in cur[i])
        print(f'0x{i:03X}    {row}')
    print('-' * (10 + 12 * N_INDEX))

    # 좌우 대조 — 이게 이 도구의 실질적인 쓸모다.
    if len(ids) == 2:
        l, r = cur[ids[0]], cur[ids[1]]
        diff = [i for i in range(N_INDEX)
                if l[i] is not None and r[i] is not None and l[i] != r[i]]
        if any(v is None for v in l + r):
            print('  ⚠ 일부 인덱스를 못 읽었다 — 재시도할 것')
        elif diff:
            print(f'  ⚠ 좌우 불일치: index {diff} → 직진이 틀어지는 원인이 될 수 있다')
        else:
            print('  ✅ 좌우 게인 완전 일치 — 직진 틀어짐의 원인은 여기가 아니다')

    if a.skp is not None or a.ski is not None or a.copy or a.rom:
        print('\n❌ 쓰기는 막아뒀다.')
        print('   이 펌웨어의 0x30/0x31/0x32는 **index + float32** 형식인데,')
        print('   어느 index가 speedKp/speedKi인지 아직 확인이 안 됐다.')
        print('   옛 바이트 6개 형식으로 쓰면 index 0에 엉뚱한 float가 들어가')
        print('   **구동 중인 루프 게인이 날아간다.**')
        print('   → 데이터시트의 0x30 index 표를 확인해 NAMES를 채운 뒤 다시 열 것.')


main()
