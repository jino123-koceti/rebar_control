#!/usr/bin/env python3
"""can2 의 RMD 모터 ID 를 훑는다 (0x9A, 읽기 전용). 보레이트 진단에 쓴다."""
import struct
import sys
import time

import can

CH = sys.argv[1] if len(sys.argv) > 1 else 'can2'
try:
    bus = can.interface.Bus(channel=CH, interface='socketcan')
except Exception as e:
    print(f"{CH} 열기 실패: {e}")
    sys.exit(1)
found = []
for cid in range(0x141, 0x149):
    got = None
    for _ in range(4):
        try:
            bus.send(can.Message(arbitration_id=cid, data=[0x9A] + [0] * 7,
                                 is_extended_id=False))
        except can.CanError:
            break
        t0 = time.time()
        while time.time() - t0 < 0.05:
            m = bus.recv(timeout=0.015)
            if m and m.arbitration_id == cid + 0x100 and m.data[0] == 0x9A:
                got = bytes(m.data)
                break
        if got:
            break
    if got:
        volt = struct.unpack('<H', bytes(got[4:6]))[0] * 0.1
        found.append(cid)
        print(f"0x{cid:03X} 응답 {volt:.1f}V 에러 0x{got[7]:02X}")
print("응답 ID: " + (' '.join(f"0x{c:03X}" for c in found) if found else "없음")
      + ("   ⚠ 0x145 응답!" if 0x145 in found else ""))
bus.shutdown()
