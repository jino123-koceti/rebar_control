#!/usr/bin/env python3
"""
모터 알람 상태 확인 스크립트
0x141 (왼쪽), 0x142 (오른쪽) 모터의 에러 상태를 확인합니다.
"""
import ctypes, time, struct, logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
log = logging.getLogger('motor_alarm_check')

USBCAN_DEVICE = 4
DEVICE_INDEX = 0

CMDS = {
    'SYSTEM_RESET': 0x76,
    'STATUS1_9A': 0x9A,
    'RUNMODE_70': 0x70,
}

# RMD 모터 에러 비트 정의
ERROR_BITS = {
    0x0001: 'Low voltage protection',
    0x0002: 'Over voltage protection',
    0x0004: 'Over current protection',
    0x0008: 'Over temperature protection (MOS)',
    0x0010: 'Magnetic encoding error',
    0x0020: 'Hall encoding error',
    0x0040: 'Motor temperature sensor error',
    0x0080: 'Reserved',
    0x0100: 'Motor stall',
    0x0200: 'Motor block',
    0x0400: 'Reserved',
    0x0800: 'Reserved',
    0x1000: 'Reserved',
    0x2000: 'Reserved',
    0x4000: 'Reserved',
    0x8000: 'Reserved',
}

class INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint32), ("AccMask", ctypes.c_uint32), ("Reserved", ctypes.c_uint32),
        ("Filter", ctypes.c_uint8), ("Timing0", ctypes.c_uint8), ("Timing1", ctypes.c_uint8), ("Mode", ctypes.c_uint8)
    ]

class CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint32), ("TimeStamp", ctypes.c_uint32), ("TimeFlag", ctypes.c_uint8), ("SendType", ctypes.c_uint8),
        ("RemoteFlag", ctypes.c_uint8), ("ExternFlag", ctypes.c_uint8), ("DataLen", ctypes.c_uint8), ("Data", ctypes.c_uint8 * 8),
        ("Reserved", ctypes.c_uint8 * 3)
    ]

def tx(lib, data: bytes, can_id):
    obj = CAN_OBJ()
    obj.ID = can_id
    obj.DataLen = len(data)
    obj.SendType = 0
    obj.RemoteFlag = 0
    obj.ExternFlag = 0
    for i, b in enumerate(data[:8]):
        obj.Data[i] = b
    r = lib.VCI_Transmit(USBCAN_DEVICE, DEVICE_INDEX, 0, ctypes.byref(obj), 1)
    log.debug(f"TX: ID=0x{can_id:03X} DATA={data.hex().upper()} res={r}")
    return r == 1

def rx(lib, timeout_ms=500, expected_id=None):
    obj = CAN_OBJ()
    start = time.time()
    while (time.time() - start) * 1000 < timeout_ms:
        r = lib.VCI_Receive(USBCAN_DEVICE, DEVICE_INDEX, 0, ctypes.byref(obj), 1, 50)
        if r > 0:
            data = bytes(obj.Data[:obj.DataLen])
            if expected_id is None or obj.ID == expected_id:
                log.debug(f"RX: ID=0x{obj.ID:03X} LEN={obj.DataLen} DATA={data.hex().upper()}")
                return obj.ID, data
        time.sleep(0.001)
    return None, None

def parse_status_9a(data: bytes):
    if len(data) < 8 or data[0] != CMDS['STATUS1_9A']:
        return None
    temp_motor = data[1]
    temp_mos = data[2]
    brake_state = data[3]
    voltage = (data[5] << 8 | data[4]) / 10.0  # 0.1V/LSB
    err = (data[7] << 8 | data[6])
    return {
        'motor_temp': temp_motor,
        'mos_temp': temp_mos,
        'brake_state': brake_state,
        'voltage_V': voltage,
        'error_bits': err
    }

def decode_error_bits(error_bits):
    """에러 비트를 해석하여 리스트로 반환"""
    errors = []
    for bit, description in ERROR_BITS.items():
        if error_bits & bit:
            errors.append(description)
    return errors

def check_motor(lib, motor_id):
    """특정 모터의 상태를 확인"""
    motor_name = "Left (0x141)" if motor_id == 0x141 else "Right (0x142)"

    print(f"\n{'='*60}")
    print(f"  모터 상태 확인: {motor_name}")
    print(f"{'='*60}")

    # Query runmode
    tx(lib, bytes([CMDS['RUNMODE_70']]) + b'\x00'*7, motor_id)
    _id, data = rx(lib, expected_id=motor_id)
    if data and len(data) >= 8 and data[0] == CMDS['RUNMODE_70']:
        runmode = data[7]
        modes = {0x00:'Stop', 0x01:'Current', 0x02:'Speed', 0x03:'Position'}
        print(f"  Run Mode: 0x{runmode:02X} ({modes.get(runmode,'Unknown')})")
    else:
        print(f"  ⚠️  Run Mode: 응답 없음")

    time.sleep(0.1)

    # Query status 0x9A
    tx(lib, bytes([CMDS['STATUS1_9A']]) + b'\x00'*7, motor_id)
    _id, data = rx(lib, expected_id=motor_id)
    st = parse_status_9a(data) if data else None

    if st:
        print(f"  전압: {st['voltage_V']:.1f}V")
        print(f"  모터 온도: {st['motor_temp']}°C")
        print(f"  MOS 온도: {st['mos_temp']}°C")
        print(f"  브레이크 상태: {st['brake_state']}")
        print(f"  에러 비트: 0x{st['error_bits']:04X}")

        if st['error_bits'] != 0:
            errors = decode_error_bits(st['error_bits'])
            print(f"\n  🚨 알람 발생!")
            for i, err in enumerate(errors, 1):
                print(f"    {i}. {err}")
        else:
            print(f"  ✅ 정상 (알람 없음)")
    else:
        print(f"  ⚠️  상태 조회 실패: 응답 없음")

    print(f"{'='*60}")

def main():
    print("\n🔍 RMD 모터 알람 상태 확인")
    print("=" * 60)

    lib = ctypes.CDLL('/lib/libusbcan.so')

    # CAN 장치 열기
    if lib.VCI_OpenDevice(USBCAN_DEVICE, DEVICE_INDEX, 0) != 1:
        log.error('❌ CAN 장치 열기 실패')
        return

    # CAN 초기화 (1Mbps)
    init = INIT_CONFIG()
    init.AccCode = 0
    init.AccMask = 0xFFFFFFFF
    init.Reserved = 0
    init.Filter = 0
    init.Timing0 = 0x00
    init.Timing1 = 0x14
    init.Mode = 0

    if lib.VCI_InitCAN(USBCAN_DEVICE, DEVICE_INDEX, 0, ctypes.byref(init)) != 1:
        log.error('❌ CAN 초기화 실패')
        lib.VCI_CloseDevice(USBCAN_DEVICE, DEVICE_INDEX)
        return

    if lib.VCI_StartCAN(USBCAN_DEVICE, DEVICE_INDEX, 0) != 1:
        log.error('❌ CAN 시작 실패')
        lib.VCI_CloseDevice(USBCAN_DEVICE, DEVICE_INDEX)
        return

    print("✅ CAN 장치 초기화 완료 (1Mbps)\n")

    # 0x141 (왼쪽 모터) 확인
    check_motor(lib, 0x141)

    time.sleep(0.2)

    # 0x142 (오른쪽 모터) 확인
    check_motor(lib, 0x142)

    print("\n")

    # CAN 장치 닫기
    lib.VCI_CloseDevice(USBCAN_DEVICE, DEVICE_INDEX)
    print("✅ CAN 장치 종료\n")

if __name__ == '__main__':
    main()
