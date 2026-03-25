#!/usr/bin/env python3
"""
횡이동 모터(0x143) 실시간 부하 모니터링

연속 횡이동 수행 시 온도, 전류, 전압, 에러 상태를 실시간 추적
Ctrl+C로 종료 시 요약 통계 출력
"""

import can
import struct
import time
import sys
from datetime import datetime

MOTOR_ID = 0x143
BUS_NAME = 'can2'
BITRATE = 1000000
MONITOR_HZ = 5  # 모니터링 주기

# RMD 에러 비트
ERROR_BITS = {
    0x0001: 'Low voltage',
    0x0002: 'Over voltage',
    0x0004: 'Over current',
    0x0008: 'Over temp (MOS)',
    0x0010: 'Magnetic enc err',
    0x0020: 'Hall enc err',
    0x0040: 'Temp sensor err',
    0x0100: 'Motor stall',
    0x0200: 'Motor block',
}


def send_and_recv(bus, motor_id, cmd_byte, resp_id=None, timeout=0.1):
    """명령 전송 후 응답 수신"""
    msg = can.Message(
        arbitration_id=motor_id,
        data=[cmd_byte, 0, 0, 0, 0, 0, 0, 0],
        is_extended_id=False
    )
    if resp_id is None:
        resp_id = motor_id + 0x100

    # 버퍼 비우기
    while bus.recv(timeout=0.005):
        pass

    bus.send(msg)
    end = time.time() + timeout
    while time.time() < end:
        resp = bus.recv(timeout=0.02)
        if resp and resp.arbitration_id == resp_id and resp.data[0] == cmd_byte:
            return resp.data
    return None


def read_status_9a(bus, motor_id):
    """0x9A: 온도, 전압, 에러"""
    data = send_and_recv(bus, motor_id, 0x9A)
    if data is None or len(data) < 8:
        return None
    return {
        'motor_temp': data[1],       # °C
        'mos_temp': data[2],         # °C (지원 시)
        'voltage': struct.unpack('<H', bytes(data[4:6]))[0] / 10.0,  # V
        'error': struct.unpack('<H', bytes(data[6:8]))[0],
    }


def read_status_9c(bus, motor_id):
    """0x9C: 온도, 전류, 속도, 위치"""
    data = send_and_recv(bus, motor_id, 0x9C)
    if data is None or len(data) < 8:
        return None
    return {
        'temp': data[1],                                              # °C
        'current': abs(struct.unpack('<h', bytes(data[2:4]))[0]) * 0.01,  # A (0.01A/LSB, signed)
        'speed': struct.unpack('<h', bytes(data[4:6]))[0],           # dps (signed)
        'position': struct.unpack('<h', bytes(data[6:8]))[0],        # encoder
    }


def decode_errors(error_bits):
    return [desc for bit, desc in ERROR_BITS.items() if error_bits & bit]


def main():
    print("=" * 80)
    print(f"  Lateral Motor (0x{MOTOR_ID:03X}) Load Monitor")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80)
    print("Ctrl+C to stop\n")

    bus = can.Bus(interface='socketcan', channel=BUS_NAME, bitrate=BITRATE)

    # 초기 연결 확인
    st = read_status_9a(bus, MOTOR_ID)
    if st is None:
        print(f"0x{MOTOR_ID:03X} not responding!")
        bus.shutdown()
        return

    print(f"Connected: voltage={st['voltage']:.1f}V, temp={st['motor_temp']}°C\n")

    # 통계 변수
    stats = {
        'max_temp': 0, 'max_current': 0.0, 'max_speed': 0,
        'min_voltage': 999.0, 'max_voltage': 0.0,
        'error_count': 0, 'sample_count': 0,
        'temp_sum': 0.0, 'current_sum': 0.0,
    }

    # 헤더
    header = f"{'Time':>8} | {'Temp':>5} | {'Current':>8} | {'Speed':>7} | {'Voltage':>7} | {'Error':>6} | Status"
    print(header)
    print("-" * len(header))

    try:
        start_time = time.time()

        while True:
            loop_start = time.time()

            s9a = read_status_9a(bus, MOTOR_ID)
            s9c = read_status_9c(bus, MOTOR_ID)

            if s9a is None and s9c is None:
                elapsed = time.time() - start_time
                print(f"{elapsed:7.1f}s | {'--- NO RESPONSE ---':^60}")
                time.sleep(1.0 / MONITOR_HZ)
                continue

            temp = s9c['temp'] if s9c else (s9a['motor_temp'] if s9a else 0)
            current = s9c['current'] if s9c else 0.0
            speed = s9c['speed'] if s9c else 0
            voltage = s9a['voltage'] if s9a else 0.0
            error = s9a['error'] if s9a else 0

            # 통계 업데이트
            stats['sample_count'] += 1
            stats['max_temp'] = max(stats['max_temp'], temp)
            stats['max_current'] = max(stats['max_current'], current)
            stats['max_speed'] = max(stats['max_speed'], abs(speed))
            stats['min_voltage'] = min(stats['min_voltage'], voltage)
            stats['max_voltage'] = max(stats['max_voltage'], voltage)
            stats['temp_sum'] += temp
            stats['current_sum'] += current
            if error:
                stats['error_count'] += 1

            # 상태 표시
            status = "OK"
            if error:
                errors = decode_errors(error)
                status = ', '.join(errors)
            elif temp >= 60:
                status = "WARM"
            elif temp >= 70:
                status = "HOT!"

            elapsed = time.time() - start_time
            line = f"{elapsed:7.1f}s | {temp:4d}C | {current:7.2f}A | {speed:+6d} | {voltage:6.1f}V | 0x{error:04X} | {status}"
            print(line)

            # 경고
            if temp >= 70:
                print(f"  >>> WARNING: High temperature {temp}°C!")
            if current > 3.0:
                print(f"  >>> WARNING: High current {current:.2f}A!")
            if error:
                print(f"  >>> ALARM: {decode_errors(error)}")

            # 주기 유지
            sleep_time = (1.0 / MONITOR_HZ) - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - start_time
        n = stats['sample_count']

        print("\n" + "=" * 80)
        print("  SUMMARY")
        print("=" * 80)
        print(f"  Duration     : {elapsed:.1f}s ({n} samples)")
        if n > 0:
            print(f"  Temperature  : avg {stats['temp_sum']/n:.1f}°C, max {stats['max_temp']}°C")
            print(f"  Current      : avg {stats['current_sum']/n:.2f}A, max {stats['max_current']:.2f}A")
            print(f"  Speed (max)  : {stats['max_speed']} dps")
            print(f"  Voltage      : {stats['min_voltage']:.1f}V ~ {stats['max_voltage']:.1f}V")
            print(f"  Errors       : {stats['error_count']} times")

            if stats['max_temp'] >= 70:
                print("\n  ** High temperature detected - check motor load/duty cycle")
            elif stats['max_temp'] >= 50:
                print("\n  ** Moderate temperature - acceptable for continuous operation")
            else:
                print("\n  ** Temperature normal")
        print("=" * 80)

        bus.shutdown()
        print("CAN closed")


if __name__ == '__main__':
    main()
