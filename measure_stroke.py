#!/usr/bin/env python3
"""
결속부 스테이지 스트로크 측정 스크립트

사용법:
  1. 축을 min 리미트에 위치시킨 후 Enter
  2. 축을 max 리미트에 위치시킨 후 Enter
  3. 스트로크(도, mm) 자동 계산

모터 매핑:
  0x144: X축 (전후)
  0x145: Y축 (횡방향)
  0x146: Z축 (상하)
  0x147: Yaw (회전)
"""

import socket
import struct
import time
import sys

# ============================================
# 설정
# ============================================
CAN_INTERFACE = 'can2'

MOTORS = {
    '0x144': {'id': 0x144, 'name': 'X축 (전후)', 'type': 'linear', 'deg_per_mm': None},
    '0x145': {'id': 0x145, 'name': 'Y축 (횡방향)', 'type': 'linear', 'deg_per_mm': None},
    '0x146': {'id': 0x146, 'name': 'Z축 (상하)', 'type': 'linear', 'deg_per_mm': None},
    '0x147': {'id': 0x147, 'name': 'Yaw (회전)', 'type': 'rotary', 'deg_per_mm': None},
}


def can_connect(interface):
    """SocketCAN 연결"""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    sock.settimeout(0.5)
    return sock


def read_multi_turn_angle(sock, motor_id):
    """0x92 명령으로 멀티턴 각도 읽기 (0.01도 단위)"""
    # 버퍼 비우기 (최대 50개만)
    sock.settimeout(0.01)
    for _ in range(50):
        try:
            sock.recv(16)
        except socket.timeout:
            break
    sock.settimeout(0.5)

    # 0x92 명령 전송
    cmd = bytes([0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    frame = struct.pack("IB3x8s", motor_id, 8, cmd)
    sock.send(frame)

    # 응답 대기
    response_id = motor_id + 0x100  # 0x244, 0x245, ...
    start = time.time()
    while time.time() - start < 1.0:
        try:
            resp = sock.recv(16)
            can_id = struct.unpack("I", resp[:4])[0] & 0x1FFFFFFF
            data = resp[8:16]

            if can_id == response_id and data[0] == 0x92:
                angle_raw = struct.unpack('<i', data[4:8])[0]  # 0.01도 단위
                angle_deg = angle_raw * 0.01
                return angle_deg
        except socket.timeout:
            continue

    return None


def read_angle_averaged(sock, motor_id, samples=5):
    """여러 번 읽어서 평균값 반환"""
    readings = []
    for _ in range(samples):
        angle = read_multi_turn_angle(sock, motor_id)
        if angle is not None:
            readings.append(angle)
        time.sleep(0.05)

    if not readings:
        return None
    return sum(readings) / len(readings)


def measure_single_axis(sock, motor_info):
    """단일 축 스트로크 측정"""
    motor_id = motor_info['id']
    name = motor_info['name']
    is_linear = motor_info['type'] == 'linear'

    print(f"\n{'='*50}")
    print(f"  {name} (0x{motor_id:03X}) 스트로크 측정")
    print(f"{'='*50}")

    # 현재 각도 확인
    current = read_multi_turn_angle(sock, motor_id)
    if current is None:
        print(f"  [오류] 모터 0x{motor_id:03X} 응답 없음!")
        return None

    print(f"  현재 각도: {current:.2f}°")

    # MIN 리미트 측정
    print(f"\n  >>> 축을 MIN 리미트로 이동시킨 후 Enter를 누르세요...")
    input()
    angle_min = read_angle_averaged(sock, motor_id)
    if angle_min is None:
        print(f"  [오류] 각도 읽기 실패")
        return None
    print(f"  MIN 위치 각도: {angle_min:.2f}°")

    # MAX 리미트 측정
    print(f"\n  >>> 축을 MAX 리미트로 이동시킨 후 Enter를 누르세요...")
    input()
    angle_max = read_angle_averaged(sock, motor_id)
    if angle_max is None:
        print(f"  [오류] 각도 읽기 실패")
        return None
    print(f"  MAX 위치 각도: {angle_max:.2f}°")

    # 스트로크 계산
    stroke_deg = abs(angle_max - angle_min)

    stroke_mm = None
    deg_per_mm = None

    if is_linear:
        print(f"\n  >>> 실제 이동 거리를 mm 단위로 입력하세요 (레이저 측정값):")
        try:
            actual_mm = float(input("  실측 거리(mm)> ").strip())
            if actual_mm > 0:
                stroke_mm = actual_mm
                deg_per_mm = stroke_deg / actual_mm
        except ValueError:
            print("  [경고] 숫자가 아닙니다. mm 환산 건너뜀")

    print(f"\n  {'─'*40}")
    print(f"  결과:")
    print(f"    MIN 각도: {angle_min:.2f}°")
    print(f"    MAX 각도: {angle_max:.2f}°")
    print(f"    스트로크: {stroke_deg:.2f}°")
    if stroke_mm is not None:
        print(f"    실측 거리: {stroke_mm:.1f} mm")
        print(f"    변환비: {deg_per_mm:.3f} deg/mm")
    print(f"  {'─'*40}")

    return {
        'name': name,
        'motor_id': motor_id,
        'angle_min': angle_min,
        'angle_max': angle_max,
        'stroke_deg': stroke_deg,
        'stroke_mm': stroke_mm,
        'deg_per_mm': deg_per_mm,
        'type': motor_info['type'],
    }


def main():
    print("\n" + "="*50)
    print("  결속부 스테이지 스트로크 측정")
    print("="*50)

    # CAN 연결
    try:
        sock = can_connect(CAN_INTERFACE)
        print(f"  CAN 인터페이스 연결: {CAN_INTERFACE}")
    except Exception as e:
        print(f"  [오류] CAN 연결 실패: {e}")
        sys.exit(1)

    # 측정할 축 선택
    print("\n  측정할 축을 선택하세요:")
    print("    1: X축 (0x144)")
    print("    2: Y축 (0x145)")
    print("    3: Z축 (0x146)")
    print("    4: Yaw (0x147)")
    print("    a: 전체 측정")
    print("    q: 종료")

    results = []

    while True:
        choice = input("\n  선택> ").strip().lower()

        if choice == 'q':
            break
        elif choice == 'a':
            for key in ['0x144', '0x145', '0x146', '0x147']:
                result = measure_single_axis(sock, MOTORS[key])
                if result:
                    results.append(result)
            break
        elif choice in ('1', '2', '3', '4'):
            keys = {'1': '0x144', '2': '0x145', '3': '0x146', '4': '0x147'}
            result = measure_single_axis(sock, MOTORS[keys[choice]])
            if result:
                results.append(result)
            continue
        else:
            print("  잘못된 입력입니다.")
            continue

    # 최종 결과 요약
    if results:
        print(f"\n\n{'='*60}")
        print(f"  스트로크 측정 결과 요약")
        print(f"{'='*60}")
        print(f"  {'축':<15} {'MIN(°)':<12} {'MAX(°)':<12} {'스트로크(°)':<14} {'mm':<10} {'deg/mm':<10}")
        print(f"  {'─'*65}")

        for r in results:
            mm_str = f"{r['stroke_mm']:.1f}" if r['stroke_mm'] is not None else "N/A"
            dpm_str = f"{r['deg_per_mm']:.3f}" if r.get('deg_per_mm') is not None else "N/A"
            print(f"  {r['name']:<15} {r['angle_min']:<12.2f} {r['angle_max']:<12.2f} {r['stroke_deg']:<14.2f} {mm_str:<10} {dpm_str:<10}")

        print(f"  {'─'*65}")

        # tying_orchestrator.yaml 업데이트 값 제안
        print(f"\n  tying_orchestrator.yaml 업데이트 값:")
        for r in results:
            if r['type'] == 'linear' and r['stroke_mm'] is not None:
                axis = r['name'].split('(')[0].strip().lower()
                print(f"    max_stage_{axis}_mm: {r['stroke_mm']:.1f}")
            if r.get('deg_per_mm') is not None:
                axis = r['name'].split('(')[0].strip().lower()
                print(f"    deg_per_mm_{axis}: {r['deg_per_mm']:.3f}")

    sock.close()
    print(f"\n  측정 완료. CAN 연결 종료.\n")


if __name__ == '__main__':
    main()
