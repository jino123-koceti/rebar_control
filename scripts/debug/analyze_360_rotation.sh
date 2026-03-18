#!/bin/bash
# 360도 회전 시간 분석 스크립트

LOG_FILE="/tmp/unified_control_debug.log"

echo "=========================================="
echo "360도 회전 시간 분석"
echo "=========================================="
echo ""

if [ ! -f "$LOG_FILE" ]; then
    echo "⚠️  로그 파일이 없습니다: $LOG_FILE"
    exit 1
fi

# Python 스크립트로 분석 수행
python3 << 'EOF'
import re
from datetime import datetime
from collections import defaultdict

log_file = "/tmp/unified_control_debug.log"

# 로그 파싱
rotation_commands = []  # (timestamp, direction, left_target, right_target)
target_reached = defaultdict(list)  # motor_id -> [(timestamp, position, target)]

# 로그 파일 읽기
with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        # 회전 명령 파싱 (목표 위치 포함)
        # 예: [S20/AN3] 🎯 주행 모터 +360° 회전 명령 (1회): 0x141 -142.6° → -502.6° (변화: -360.0°), 0x142 140.6° → 500.6° (변화: 360.0°)
        match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}).*\[S20/AN3\].*주행 모터 ([+-]360°).*회전 명령.*0x141.*→ ([\d.-]+)°.*0x142.*→ ([\d.-]+)°', line)
        if match:
            timestamp_str = match.group(1)
            direction = match.group(2)
            left_target = float(match.group(3))
            right_target = float(match.group(4))
            try:
                timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                rotation_commands.append((timestamp, direction, left_target, right_target))
            except Exception as e:
                pass
        
        # 목표 도달 파싱
        match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}).*모터 (0x14[12]).*목표 도달: ([\d.-]+)°.*목표: ([\d.-]+)°', line)
        if match:
            timestamp_str = match.group(1)
            motor_id = match.group(2)
            position = float(match.group(3))
            target = float(match.group(4))
            try:
                timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                target_reached[motor_id].append((timestamp, position, target))
            except:
                pass

print(f"총 회전 명령 수: {len(rotation_commands)}")
print(f"0x141 목표 도달 수: {len(target_reached['0x141'])}")
print(f"0x142 목표 도달 수: {len(target_reached['0x142'])}")
print("")

# 각 회전 명령에 대해 완료 시간 찾기
rotation_times = []

for cmd_time, direction, left_target, right_target in rotation_commands:
    # 해당 명령 이후의 목표 도달 이벤트 찾기
    # 두 모터 모두 목표 도달해야 함
    left_reached = None
    right_reached = None
    
    # 0x141 목표 도달 찾기
    for reached_time, pos, target in target_reached['0x141']:
        if reached_time > cmd_time:
            # 목표 위치가 일치하는지 확인 (오차 1도 이내)
            if abs(target - left_target) < 1.0:
                if left_reached is None or reached_time < left_reached[0]:
                    left_reached = (reached_time, pos, target)
                break
    
    # 0x142 목표 도달 찾기
    for reached_time, pos, target in target_reached['0x142']:
        if reached_time > cmd_time:
            # 목표 위치가 일치하는지 확인 (오차 1도 이내)
            if abs(target - right_target) < 1.0:
                if right_reached is None or reached_time < right_reached[0]:
                    right_reached = (reached_time, pos, target)
                break
    
    if left_reached and right_reached:
        # 두 모터 중 더 늦게 도달한 시간 사용
        completion_time = max(left_reached[0], right_reached[0])
        duration = (completion_time - cmd_time).total_seconds()
        rotation_times.append((cmd_time, direction, duration, left_reached[0], right_reached[0], left_target, right_target))

# 결과 출력
if rotation_times:
    print("=" * 60)
    print("360도 회전 시간 분석 결과")
    print("=" * 60)
    print("")
    
    total_time = 0
    for i, (cmd_time, direction, duration, left_time, right_time, left_target, right_target) in enumerate(rotation_times, 1):
        print(f"[{i}] {cmd_time.strftime('%H:%M:%S')} - {direction} 회전")
        print(f"    명령 시간: {cmd_time.strftime('%H:%M:%S')}")
        print(f"    목표 위치: 0x141={left_target:.1f}°, 0x142={right_target:.1f}°")
        print(f"    0x141 도달: {left_time.strftime('%H:%M:%S')} ({duration:.3f}초 후)")
        print(f"    0x142 도달: {right_time.strftime('%H:%M:%S')}")
        print(f"    완료 시간: {duration:.3f}초")
        print("")
        total_time += duration
    
    avg_time = total_time / len(rotation_times)
    min_time = min(d[2] for d in rotation_times)
    max_time = max(d[2] for d in rotation_times)
    
    print("=" * 60)
    print("통계:")
    print(f"  총 회전 횟수: {len(rotation_times)}")
    print(f"  평균 시간: {avg_time:.3f}초")
    print(f"  최소 시간: {min_time:.3f}초")
    print(f"  최대 시간: {max_time:.3f}초")
    print("=" * 60)
    print("")
    
    # 18cm를 mm/sec로 변환
    distance_cm = 18.0
    distance_mm = distance_cm * 10  # 180mm
    
    print("=" * 60)
    print("속도 계산 (1회 360도 회전 시 18cm 이동)")
    print("=" * 60)
    print(f"이동 거리: {distance_cm}cm = {distance_mm}mm")
    print(f"평균 회전 시간: {avg_time:.3f}초")
    print(f"")
    print(f"속도 계산:")
    print(f"  평균 속도: {distance_mm / avg_time:.2f} mm/sec")
    print(f"  최소 속도: {distance_mm / max_time:.2f} mm/sec (최대 시간 기준)")
    print(f"  최대 속도: {distance_mm / min_time:.2f} mm/sec (최소 시간 기준)")
    print("=" * 60)
else:
    print("⚠️  완료된 회전 명령을 찾을 수 없습니다.")
    print("   회전 명령과 목표 도달 로그를 확인하세요.")

EOF

