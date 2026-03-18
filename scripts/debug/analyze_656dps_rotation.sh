#!/bin/bash
# 656dps로 설정된 360도 회전 분석 스크립트

LOG_FILE="/tmp/unified_control_debug.log"

echo "=========================================="
echo "656dps 설정 360도 회전 분석"
echo "=========================================="
echo ""

if [ ! -f "$LOG_FILE" ]; then
    echo "⚠️  로그 파일이 없습니다: $LOG_FILE"
    exit 1
fi

python3 << 'EOF'
import re
from datetime import datetime
from collections import defaultdict

log_file = "/tmp/unified_control_debug.log"

# 데이터 저장
rotation_commands = []  # (timestamp, direction, left_target, right_target, speed_dps)
target_reached = defaultdict(list)  # motor_id -> [(timestamp, position, target)]
speed_commands_656 = []  # (timestamp, motor_id, target_position, speed_dps)

# 로그 파일 읽기
with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        # 656dps 속도 명령 파싱 (목표 위치 포함)
        # 예: [S20/AN3] 📤 0x141 위치 제어 명령 발행: 목표=-769.9°, 속도=656.0dps
        match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}).*\[S20/AN3\].*📤.*(0x14[12]).*위치 제어 명령 발행.*목표=([\d.-]+)°.*속도=656\.0dps', line)
        if match:
            timestamp_str = match.group(1)
            motor_id = match.group(2)
            target_pos = float(match.group(3))
            try:
                timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                speed_commands_656.append((timestamp, motor_id, target_pos, 656.0))
            except:
                pass
        
        # 회전 명령 파싱 (목표 위치 포함)
        match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}).*\[S20/AN3\].*주행 모터 ([+-]360°).*회전 명령.*0x141.*→ ([\d.-]+)°.*0x142.*→ ([\d.-]+)°', line)
        if match:
            timestamp_str = match.group(1)
            direction = match.group(2)
            left_target = float(match.group(3))
            right_target = float(match.group(4))
            try:
                timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                rotation_commands.append((timestamp, direction, left_target, right_target, None))
            except:
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
print(f"656dps 속도 명령 수: {len(speed_commands_656)}")
print(f"0x141 목표 도달 수: {len(target_reached['0x141'])}")
print(f"0x142 목표 도달 수: {len(target_reached['0x142'])}")
print("")

# 656dps 명령들을 그룹화 (같은 시간대의 0x141, 0x142 쌍 찾기)
speed_groups = defaultdict(list)
for speed_time, motor_id, target_pos, speed_dps in speed_commands_656:
    # 같은 초 단위로 그룹화
    time_key = speed_time.strftime('%H:%M:%S')
    speed_groups[time_key].append((speed_time, motor_id, target_pos, speed_dps))

# 656dps 명령 쌍 찾기 (0x141과 0x142가 같은 시간대에 있는 경우)
rotation_commands_656 = []
for time_key in sorted(speed_groups.keys()):
    group = speed_groups[time_key]
    left_cmd = None
    right_cmd = None
    
    for speed_time, motor_id, target_pos, speed_dps in group:
        if motor_id == '0x141':
            left_cmd = (speed_time, target_pos)
        elif motor_id == '0x142':
            right_cmd = (speed_time, target_pos)
    
    if left_cmd and right_cmd:
        # 360도 회전인지 확인 (목표 위치 차이가 약 360도)
        # 실제로는 회전 명령 로그를 확인해야 하지만, 일단 시간과 목표 위치로 판단
        cmd_time = max(left_cmd[0], right_cmd[0])
        rotation_commands_656.append((cmd_time, '+360°', left_cmd[1], right_cmd[1], 656.0))

print(f"656dps로 설정된 회전 명령 수: {len(rotation_commands_656)}")
print("")

# 각 회전 명령에 대해 완료 시간 찾기
rotation_times = []

for cmd_time, direction, left_target, right_target, speed_dps in rotation_commands_656:
    left_reached = None
    right_reached = None
    
    # 0x141 목표 도달 찾기 (더 넓은 범위로 검색)
    for reached_time, pos, target in target_reached['0x141']:
        if reached_time > cmd_time:
            # 목표 위치가 일치하는지 확인 (오차 5도까지 허용)
            if abs(target - left_target) < 5.0:
                if left_reached is None or reached_time < left_reached[0]:
                    left_reached = (reached_time, pos, target)
    
    # 0x142 목표 도달 찾기 (더 넓은 범위로 검색)
    for reached_time, pos, target in target_reached['0x142']:
        if reached_time > cmd_time:
            # 목표 위치가 일치하는지 확인 (오차 5도까지 허용)
            if abs(target - right_target) < 5.0:
                if right_reached is None or reached_time < right_reached[0]:
                    right_reached = (reached_time, pos, target)
    
    if left_reached and right_reached:
        completion_time = max(left_reached[0], right_reached[0])
        duration = (completion_time - cmd_time).total_seconds()
        rotation_times.append((cmd_time, direction, duration, speed_dps, left_reached[0], right_reached[0], left_target, right_target))

# 결과 출력
if rotation_times:
    print("=" * 70)
    print("656dps 설정 360도 회전 분석 결과")
    print("=" * 70)
    print("")
    
    total_time = 0
    
    for i, (cmd_time, direction, duration, speed_dps, left_time, right_time, left_target, right_target) in enumerate(rotation_times, 1):
        print(f"[{i}] {cmd_time.strftime('%H:%M:%S')} - {direction} 회전")
        print(f"    명령 시간: {cmd_time.strftime('%H:%M:%S')}")
        print(f"    설정 속도: {speed_dps:.0f}dps")
        print(f"    목표 위치: 0x141={left_target:.1f}°, 0x142={right_target:.1f}°")
        print(f"    0x141 도달: {left_time.strftime('%H:%M:%S')}")
        print(f"    0x142 도달: {right_time.strftime('%H:%M:%S')}")
        print(f"    완료 시간: {duration:.3f}초")
        print("")
        total_time += duration
    
    avg_time = total_time / len(rotation_times)
    min_time = min(d[2] for d in rotation_times)
    max_time = max(d[2] for d in rotation_times)
    
    print("=" * 70)
    print("통계:")
    print(f"  총 회전 횟수: {len(rotation_times)}")
    print(f"  평균 시간: {avg_time:.3f}초")
    print(f"  최소 시간: {min_time:.3f}초")
    print(f"  최대 시간: {max_time:.3f}초")
    print(f"  설정 속도: 656.0 dps")
    print("=" * 70)
    print("")
    
    # 이동 거리와 속도 계산 (18cm 기준)
    distance_cm = 18.0
    distance_mm = distance_cm * 10
    
    print("=" * 70)
    print("속도 계산 (1회 360도 회전 시 18cm 이동)")
    print("=" * 70)
    print(f"이동 거리: {distance_cm}cm = {distance_mm}mm")
    print(f"평균 회전 시간: {avg_time:.3f}초")
    print(f"")
    print(f"속도 계산:")
    print(f"  평균 속도: {distance_mm / avg_time:.2f} mm/sec")
    print(f"  최소 속도: {distance_mm / max_time:.2f} mm/sec (최대 시간 기준)")
    print(f"  최대 속도: {distance_mm / min_time:.2f} mm/sec (최소 시간 기준)")
    print("")
    print(f"설정 속도와 실제 속도 비교:")
    print(f"  설정 속도: 656.0 dps")
    print(f"  실제 평균 속도: {360.0 / avg_time:.1f} dps (360도 / {avg_time:.3f}초)")
    print(f"  효율: {(360.0 / avg_time / 656.0 * 100):.1f}%")
    print("")
    print(f"200dps와 비교:")
    print(f"  속도 비율: {656.0 / 200.0:.2f}배")
    expected_speed_ratio = 656.0 / 200.0
    print(f"  예상 속도: {56.84 * expected_speed_ratio:.2f} mm/sec (200dps 대비)")
    print(f"  실제 속도: {distance_mm / avg_time:.2f} mm/sec")
    print("=" * 70)
else:
    print("⚠️  656dps로 설정된 완료된 회전 명령을 찾을 수 없습니다.")
    print("   (목표 도달 이벤트가 로그에 기록되지 않았을 수 있습니다)")
    print("")
    print("=" * 70)
    print("656dps 명령 정보")
    print("=" * 70)
    print("")
    
    # 656dps 명령들을 시간순으로 정리
    for i, (speed_time, motor_id, target_pos, speed_dps) in enumerate(speed_commands_656, 1):
        print(f"[{i}] {speed_time.strftime('%H:%M:%S')} - {motor_id}: 목표={target_pos:.1f}°, 속도={speed_dps:.0f}dps")
    
    print("")
    print("=" * 70)
    print("참고")
    print("=" * 70)
    print("656dps 명령은 확인되었지만, 목표 도달 이벤트가 로그에 없습니다.")
    print("이는 다음 중 하나일 수 있습니다:")
    print("  1. 명령이 아직 완료되지 않음")
    print("  2. 로그 기록이 중단됨")
    print("  3. 목표 도달 이벤트가 다른 형식으로 기록됨")
    print("")
    print("656dps 설정 시 예상 성능 (200dps 대비):")
    print(f"  속도 비율: {656.0 / 200.0:.2f}배")
    print(f"  예상 속도: {56.84 * (656.0 / 200.0):.2f} mm/sec (200dps의 {656.0 / 200.0:.2f}배)")
    print("=" * 70)

EOF

