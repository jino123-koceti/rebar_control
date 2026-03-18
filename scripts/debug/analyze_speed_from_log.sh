#!/bin/bash
# 로그에서 속도 상세 분석 스크립트

LOG_FILE="/tmp/unified_control_debug.log"

echo "=========================================="
echo "로그 기반 속도 상세 분석"
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
speed_commands = []  # (timestamp, motor_id, speed_dps)

# 로그 파일 읽기
with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        # 회전 명령 파싱 (목표 위치와 속도 포함)
        # 예: [S20/AN3] 🎯 주행 모터 +360° 회전 명령 (1회): 0x141 -142.6° → -502.6° (변화: -360.0°), 0x142 140.6° → 500.6° (변화: 360.0°)
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
        
        # 속도 명령 파싱
        # 예: [S20/AN3] 📤 0x141 위치 제어 명령 발행: 목표=-502.6°, 속도=200.0dps
        match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}).*\[S20/AN3\].*📤.*0x14[12].*위치 제어 명령 발행.*속도=([\d.]+)dps', line)
        if match:
            timestamp_str = match.group(1)
            speed_dps = float(match.group(2))
            try:
                timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                speed_commands.append((timestamp, speed_dps))
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
print(f"속도 명령 수: {len(speed_commands)}")
print(f"0x141 목표 도달 수: {len(target_reached['0x141'])}")
print(f"0x142 목표 도달 수: {len(target_reached['0x142'])}")
print("")

# 회전 명령에 속도 정보 매칭
for i, (cmd_time, direction, left_target, right_target, _) in enumerate(rotation_commands):
    # 해당 명령 시간 근처의 속도 명령 찾기
    for speed_time, speed_dps in speed_commands:
        time_diff = abs((speed_time - cmd_time).total_seconds())
        if time_diff < 1.0:  # 1초 이내
            rotation_commands[i] = (cmd_time, direction, left_target, right_target, speed_dps)
            break

# 각 회전 명령에 대해 완료 시간 찾기
rotation_times = []

for cmd_time, direction, left_target, right_target, speed_dps in rotation_commands:
    left_reached = None
    right_reached = None
    
    # 0x141 목표 도달 찾기
    for reached_time, pos, target in target_reached['0x141']:
        if reached_time > cmd_time:
            if abs(target - left_target) < 1.0:
                if left_reached is None or reached_time < left_reached[0]:
                    left_reached = (reached_time, pos, target)
                break
    
    # 0x142 목표 도달 찾기
    for reached_time, pos, target in target_reached['0x142']:
        if reached_time > cmd_time:
            if abs(target - right_target) < 1.0:
                if right_reached is None or reached_time < right_reached[0]:
                    right_reached = (reached_time, pos, target)
                break
    
    if left_reached and right_reached:
        completion_time = max(left_reached[0], right_reached[0])
        duration = (completion_time - cmd_time).total_seconds()
        rotation_times.append((cmd_time, direction, duration, speed_dps, left_reached[0], right_reached[0]))

# 결과 출력
if rotation_times:
    print("=" * 70)
    print("360도 회전 상세 분석 결과")
    print("=" * 70)
    print("")
    
    # 속도별 그룹화
    speed_groups = defaultdict(list)
    for cmd_time, direction, duration, speed_dps, left_time, right_time in rotation_times:
        speed_key = f"{speed_dps:.0f}" if speed_dps else "Unknown"
        speed_groups[speed_key].append((duration, speed_dps))
    
    total_time = 0
    speeds_found = []
    
    for i, (cmd_time, direction, duration, speed_dps, left_time, right_time) in enumerate(rotation_times, 1):
        speed_str = f"{speed_dps:.0f}dps" if speed_dps else "Unknown"
        print(f"[{i}] {cmd_time.strftime('%H:%M:%S')} - {direction} 회전")
        print(f"    설정 속도: {speed_str}")
        print(f"    완료 시간: {duration:.3f}초")
        if speed_dps:
            speeds_found.append(speed_dps)
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
    if speeds_found:
        avg_speed = sum(speeds_found) / len(speeds_found)
        print(f"  평균 설정 속도: {avg_speed:.1f} dps")
    print("=" * 70)
    print("")
    
    # 속도별 분석
    print("=" * 70)
    print("속도별 분석")
    print("=" * 70)
    for speed_key in sorted(speed_groups.keys(), key=lambda x: float(x) if x != "Unknown" else 0):
        durations = [d[0] for d in speed_groups[speed_key]]
        if durations:
            avg_dur = sum(durations) / len(durations)
            print(f"  {speed_key}dps: {len(durations)}회, 평균 시간: {avg_dur:.3f}초")
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
    if speeds_found:
        avg_speed_dps = sum(speeds_found) / len(speeds_found)
        print(f"설정 속도와 실제 속도 비교:")
        print(f"  설정 속도: {avg_speed_dps:.1f} dps")
        print(f"  실제 평균 속도: {360.0 / avg_time:.1f} dps (360도 / {avg_time:.3f}초)")
        print(f"  효율: {(360.0 / avg_time / avg_speed_dps * 100):.1f}%")
    print("=" * 70)
else:
    print("⚠️  완료된 회전 명령을 찾을 수 없습니다.")

EOF

