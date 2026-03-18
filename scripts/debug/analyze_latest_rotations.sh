#!/bin/bash
# 최신 로그 기반 회전 분석 (목표 근접 이벤트 포함)

LOG_FILE="/tmp/unified_control_debug.log"

echo "=========================================="
echo "최신 로그 기반 회전 분석 (목표 근접 포함)"
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
target_near = defaultdict(list)  # motor_id -> [(timestamp, position, target)]

# 로그 파일 읽기
with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        # 회전 명령 파싱
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
        
        # 속도 명령 파싱 (회전 명령과 매칭)
        match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}).*\[S20/AN3\].*📤.*0x14[12].*위치 제어 명령 발행.*목표=([\d.-]+)°.*속도=([\d.]+)dps', line)
        if match:
            timestamp_str = match.group(1)
            target_pos = float(match.group(2))
            speed_dps = float(match.group(3))
            try:
                timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                # 회전 명령과 매칭
                for i, (cmd_time, direction, left_target, right_target, _) in enumerate(rotation_commands):
                    time_diff = abs((timestamp - cmd_time).total_seconds())
                    if time_diff < 1.0:  # 1초 이내
                        if abs(target_pos - left_target) < 1.0 or abs(target_pos - right_target) < 1.0:
                            rotation_commands[i] = (cmd_time, direction, left_target, right_target, speed_dps)
                            break
            except:
                pass
        
        # 목표 근접 파싱
        match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}).*\[동기화\].*(0x14[12]).*목표 근접: ([\d.-]+)°.*목표: ([\d.-]+)°', line)
        if match:
            timestamp_str = match.group(1)
            motor_id = match.group(2)
            position = float(match.group(3))
            target = float(match.group(4))
            try:
                timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                target_near[motor_id].append((timestamp, position, target))
            except:
                pass

print(f"총 회전 명령 수: {len(rotation_commands)}")
print(f"0x141 목표 근접 수: {len(target_near['0x141'])}")
print(f"0x142 목표 근접 수: {len(target_near['0x142'])}")
print("")

# 각 회전 명령에 대해 완료 시간 찾기
rotation_times = []

for cmd_time, direction, left_target, right_target, speed_dps in rotation_commands:
    left_near = None
    right_near = None
    
    # 0x141 목표 근접 찾기
    for near_time, pos, target in target_near['0x141']:
        if near_time > cmd_time:
            if abs(target - left_target) < 5.0:  # 5도 오차 허용
                if left_near is None or near_time < left_near[0]:
                    left_near = (near_time, pos, target)
    
    # 0x142 목표 근접 찾기
    for near_time, pos, target in target_near['0x142']:
        if near_time > cmd_time:
            if abs(target - right_target) < 5.0:  # 5도 오차 허용
                if right_near is None or near_time < right_near[0]:
                    right_near = (near_time, pos, target)
    
    if left_near and right_near:
        completion_time = max(left_near[0], right_near[0])
        duration = (completion_time - cmd_time).total_seconds()
        rotation_times.append((cmd_time, direction, duration, speed_dps, left_near[0], right_near[0], left_target, right_target))

# 속도별 그룹화
speed_groups = defaultdict(list)
for cmd_time, direction, duration, speed_dps, left_time, right_time, left_target, right_target in rotation_times:
    speed_key = f"{speed_dps:.0f}" if speed_dps else "Unknown"
    speed_groups[speed_key].append((duration, speed_dps))

# 결과 출력
if rotation_times:
    print("=" * 70)
    print("회전 분석 결과 (목표 근접 기준)")
    print("=" * 70)
    print("")
    
    total_time = 0
    speeds_found = []
    
    for i, (cmd_time, direction, duration, speed_dps, left_time, right_time, left_target, right_target) in enumerate(rotation_times, 1):
        speed_str = f"{speed_dps:.0f}dps" if speed_dps else "Unknown"
        print(f"[{i}] {cmd_time.strftime('%H:%M:%S')} - {direction} 회전")
        print(f"    설정 속도: {speed_str}")
        print(f"    목표 위치: 0x141={left_target:.1f}°, 0x142={right_target:.1f}°")
        print(f"    0x141 근접: {left_time.strftime('%H:%M:%S')}")
        print(f"    0x142 근접: {right_time.strftime('%H:%M:%S')}")
        print(f"    완료 시간: {duration:.3f}초")
        print("")
        total_time += duration
        if speed_dps:
            speeds_found.append(speed_dps)
    
    avg_time = total_time / len(rotation_times)
    min_time = min(d[2] for d in rotation_times)
    max_time = max(d[2] for d in rotation_times)
    
    print("=" * 70)
    print("전체 통계:")
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
            min_dur = min(durations)
            max_dur = max(durations)
            print(f"  {speed_key}dps: {len(durations)}회")
            print(f"    평균 시간: {avg_dur:.3f}초")
            print(f"    최소 시간: {min_dur:.3f}초")
            print(f"    최대 시간: {max_dur:.3f}초")
            print("")
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
    print(f"전체 평균 속도: {distance_mm / avg_time:.2f} mm/sec")
    print("")
    
    # 속도별 속도 계산
    for speed_key in sorted(speed_groups.keys(), key=lambda x: float(x) if x != "Unknown" else 0):
        durations = [d[0] for d in speed_groups[speed_key]]
        if durations:
            avg_dur = sum(durations) / len(durations)
            speed_mm_per_sec = distance_mm / avg_dur
            print(f"{speed_key}dps: {speed_mm_per_sec:.2f} mm/sec (평균 {avg_dur:.3f}초)")
    print("=" * 70)
else:
    print("⚠️  완료된 회전 명령을 찾을 수 없습니다.")

EOF



