#!/bin/bash
# 나노초 단위 타임스탬프를 이용한 정밀 회전 분석

LOG_FILE="/tmp/unified_control_debug.log"

echo "=========================================="
echo "정밀 회전 분석 (나노초 단위 타임스탬프)"
echo "=========================================="
echo ""

if [ ! -f "$LOG_FILE" ]; then
    echo "⚠️  로그 파일이 없습니다: $LOG_FILE"
    exit 1
fi

python3 << 'EOF'
import re
from datetime import datetime

log_file = "/tmp/unified_control_debug.log"

# 나노초 타임스탬프 파싱
def parse_timestamp(line):
    # 형식: [INFO] [1765261073.858927756] [iron_md_teleop]: ...
    match = re.search(r'\[(\d+)\.(\d+)\]', line)
    if match:
        seconds = int(match.group(1))
        nanoseconds = int(match.group(2))
        total_seconds = seconds + nanoseconds / 1e9
        return total_seconds
    # 기존 형식: 2025-12-09 14:02:13
    match = re.search(r'(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})', line)
    if match:
        try:
            dt = datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S")
            return dt.timestamp()
        except:
            pass
    return None

# 데이터 수집
rotations = []

with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
    for line in f:
        # 회전 명령 파싱
        match = re.search(r'주행 모터 ([+-]360°).*회전 명령.*0x141 ([\d.-]+)° → ([\d.-]+)°.*0x142 ([\d.-]+)° → ([\d.-]+)°', line)
        if match:
            timestamp = parse_timestamp(line)
            if timestamp:
                direction = match.group(1)
                left_start = float(match.group(2))
                left_target = float(match.group(3))
                right_start = float(match.group(4))
                right_target = float(match.group(5))
                
                rotations.append({
                    'time': timestamp,
                    'direction': direction,
                    'left_start': left_start,
                    'left_target': left_target,
                    'right_start': right_start,
                    'right_target': right_target,
                    'speed': None,
                    'left_near': None,
                    'right_near': None,
                    'line': line[:100]
                })
        
        # 속도 명령 파싱
        match = re.search(r'속도=([\d.]+)dps', line)
        if match:
            speed = float(match.group(1))
            timestamp = parse_timestamp(line)
            if timestamp:
                # 가장 가까운 회전 명령에 속도 할당
                for rot in rotations:
                    if rot['speed'] is None:
                        time_diff = abs(timestamp - rot['time'])
                        if time_diff < 1.0:
                            rot['speed'] = speed
                            break
        
        # 목표 근접 파싱
        match = re.search(r'\[동기화\].*(0x14[12]).*목표 근접: ([\d.-]+)°.*목표: ([\d.-]+)°', line)
        if match:
            motor_id = match.group(1)
            position = float(match.group(2))
            target = float(match.group(3))
            timestamp = parse_timestamp(line)
            
            if timestamp:
                for rot in rotations:
                    if rot['speed'] and rot['speed'] >= 650:  # 656dps 이상만
                        if motor_id == '0x141' and abs(target - rot['left_target']) < 5.0:
                            if rot['left_near'] is None or timestamp < rot['left_near']:
                                rot['left_near'] = timestamp
                        elif motor_id == '0x142' and abs(target - rot['right_target']) < 5.0:
                            if rot['right_near'] is None or timestamp < rot['right_near']:
                                rot['right_near'] = timestamp

# 656dps 이상 회전만 필터링
high_speed_rotations = [r for r in rotations if r['speed'] and r['speed'] >= 650 and r['left_near'] and r['right_near']]

if high_speed_rotations:
    print("=" * 80)
    print(f"고속 회전 분석 (656dps 이상, {len(high_speed_rotations)}회)")
    print("=" * 80)
    print("")
    
    for i, rot in enumerate(high_speed_rotations, 1):
        completion_time = max(rot['left_near'], rot['right_near'])
        duration = completion_time - rot['time']
        
        # 시간 포맷팅
        cmd_time_str = datetime.fromtimestamp(rot['time']).strftime('%H:%M:%S.%f')[:-3]
        near_time_str = datetime.fromtimestamp(completion_time).strftime('%H:%M:%S.%f')[:-3]
        
        print(f"[회차 {i}] {cmd_time_str} - {rot['direction']} 회전")
        print("-" * 80)
        print(f"설정 속도: {rot['speed']:.0f}dps")
        print(f"0x141: {rot['left_start']:.1f}° → {rot['left_target']:.1f}°")
        print(f"0x142: {rot['right_start']:.1f}° → {rot['right_target']:.1f}°")
        print(f"목표 근접: {near_time_str}")
        print(f"완료 시간: {duration:.3f}초")
        print(f"속도: {180.0 / duration:.2f} mm/sec")
        print("")
    
    # 통계
    durations = [max(r['left_near'], r['right_near']) - r['time'] for r in high_speed_rotations]
    speeds = [r['speed'] for r in high_speed_rotations]
    
    print("=" * 80)
    print("통계")
    print("=" * 80)
    print(f"총 회전 횟수: {len(high_speed_rotations)}")
    print(f"평균 시간: {sum(durations) / len(durations):.3f}초")
    print(f"최소 시간: {min(durations):.3f}초")
    print(f"최대 시간: {max(durations):.3f}초")
    print(f"평균 속도: {sum([180.0/d for d in durations]) / len(durations):.2f} mm/sec")
    print("")
    
    # 속도별 그룹화
    speed_groups = {}
    for rot in high_speed_rotations:
        speed_key = f"{rot['speed']:.0f}"
        if speed_key not in speed_groups:
            speed_groups[speed_key] = []
        duration = max(rot['left_near'], rot['right_near']) - rot['time']
        speed_groups[speed_key].append(duration)
    
    print("속도별 분석:")
    for speed_key in sorted(speed_groups.keys(), key=lambda x: float(x)):
        durations = speed_groups[speed_key]
        avg_dur = sum(durations) / len(durations)
        print(f"  {speed_key}dps: {len(durations)}회, 평균 {avg_dur:.3f}초, 속도 {180.0/avg_dur:.2f} mm/sec")
    
    print("=" * 80)
else:
    print("⚠️  고속 회전(656dps 이상) 데이터를 찾을 수 없습니다.")
    print(f"총 회전 명령 수: {len(rotations)}")

EOF



