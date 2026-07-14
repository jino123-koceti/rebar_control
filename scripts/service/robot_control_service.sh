#!/bin/bash
# 철근 결속 로봇 통합 제어 시스템 (Full System)
# HAL(base_system) + 상위 제어(control_system) + ZED 카메라 전체 실행

# 로그 디렉토리 설정
LOG_DIR="/var/log/robot_control"
mkdir -p $LOG_DIR 2>/dev/null || true
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/control_${TIMESTAMP}.log"

log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $LOG_FILE
}

log_msg "========== 철근 결속 로봇 전체 시스템 시작 (Full System) =========="

# ROS2 환경 설정
cd /home/koceti/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

log_msg "ROS2 환경 설정 완료"

# Orbbec Gemini 2L - 별도 프로세스로 실행 (ZED와 camera_container 이름충돌 회피)
# 같은 launch에 넣으면 orbbec 노드가 ZED 컨테이너로 로드돼 스트림 안 됨 → 독립 프로세스
log_msg "Orbbec 카메라 별도 실행 (background)"
ros2 launch orbbec_camera gemini2L.launch.py >> $LOG_FILE 2>&1 &
ORBBEC_PID=$!
log_msg "Orbbec PID=$ORBBEC_PID (서비스 종료 시 cgroup으로 함께 종료됨)"
sleep 3

log_msg "실행: ros2 launch rebar_control full_system.launch.py"
# 전체 시스템 launch 실행 (HAL + 상위 제어 + ZED)
ros2 launch rebar_control full_system.launch.py 2>&1 | tee -a $LOG_FILE
