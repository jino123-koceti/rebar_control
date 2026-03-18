#!/bin/bash
# 철근 결속 로봇 통합 제어 시스템 (부팅 자동 실행용)
# 모니터 없는 환경 - 로그 중심 실행

# 로그 디렉토리 및 파일 설정
LOG_DIR="/var/log/robot_control"
mkdir -p $LOG_DIR
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/control_${TIMESTAMP}.log"
LATEST_LOG="$LOG_DIR/control_latest.log"

# 로그 함수
log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $LOG_FILE
}

# 시작 로그
log_msg "========== 철근 결속 로봇 제어 시스템 시작 =========="
log_msg "로그 파일: $LOG_FILE"

# ROS2 환경 설정
cd /home/koceti/ros2_ws
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null
log_msg "ROS2 환경 설정 완료"

# 1. RMD 모터 제어 노드 (can2 - 7개 모터)
log_msg "[1/3] RMD 모터 제어 노드 시작 (can2)..."
ros2 run rmd_robot_control position_control_node >> $LOG_FILE 2>&1 &
MOTOR_PID=$!
log_msg "  - PID: $MOTOR_PID"
sleep 3

# 2. EZI-IO 리미트 센서 노드
log_msg "[2/4] EZI-IO 센서 노드 시작 (192.168.0.3)..."
ros2 run ezi_io_ros2 ezi_io_node --ros-args -p ip_address:=192.168.0.3 >> $LOG_FILE 2>&1 &
EZIIO_PID=$!
log_msg "  - PID: $EZIIO_PID"
sleep 2

# 3. Seengrip 그리퍼 노드 (Modbus RTU)
log_msg "[3/4] Seengrip 그리퍼 노드 시작 (/dev/ttyUSB0)..."
ros2 run seengrip_ros2 seengrip_node --ros-args -p serial_port:=/dev/ttyUSB0 >> $LOG_FILE 2>&1 &
GRIPPER_PID=$!
log_msg "  - PID: $GRIPPER_PID"
sleep 2

# 4. Iron-MD 텔레옵 노드 (can3 - 무선 리모콘)
log_msg "[4/4] Iron-MD 텔레옵 노드 시작 (can3)..."
ros2 run rebar_control iron_md_teleop >> $LOG_FILE 2>&1 &
TELEOP_PID=$!
log_msg "  - PID: $TELEOP_PID"
sleep 2

# 시작 완료
log_msg "========== 시스템 시작 완료 =========="
log_msg "모터 제어: PID $MOTOR_PID"
log_msg "EZI-IO 센서: PID $EZIIO_PID"
log_msg "그리퍼: PID $GRIPPER_PID"
log_msg "텔레옵: PID $TELEOP_PID"
log_msg "로그 확인: tail -f $LOG_FILE"

# 최신 로그 심볼릭 링크 생성
ln -sf $LOG_FILE $LATEST_LOG

# 프로세스 모니터링 (5분마다 상태 확인)
monitor_processes() {
    while true; do
        sleep 300  # 5분
        
        # 프로세스 상태 확인
        if ! kill -0 $MOTOR_PID 2>/dev/null; then
            log_msg "⚠️ ERROR: 모터 제어 노드 종료됨 (PID $MOTOR_PID)"
        fi
        if ! kill -0 $EZIIO_PID 2>/dev/null; then
            log_msg "⚠️ ERROR: EZI-IO 노드 종료됨 (PID $EZIIO_PID)"
        fi
        if ! kill -0 $GRIPPER_PID 2>/dev/null; then
            log_msg "⚠️ ERROR: 그리퍼 노드 종료됨 (PID $GRIPPER_PID)"
        fi
        if ! kill -0 $TELEOP_PID 2>/dev/null; then
            log_msg "⚠️ ERROR: 텔레옵 노드 종료됨 (PID $TELEOP_PID)"
        fi
    done
}

# 백그라운드 모니터링 시작
monitor_processes &
MONITOR_PID=$!

# 종료 트랩 (시스템 종료 시)
cleanup() {
    log_msg "========== 시스템 종료 중 =========="
    kill $MONITOR_PID 2>/dev/null
    kill $TELEOP_PID 2>/dev/null
    kill $GRIPPER_PID 2>/dev/null
    kill $EZIIO_PID 2>/dev/null
    kill $MOTOR_PID 2>/dev/null
    sleep 1
    log_msg "모든 노드 종료 완료"
    log_msg "========== 시스템 종료 완료 =========="
}

trap cleanup SIGINT SIGTERM EXIT

# 계속 실행 (데몬 모드)
wait
