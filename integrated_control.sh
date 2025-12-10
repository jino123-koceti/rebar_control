#!/bin/bash
# 철근 결속 로봇 통합 제어: Iron-MD(can3) + RMD Motors(can2)

echo "=========================================="
echo "통합 제어 시스템 시작"
echo "=========================================="
echo ""

cd /home/koceti/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "CAN 인터페이스 상태:"
ip link show can2 | grep -E "can2|state"
ip link show can3 | grep -E "can3|state"
echo ""

echo "=========================================="
echo "1. RMD 모터 제어 노드 실행 (can2)"
echo "=========================================="
ros2 run rmd_robot_control position_control_node &
MOTOR_PID=$!
sleep 3

echo ""
echo "=========================================="
echo "2. Iron-MD 텔레옵 노드 실행 (can3)"
echo "=========================================="
ros2 run rebar_control iron_md_teleop &
TELEOP_PID=$!
sleep 2

echo ""
echo "=========================================="
echo "✅ 통합 제어 시스템 시작 완료!"
echo "=========================================="
echo ""
echo "실행 중인 ROS2 노드:"
ros2 node list
echo ""
echo "사용 가능한 서비스:"
ros2 service list | grep brake
echo ""
echo "발행 중인 토픽:"
ros2 topic list | grep -E "cmd_vel|joint|position"
echo ""
echo "=========================================="
echo "🎮 Iron-MD 조종기 사용 방법"
echo "=========================================="
echo "1. S13 버튼: 브레이크 해제/잠금 토글"
echo "   - 첫 번째 누름: 브레이크 해제 (모터 움직일 수 있음)"
echo "   - 두 번째 누름: 브레이크 잠금 (모터 고정)"
echo ""
echo "2. S14 버튼: 현재 위치를 원점(0)으로 리셋"
echo ""
echo "3. 주행 제어 (AN3): 0x141, 0x142 모터"
echo "   - 전후진 및 회전"
echo ""
echo "4. 위치 제어:"
echo "   - AN1: X축 (0x144)"
echo "   - AN2: Y축 (0x145)"
echo "   - S17/S18: 횡이동 (0x143)"
echo "   - S23/S24: Yaw (0x147)"
echo ""
echo "=========================================="
echo "모니터링 명령어 (별도 터미널):"
echo "=========================================="
echo "# ROS2 토픽 확인:"
echo "ros2 topic echo /cmd_vel"
echo "ros2 topic echo /joint_states"
echo ""
echo "# CAN 버스 확인:"
echo "candump can2  # 모터 명령/응답"
echo "candump can3  # 조종기 입력"
echo ""
echo "# 브레이크 수동 제어:"
echo "ros2 service call /safe_brake_release std_srvs/srv/Trigger"
echo "ros2 service call /safe_brake_lock std_srvs/srv/Trigger"
echo ""
echo "=========================================="
echo "종료: Ctrl+C"
echo "=========================================="

# 종료 트랩
trap "echo ''; echo '노드 종료 중...'; kill $TELEOP_PID $MOTOR_PID 2>/dev/null; exit" SIGINT SIGTERM

# 계속 실행
wait
