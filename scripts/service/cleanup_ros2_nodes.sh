#!/bin/bash
# ROS2 노드 정리 스크립트

echo "=== ROS2 노드 정리 시작 ==="

# 1. ROS2 daemon 중지
echo "[1/4] ROS2 daemon 중지..."
ros2 daemon stop 2>/dev/null || true
sleep 1

# 2. 모든 ROS2 관련 프로세스 종료
echo "[2/4] ROS2 관련 프로세스 종료..."
pkill -9 -f "image_bridge" 2>/dev/null || true
pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "ros2.*launch" 2>/dev/null || true
pkill -9 -f "ros2.*run" 2>/dev/null || true
sleep 2

# 3. ROS2 daemon 재시작
echo "[3/4] ROS2 daemon 재시작..."
ros2 daemon start
sleep 2

# 4. 노드 목록 확인
echo "[4/4] 노드 목록 확인..."
echo ""
ros2 node list 2>&1

echo ""
echo "=== 정리 완료 ==="

