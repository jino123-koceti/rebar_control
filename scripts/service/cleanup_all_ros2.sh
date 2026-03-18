#!/bin/bash
# ROS2 완전 정리 스크립트

echo "=========================================="
echo "ROS2 노드 완전 정리 스크립트"
echo "=========================================="
echo ""

# 1. 현재 노드 상태 확인
echo "[1/6] 현재 노드 상태 확인..."
ros2 node list 2>&1 | grep -v "WARNING\|dump_bash"
echo ""

# 2. ROS2 daemon 중지
echo "[2/6] ROS2 daemon 중지..."
ros2 daemon stop 2>/dev/null || true
sleep 2

# 3. 모든 ROS2 관련 프로세스 강제 종료
echo "[3/6] ROS2 관련 프로세스 강제 종료..."
pkill -9 -f "image_bridge" 2>/dev/null || true
pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "launch" 2>/dev/null || true
sleep 2

# 4. ROS2 로그 및 캐시 정리
echo "[4/6] ROS2 로그 및 캐시 정리..."
rm -rf ~/.ros/log/* 2>/dev/null || true
rm -rf ~/.ros/.ros2_daemon* 2>/dev/null || true
sleep 1

# 5. ROS2 daemon 재시작
echo "[5/6] ROS2 daemon 재시작..."
ros2 daemon start
sleep 3

# 6. 최종 노드 상태 확인
echo "[6/6] 최종 노드 상태 확인..."
echo ""
ros2 node list 2>&1 | grep -v "WARNING\|dump_bash" || echo "노드 없음"
echo ""

echo "=========================================="
echo "정리 완료!"
echo ""
echo "만약 여전히 중복 노드가 보인다면:"
echo "1. 다른 터미널에서 실행 중인 launch 파일 확인"
echo "2. 다른 사용자 세션 확인: who"
echo "3. Docker 컨테이너 확인: docker ps"
echo "4. 시스템 재부팅 (최후의 수단)"
echo "=========================================="

