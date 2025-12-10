#!/bin/bash
# Precision Navigation 테스트 스크립트

echo "=========================================="
echo "Precision Navigation Test Script"
echo "=========================================="
echo ""

cd /home/koceti/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. ZED 카메라 실행
echo "[1/2] Launching ZED X camera..."
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx > /tmp/zed_test_nav.log 2>&1 &
ZED_PID=$!
echo "  ZED PID: $ZED_PID"
sleep 10

# 2. Precision Navigation 노드 실행
echo "[2/2] Launching Precision Navigation Node..."
ros2 run rebar_control precision_navigation_node &
NAV_PID=$!
echo "  Nav PID: $NAV_PID"
sleep 3

echo ""
echo "=========================================="
echo "✅ System Ready!"
echo "=========================================="
echo ""
echo "ROS2 Topics:"
ros2 topic list | grep precision_nav
echo ""
echo "=========================================="
echo "📝 Test Commands:"
echo "=========================================="
echo ""
echo "# Test 1: Forward 600mm"
echo "ros2 topic pub /precision_nav/goal rebar_msgs/PrecisionNavGoal \\"
echo "  \"{type: 0, distance: 600.0, max_velocity: 0.3}\" --once"
echo ""
echo "# Test 2: Backward 600mm"
echo "ros2 topic pub /precision_nav/goal rebar_msgs/PrecisionNavGoal \\"
echo "  \"{type: 0, distance: -600.0, max_velocity: 0.3}\" --once"
echo ""
echo "# Monitor feedback"
echo "ros2 topic echo /precision_nav/feedback"
echo ""
echo "# Monitor status"
echo "ros2 topic echo /precision_nav/status"
echo ""
echo "# Monitor cmd_vel_precise"
echo "ros2 topic echo /cmd_vel_precise"
echo ""
echo "=========================================="
echo "Press Ctrl+C to stop"
echo "=========================================="

# 종료 트랩
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $NAV_PID $ZED_PID 2>/dev/null
    sleep 1
    echo "Done"
}

trap cleanup SIGINT SIGTERM EXIT

# 계속 실행
wait
