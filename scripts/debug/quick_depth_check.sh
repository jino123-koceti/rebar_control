#!/bin/bash
# Depth 토픽 빠른 확인 스크립트

echo "====================================="
echo "Depth Topic Quick Check"
echo "====================================="
echo ""

source install/setup.bash

echo "1. Checking if depth topic is publishing..."
timeout 3 ros2 topic hz /zedxmini2/zedxmini2_node/depth/depth_registered 2>&1 | grep "average rate" | head -1 || echo "   ❌ Topic not found or not publishing"

echo ""
echo "2. Checking depth_info..."
ros2 topic echo /zedxmini2/zedxmini2_node/depth/depth_info --once 2>/dev/null || echo "   ❌ Could not get depth_info"

echo ""
echo "3. Quick depth statistics..."
python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import signal
import sys

def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(SIGINT, signal_handler)
signal.signal(SIGALRM, signal_handler)

class QuickCheck(Node):
    def __init__(self):
        super().__init__('quick_check')
        self.bridge = CvBridge()
        self.done = False
        self.subscription = self.create_subscription(
            Image, '/zedxmini2/zedxmini2_node/depth/depth_registered',
            self.callback, 10
        )

    def callback(self, msg):
        if self.done:
            return
        self.done = True

        depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        valid = depth[np.isfinite(depth) & (depth > 0)]

        if len(valid) > 0:
            print(f"   ✅ Depth data OK!")
            print(f"      - Valid pixels: {len(valid)}/{depth.size} ({100*len(valid)/depth.size:.1f}%)")
            print(f"      - Range: {valid.min():.3f}m - {valid.max():.3f}m")
            print(f"      - Mean: {valid.mean():.3f}m ({valid.mean()*1000:.0f}mm)")
        else:
            print(f"   ⚠️  No valid depth pixels!")
            print(f"      - Check if camera is viewing objects with texture")

        raise SystemExit(0)

rclpy.init()
node = QuickCheck()

# 5초 타임아웃
signal.alarm(5)

try:
    rclpy.spin(node)
except:
    pass
finally:
    if not node.done:
        print("   ❌ Timeout waiting for depth message")
    node.destroy_node()
    rclpy.try_shutdown()
PYEOF

echo ""
echo "====================================="
echo "Check complete!"
echo ""
echo "To visualize depth in real-time:"
echo "  python3 check_depth_live.py"
echo ""
echo "For RViz2 setup guide:"
echo "  cat RVIZ2_DEPTH_SETUP.md"
echo "====================================="
