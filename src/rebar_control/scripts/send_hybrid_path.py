#!/usr/bin/env python3
"""
Send Hybrid Path Script (External PC)

외부 PC에서 Zenoh를 통해 Rebar 로봇에 복합 경로 전송
- 전진/후진: X축 이동
- 횡이동: Y축 이동 (회전 없음)

Requirements:
    pip3 install zenoh

Usage:
    python3 send_hybrid_path.py --host <robot_ip>
"""

import zenoh
import json
import time
import argparse


def send_test_path(zenoh_mode='peer', robot_host=None):
    """
    테스트 경로 전송

    경로:
    x = [0, 200, 200, 200, 400, 400, 400, 200, 0] mm
    y = [0, 0, 50, 100, 100, 50, 0, 0, 0] mm

    Motion sequence:
    1. (0,0) → (200,0): Forward 200mm
    2. (200,0) → (200,50): Lateral +50mm
    3. (200,50) → (200,100): Lateral +50mm
    4. (200,100) → (400,100): Forward 200mm
    5. (400,100) → (400,50): Lateral -50mm
    6. (400,50) → (400,0): Lateral -50mm
    7. (400,0) → (200,0): Backward 200mm
    8. (200,0) → (0,0): Backward 200mm
    """

    # Zenoh configuration
    config = zenoh.Config()

    if zenoh_mode == 'client' and robot_host:
        # Client mode: connect to specific robot
        config.insert_json5(zenoh.config.MODE_KEY, json.dumps('client'))
        config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([f"tcp/{robot_host}:7447"]))
        print(f"🔗 Zenoh client mode: connecting to {robot_host}:7447")
    else:
        # Peer mode: multicast discovery
        print("🔗 Zenoh peer mode: using multicast discovery")

    session = zenoh.open(config)

    # Define waypoints (in mm)
    waypoints = [
        {"x": 0, "y": 0},       # Start
        {"x": 200, "y": 0},     # Forward 200mm
        {"x": 200, "y": 50},    # Lateral +50mm
        {"x": 200, "y": 100},   # Lateral +50mm
        {"x": 400, "y": 100},   # Forward 200mm
        {"x": 400, "y": 50},    # Lateral -50mm
        {"x": 400, "y": 0},     # Lateral -50mm
        {"x": 200, "y": 0},     # Backward 200mm
        {"x": 0, "y": 0},       # Backward 200mm (return to start)
    ]

    print("\n" + "="*70)
    print("📋 Hybrid Navigation Test Path")
    print("="*70)
    print(f"Total waypoints: {len(waypoints)}")
    print("\nPath breakdown:")
    for i, wp in enumerate(waypoints):
        if i == 0:
            print(f"  [{i}] Start: ({wp['x']}, {wp['y']}) mm")
        else:
            prev = waypoints[i-1]
            dx = wp['x'] - prev['x']
            dy = wp['y'] - prev['y']
            if abs(dy) > 1 and abs(dx) < 1:
                motion = f"Lateral {dy:+.0f}mm"
            elif dx > 0:
                motion = f"Forward {dx:.0f}mm"
            elif dx < 0:
                motion = f"Backward {abs(dx):.0f}mm"
            else:
                motion = "Unknown"
            print(f"  [{i}] → ({wp['x']}, {wp['y']}) mm : {motion}")
    print("="*70)

    # Send waypoints
    waypoint_data = {"waypoints": waypoints}
    waypoint_json = json.dumps(waypoint_data)
    command_str = f"WAYPOINTS:{waypoint_json}"

    print(f"\n📤 Sending waypoints to 'rebar/command'...")
    session.put("rebar/command", command_str)
    print("✅ Waypoints sent")

    time.sleep(1)

    # Start mission
    print(f"\n🚀 Starting mission...")
    session.put("rebar/command", "START_MISSION")
    print("✅ Mission started")

    print("\n" + "="*70)
    print("Mission commands sent successfully!")
    print("Monitor robot status on 'rebar/status' topic")
    print("="*70)

    # Close session
    time.sleep(0.5)
    session.close()


def main():
    parser = argparse.ArgumentParser(
        description='Send hybrid navigation path to Rebar robot via Zenoh'
    )
    parser.add_argument(
        '--mode',
        type=str,
        default='peer',
        choices=['peer', 'client'],
        help='Zenoh mode (peer=multicast, client=direct connect)'
    )
    parser.add_argument(
        '--host',
        type=str,
        default=None,
        help='Robot IP address (for client mode)'
    )

    args = parser.parse_args()

    if args.mode == 'client' and args.host is None:
        print("❌ Error: --host required for client mode")
        parser.print_help()
        return

    try:
        send_test_path(zenoh_mode=args.mode, robot_host=args.host)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
