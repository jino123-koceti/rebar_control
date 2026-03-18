# External PC Communication Protocol

## Overview

This document describes the communication protocol for sending navigation commands from an external PC to the Rebar robot via Zenoh.

## Network Configuration

- **Protocol**: Zenoh (peer-to-peer or client-server mode)
- **Network**: Same LAN as robot
- **Default Mode**: Peer mode (automatic peer discovery)
- **Port**: 7447 (default Zenoh port)

## Zenoh Configuration

### Robot Side (Receiver)
```yaml
# src/rebar_control/config/zenoh_config.yaml
zenoh_client:
  ros__parameters:
    zenoh_mode: "peer"
    command_key: "rebar/command"      # UI → Robot
    status_key: "rebar/status"        # Robot → UI
    pose_key: "rebar/pose"            # Robot → UI
```

### External PC Side (Sender)
```python
import zenoh

# Peer mode (recommended - automatic discovery)
config = zenoh.Config()
session = zenoh.open(config)

# OR Client mode (requires router address)
config = zenoh.Config()
config.insert_json5("connect/endpoints", '["tcp/192.168.1.XXX:7447"]')
session = zenoh.open(config)
```

## Message Protocol

### 1. Command Key
- **Zenoh Key**: `rebar/command`
- **Format**: JSON string
- **Encoding**: UTF-8

### 2. Command Types

#### A. Waypoint Mission Command

**Message Structure**:
```json
{
  "waypoints": [
    {"x": 0.0, "y": 0.0},
    {"x": 0.2, "y": 0.0},
    {"x": 0.2, "y": 0.05},
    ...
  ]
}
```

**Field Specifications**:
| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `waypoints` | Array | - | List of waypoint objects |
| `waypoints[].x` | Float | meters | X coordinate (forward/backward) |
| `waypoints[].y` | Float | meters | Y coordinate (lateral) |

**Important Notes**:
- Coordinates are in **meters** (the navigator auto-detects units)
- Origin (0,0) is the robot's starting position
- X-axis: Forward/backward motion (uses differential drive)
- Y-axis: Lateral motion (uses lateral motor 0x143)
- **Unit Auto-Detection**: If first waypoint has x or y > 10, treated as mm; otherwise meters
- **Auto-Start**: In auto mode, waypoints are automatically started after loading

#### B. Example: 9-Waypoint Test Path

**Path in mm**:
```
x = [0, 200, 200, 200, 400, 400, 400, 200, 0]
y = [0, 0, 50, 100, 100, 50, 0, 0, 0]
```

**Correct JSON Message** (converted to meters):
```json
{
  "waypoints": [
    {"x": 0.0, "y": 0.0},
    {"x": 0.2, "y": 0.0},
    {"x": 0.2, "y": 0.05},
    {"x": 0.2, "y": 0.1},
    {"x": 0.4, "y": 0.1},
    {"x": 0.4, "y": 0.05},
    {"x": 0.4, "y": 0.0},
    {"x": 0.2, "y": 0.0},
    {"x": 0.0, "y": 0.0}
  ]
}
```

**Alternative Format** (also supported):
```json
{
  "command": "START_MISSION"
}
```
or
```json
{
  "command": "PAUSE"
}
```

**Note**: Both `{"waypoints": [...]}` and `{"command": "..."}` formats are now supported!

## Python Implementation

### Complete Example Script

```python
#!/usr/bin/env python3
"""
External PC script to send waypoint mission to Rebar robot via Zenoh
"""

import zenoh
import json
import time

def send_waypoint_mission():
    """Send 9-waypoint test path to robot"""

    # 1. Initialize Zenoh session (peer mode)
    print("Initializing Zenoh session...")
    config = zenoh.Config()
    session = zenoh.open(config)
    print("✅ Zenoh session opened")

    # 2. Define waypoints (in meters)
    waypoints = [
        {"x": 0.0, "y": 0.0},
        {"x": 0.2, "y": 0.0},
        {"x": 0.2, "y": 0.05},
        {"x": 0.2, "y": 0.1},
        {"x": 0.4, "y": 0.1},
        {"x": 0.4, "y": 0.05},
        {"x": 0.4, "y": 0.0},
        {"x": 0.2, "y": 0.0},
        {"x": 0.0, "y": 0.0}
    ]

    # 3. Create command message
    command = {
        "waypoints": waypoints
    }

    # 4. Convert to JSON string
    json_str = json.dumps(command)

    # 5. Send via Zenoh
    print(f"\n📤 Sending waypoint mission...")
    print(f"   Total waypoints: {len(waypoints)}")
    print(f"   Message: {json_str[:100]}...")

    session.put("rebar/command", json_str)
    print("✅ Command sent successfully")

    # 6. Wait a moment for transmission
    time.sleep(0.5)

    # 7. Close session
    session.close()
    print("✅ Session closed")

if __name__ == '__main__':
    send_waypoint_mission()
```

### Usage

```bash
# On external PC
python3 send_waypoint_mission.py
```

**Expected Output**:
```
Initializing Zenoh session...
✅ Zenoh session opened

📤 Sending waypoint mission...
   Total waypoints: 9
   Message: {"waypoints": [{"x": 0.0, "y": 0.0}, {"x": 0.2, "y": 0.0}, ...
✅ Command sent successfully
✅ Session closed
```

## Robot Side Message Processing

### 1. Message Reception Flow

```
External PC
    ↓ (Zenoh: rebar/command)
zenoh_client.py
    ↓ (Parse JSON)
    ↓ (ROS2 Topic: /mission/command)
navigator.py
    ↓ (Motion type detection)
    ↓ (ROS2 Topic: /mission/enhanced_target)
rebar_controller.py
    ↓ (Differential) → drive_controller → CAN (0x141, 0x142)
    ↓ (Lateral) → joint_controller → CAN (0x143)
Motors
```

### 2. Motion Type Auto-Detection

The `navigator.py` automatically detects motion type for each segment:

| Condition | Motion Type | Description |
|-----------|-------------|-------------|
| `dy > 1mm AND dx < 1mm` | **LATERAL** | Pure Y-axis motion using lateral motor |
| Otherwise | **DIFFERENTIAL** | X-axis motion or diagonal using differential drive |

**Example Detection** for 9-waypoint path:
```
Waypoint 0 → 1: dx=200mm, dy=0mm   → DIFFERENTIAL (forward)
Waypoint 1 → 2: dx=0mm, dy=50mm    → LATERAL (sideways)
Waypoint 2 → 3: dx=0mm, dy=50mm    → LATERAL (sideways)
Waypoint 3 → 4: dx=200mm, dy=0mm   → DIFFERENTIAL (forward)
Waypoint 4 → 5: dx=0mm, dy=-50mm   → LATERAL (sideways)
Waypoint 5 → 6: dx=0mm, dy=-50mm   → LATERAL (sideways)
Waypoint 6 → 7: dx=-200mm, dy=0mm  → DIFFERENTIAL (backward)
Waypoint 7 → 8: dx=-200mm, dy=0mm  → DIFFERENTIAL (backward)
```

## Status Feedback (Robot → External PC)

### Status Message Structure

**Zenoh Key**: `rebar/status`
**Format**: MessagePack binary

```python
{
    "state": "NAVIGATING",           # State: IDLE, NAVIGATING, PAUSED, COMPLETED
    "current_waypoint": 3,           # Current waypoint index
    "total_waypoints": 9,            # Total waypoints
    "position": {"x": 0.2, "y": 0.05},  # Current position (meters)
    "progress": 33.3                 # Progress percentage
}
```

### Pose Feedback

**Zenoh Key**: `rebar/pose`
**Format**: MessagePack binary
**Rate**: 20 Hz

```python
{
    "x": 0.15,          # X position (meters)
    "y": 0.03,          # Y position (meters)
    "yaw": 0.0,         # Heading angle (radians)
    "timestamp": 1234567890.123
}
```

### Subscribing to Feedback (External PC)

```python
import zenoh
import msgpack

def status_listener(sample):
    """Handle status updates from robot"""
    data = msgpack.unpackb(sample.payload.to_bytes())
    print(f"Status: {data['state']}, Waypoint: {data['current_waypoint']}/{data['total_waypoints']}")

def pose_listener(sample):
    """Handle pose updates from robot"""
    data = msgpack.unpackb(sample.payload.to_bytes())
    print(f"Pose: x={data['x']:.3f}m, y={data['y']:.3f}m, yaw={data['yaw']:.2f}rad")

# Subscribe
session = zenoh.open(zenoh.Config())
status_sub = session.declare_subscriber("rebar/status", status_listener)
pose_sub = session.declare_subscriber("rebar/pose", pose_listener)

# Keep running
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
```

## Testing Procedure

### 1. Robot Side

```bash
# Terminal 1: Launch robot system
ros2 launch rebar_control full_system.launch.py

# Terminal 2 (optional): Monitor Zenoh messages
python3 test_zenoh_receive.py
```

### 2. External PC Side

```bash
# Send waypoint mission
python3 send_waypoint_mission.py
```

### 3. Verification

**Check robot logs for**:
```
[zenoh_client]: Received command on rebar/command
[navigator]: Loaded 9 waypoints
[navigator]: Waypoint 0→1: DIFFERENTIAL motion
[navigator]: Waypoint 1→2: LATERAL motion
[rebar_controller]: Executing differential motion to (0.200, 0.000)
[rebar_controller]: Executing lateral motion to (0.200, 0.050)
...
```

## Troubleshooting

### Issue: Robot not receiving commands

**Symptoms**: External PC sends but robot doesn't respond

**Checks**:
1. Verify robot system is running:
   ```bash
   ros2 node list | grep zenoh_client
   ```

2. Check network connectivity:
   ```bash
   ping <robot_ip>
   ```

3. Verify Zenoh peers:
   ```bash
   # On robot
   python3 test_zenoh_receive.py

   # On external PC
   python3 send_waypoint_mission.py
   ```

4. Check firewall settings (port 7447)

### Issue: Invalid JSON format

**Symptom**: Robot receives but doesn't parse

**Solution**: Ensure JSON only contains `waypoints` array:
```python
# ✅ Correct
{"waypoints": [...]}

# ❌ Incorrect
{"command": "WAYPOINTS", "waypoints": [...]}
```

### Issue: Robot moves incorrectly

**Symptoms**: Wrong motion type or direction

**Checks**:
1. Verify coordinate units (should be meters, not mm)
2. Check motion type detection threshold (1mm = 0.001m)
3. Review navigator logs for motion type detection

## Additional Commands (Future Extension)

### Pause Mission
```json
{"command": "PAUSE"}
```

### Resume Mission
```json
{"command": "RESUME"}
```

### Cancel Mission
```json
{"command": "CANCEL"}
```

### Emergency Stop
```json
{"command": "EMERGENCY_STOP"}
```

**Note**: These commands are not yet implemented but reserved for future use.

## Technical Specifications

### Lateral Motion
- **Motor**: 0x143
- **Conversion**: 50mm per 360° rotation
- **Speed**: 200 dps (degrees per second)
- **Accuracy**: ±5mm target

### Differential Motion
- **Motors**: 0x141 (left), 0x142 (right)
- **Control**: cmd_vel (linear.x, angular.z)
- **Max Speed**: 0.5 m/s linear, 1.0 rad/s angular
- **Accuracy**: ±10mm target

### Visual SLAM
- **Forward Motion**: Uses `forward_zed` camera
- **Backward Motion**: Uses `backward_zed` camera
- **Switching**: Automatic via `pose_mux` based on cmd_vel.linear.x sign

## Reference Files

- Robot Zenoh Config: [src/rebar_control/config/zenoh_config.yaml](src/rebar_control/config/zenoh_config.yaml)
- Example Script: [src/rebar_control/scripts/send_hybrid_path.py](src/rebar_control/scripts/send_hybrid_path.py)
- Test Receiver: [test_zenoh_receive.py](test_zenoh_receive.py)
- Development Summary: [REBAR_DEVELOPMENT_SUMMARY.md](REBAR_DEVELOPMENT_SUMMARY.md)
