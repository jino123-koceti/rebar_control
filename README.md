# Rebar Tying Robot Control System

ROS2 Humble-based integrated control system for an automated rebar tying robot used in construction sites.

## System Overview

This project controls a rebar tying robot that automatically ties rebar intersections on construction sites. It integrates 7 RMD motors, an Iron-MD wireless remote controller, a Seengrip gripper, limit sensors, dual ZED X Mini cameras, and a YOLO-based vision system under ROS2 Humble.

### Key Features

- **7-axis RMD motor control**: Position/velocity control with event-driven feedback
- **Wireless remote control**: Iron-MD CAN protocol (4 joysticks, 24 buttons)
- **Dual CAN bus**: can2 (1 Mbps, motors), can3 (250 kbps, remote)
- **Gripper integration**: Seengrip Modbus RTU control
- **Limit sensors**: FASTECH EZI-IO Modbus TCP (16 input channels)
- **Dual camera vision**: ZED X Mini stereo cameras with YOLO rebar crossing detection
- **Automated tying sequence**: Vision-guided 3-axis stage positioning + S21/S22 tying cycles
- **Headless operation**: Log-based operation without monitor

## System Architecture

```
                    ┌───────────────────────────┐
                    │  Iron-MD Wireless Remote   │
                    │   (CAN3 @ 250 kbps)        │
                    └─────────────┬─────────────┘
                                  │
                    ┌─────────────▼─────────────┐
                    │   Jetson Orin / Ubuntu PC  │
                    │       ROS2 Humble          │
                    └─────────────┬─────────────┘
                                  │
       ┌──────────────┬───────────┼───────────┬──────────────┐
       │              │           │           │              │
  ┌────▼────┐   ┌─────▼─────┐ ┌──▼───┐ ┌─────▼─────┐  ┌────▼────┐
  │  CAN2   │   │Modbus RTU │ │ USB  │ │Modbus TCP │  │  GMSL2  │
  │ 1 Mbps  │   │ RS485     │ │Serial│ │ Ethernet  │  │ Camera  │
  └────┬────┘   └─────┬─────┘ └──┬───┘ └─────┬─────┘  └────┬────┘
       │              │          │            │              │
  ┌────▼──────┐  ┌────▼────┐ ┌──▼──────┐ ┌───▼──────┐ ┌────▼──────┐
  │ 7x RMD   │  │Seengrip │ │ Pololu  │ │ EZI-IO   │ │ 2x ZED X │
  │ Motors   │  │ Gripper │ │Trigger  │ │ Sensors  │ │   Mini   │
  │0x141-147 │  │         │ │ Motor   │ │ 16ch I/O │ │ (Stereo) │
  └──────────┘  └─────────┘ └─────────┘ └──────────┘ └──────────┘
```

### Motor Configuration

| Motor ID | Function | Control Type | Measured Specs |
|----------|----------|--------------|----------------|
| 0x141 | Left drive motor | Velocity (0xA2) | Differential drive |
| 0x142 | Right drive motor | Velocity (0xA2) | Differential drive |
| 0x143 | Lateral traverse | Position (0xA4) | 360 CPR encoder |
| 0x144 | X-axis stage | Velocity (0xA2) | 2.698 deg/mm, 420 mm stroke |
| 0x145 | Y-axis stage | Velocity (0xA2) | 2.677 deg/mm, 300 mm stroke |
| 0x146 | Z-axis (up/down) | Position (0xA4) | ~2.69 deg/mm |
| 0x147 | Yaw (rotation) | Position (0xA4) | 243 deg stroke |

### Camera Setup

Two ZED X Mini cameras mounted in an inward-facing V configuration:

| Camera | Position (mm) | Tilt | Yaw | Facing |
|--------|---------------|------|-----|--------|
| Left (zedxmini1) | [-200, +100, 108] | 40 deg | -20 deg | Right-inward |
| Right (zedxmini2) | [-200, -100, 108] | 40 deg | +20 deg | Left-inward |

## Package Structure

### rebar_base_control
Core robot control nodes for the rebar tying robot.

| Node | Description |
|------|-------------|
| `can_parser.py` | CAN message receiver and ROS2 message converter |
| `can_sender.py` | ROS2 to CAN message transmitter |
| `drive_controller.py` | Differential drive kinematics (`/cmd_vel` to motor commands) |
| `joint_controller.py` | Stage/yaw motor control (0x143-0x147) with joystick and button input |
| `sequence_controller.py` | S21/S22 tying work sequences (descend, grip, trigger, ascend) |
| `authority_controller.py` | S10 (Manual) / S20 (Auto) mode switching |
| `ezi_io_controller.py` | FASTECH EZI-IO limit sensor monitoring via Modbus TCP |
| `modbus_controller.py` | Seengrip gripper + EZI-IO integration |
| `navigator_base.py` | State machine for autonomous navigation |
| `lateral_motion.py` | Lateral traverse (0x143) motion module |

### rebar_base_interfaces
Custom ROS2 message and service definitions.

| Type | Name | Description |
|------|------|-------------|
| msg | DriveControl | Differential drive motor commands |
| msg | MotorFeedback | Motor position/velocity/status feedback |
| msg | RemoteControl | Iron-MD remote controller state |
| msg | JointControl | Joint position/velocity commands |
| msg | GripperControl | Gripper open/close commands |
| msg | IOStatus | Digital I/O sensor states |
| msg | RebarDetection | Single rebar crossing detection result (3D coordinates) |
| msg | RebarGrid | 2x3 grid of detected rebar crossings |
| srv | DetectCrossings | Request rebar crossing detection from vision system |

### rebar_vision
Vision system for rebar crossing detection and automated tying orchestration.

| Node | Description |
|------|-------------|
| `rebar_detection_node.py` | YOLO-based rebar crossing detection with dual camera fusion |
| `tying_orchestrator_node.py` | Automated tying sequence: detect crossings, move stage, trigger S21/S22 |
| `dual_camera_recorder_node.py` | Dual ZED X Mini data collection for training |

**Detection strategies**: `cross` (each camera detects far-side points), `merge` (combine both cameras), `single` (one camera only)

### rmd_robot_control
Low-level RMD motor communication and control.

| Node | Description |
|------|-------------|
| `position_control_node.py` | 7-motor integrated control (CAN protocol) |
| `can_manager.py` | SocketCAN communication manager |

### Supporting Packages

| Package | Description |
|---------|-------------|
| `seengrip_ros2` | Seengrip gripper Modbus RTU driver |
| `ezi_io_ros2` | FASTECH EZI-IO limit sensor Modbus TCP driver |
| `pololu_ros2` | Pololu Simple Motor Controller driver (trigger motor) |

## Quick Start

### Prerequisites

- **OS**: Ubuntu 22.04 LTS (Jetson Orin / x86_64)
- **ROS**: ROS2 Humble
- **Hardware**:
  - 2x CAN interfaces (can2, can3)
  - USB-Serial adapter (/dev/ttyUSB0)
  - Ethernet (192.168.0.x network)
  - 2x ZED X Mini cameras (GMSL2)

### Installation

```bash
# Clone repository
git clone https://github.com/jino123-koceti/rebar_control.git
cd rebar_control

# Install ROS2 dependencies
sudo apt update
sudo apt install ros-humble-desktop python3-pip

# Install Python packages
pip3 install python-can pymodbus pyserial ultralytics

# Setup CAN interfaces
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can3 type can bitrate 250000
sudo ip link set can2 up
sudo ip link set can3 up

# Build
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Running

#### Method 1: Systemd Service (Production)

```bash
# Enable and start the service
sudo systemctl enable robot-control.service
sudo systemctl start robot-control.service

# Check status
sudo systemctl status robot-control.service

# View logs
journalctl -u robot-control.service -f
```

#### Method 2: Launch Files

```bash
# Base control system (motors, remote, sensors)
ros2 launch rebar_base_control base_system.launch.py

# Vision-guided tying system (cameras + detection + orchestrator)
ros2 launch rebar_vision vision_tying.launch.py
```

#### Method 3: Individual Nodes

```bash
# Terminal 1: RMD motor control
ros2 run rmd_robot_control position_control_node

# Terminal 2: EZI-IO limit sensors
ros2 run ezi_io_ros2 ezi_io_node --ros-args -p ip_address:=192.168.0.6

# Terminal 3: Seengrip gripper
ros2 run seengrip_ros2 seengrip_node --ros-args -p serial_port:=/dev/ttyUSB0
```

## Remote Controller Mapping (Iron-MD)

### Joysticks (Analog)

| Joystick | Function | Motor | Range |
|----------|----------|-------|-------|
| AN3 | Forward/Backward | 0x141, 0x142 | -1.0 to 1.0 |
| AN4 | Left/Right turn | 0x141, 0x142 | -1.0 to 1.0 |
| AN1 | X-axis stage | 0x144 | +/-200 dps |
| AN2 | Y-axis stage | 0x145 | +/-200 dps |

> **Note**: Drive motors (0x141, 0x142) are mounted 180 degrees rotated, so AN3/AN4 mapping is inverted in code.

### Buttons (Digital)

| Button | Function | Description |
|--------|----------|-------------|
| S10 | Manual mode | Switch to manual (remote) control |
| S13 | Brake toggle | Motor brake release/lock |
| S14 | Homing | Drive motor home limit search |
| S17 | Lateral + | 0x143 +360 deg rotation (one full lead screw turn) |
| S18 | Lateral - | 0x143 -360 deg rotation |
| S20 | Auto mode | Switch to autonomous operation |
| S21 | Work sequence 1 | Z-axis descend, gripper close |
| S22 | Work sequence 2 | Trigger, gripper open, Z-axis ascend |
| S23 | Yaw + | 0x147 +45 deg rotation |
| S24 | Yaw - | 0x147 -45 deg rotation |

## Configuration

### CAN Device Config: `src/rebar_base_control/config/can_devices.yaml`

Key parameters (measured 2026-03-04):
- Stage X: 2.698 deg/mm (1133.29 deg / 420 mm)
- Stage Y: 2.677 deg/mm (803.09 deg / 300 mm)
- Stage Z: ~2.69 deg/mm (estimated, same mechanism)
- Yaw: 243 deg total stroke (mechanical stoppers)

### Limit Sensor Channel Mapping (EZI-IO)

| Channel | Sensor | Description |
|---------|--------|-------------|
| IN00 | Y-axis max | Y+ limit |
| IN01 | Y-axis min | Y- limit |
| IN02 | X-axis min | X- limit |
| IN03 | X-axis max | X+ limit |
| IN04 | Yaw home | Yaw home position sensor |
| IN05 | Z-axis min | Z bottom limit |
| IN06 | Z-axis max | Z top limit |

## Troubleshooting

### CAN Bus Errors

```bash
# Restart CAN interface
sudo ip link set can2 down && sudo ip link set can2 up

# Check CAN statistics
ip -details -statistics link show can2

# Bus-off recovery
sudo ip link set can2 type can restart-ms 100
```

### Motor Error Codes

| Code | Description |
|------|-------------|
| 0x01 | Low voltage |
| 0x02 | Overcurrent |
| 0x04 | Overtemperature |
| 0x08 | Encoder error |
| 0x10 | Overload |

### Gripper Connection

```bash
# Check serial port
ls -l /dev/ttyUSB* /dev/ttyACM*

# Set permissions
sudo chmod 666 /dev/ttyUSB0

# Or add user to dialout group (permanent)
sudo usermod -a -G dialout $USER
```

## Utility Scripts

| Script | Description |
|--------|-------------|
| `measure_stroke.py` | Interactive motor stroke measurement via CAN (0x92 multi-turn read) |
| `check_motor_alarm.py` | Check motor error/alarm status (0x9A) |
| `cleanup_ros2_nodes.sh` | Clean up orphaned ROS2 processes |
| `robot_control_service.sh` | Systemd service management helper |

## Safety Features

- **Emergency stop**: Iron-MD remote E-stop button (immediate all-motor stop)
- **Limit sensors**: 7 sensors on EZI-IO for overtravel prevention (X/Y/Z min/max + Yaw home)
- **Brake system**: Automatic brake engagement on power loss
- **Error detection**: Motor overtemp/overcurrent auto-detection and logging
- **Stroke limits**: Software-enforced stage travel limits based on measured values

## License

MIT License

## Team

- **KOCETI Robotics Team**
- GitHub: [@jino123-koceti](https://github.com/jino123-koceti)

---

**Last Updated**: 2026-03-04
**ROS2 Version**: Humble
**Platform**: Jetson Orin (Ubuntu 22.04 LTS, aarch64)
