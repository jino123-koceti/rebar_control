# Rebar Tying Robot Control System

ROS2 Humble-based integrated control system for an automated rebar tying robot used in construction sites.

## System Overview

This project controls a rebar tying robot that automatically detects and ties rebar intersections on construction sites. It integrates 7 RMD motors, an Iron-MD wireless remote controller, a Seengrip gripper, limit sensors, dual ZED X Mini cameras, and a YOLO-based vision system under ROS2 Humble.

### Key Features

- **7-axis RMD motor control**: Position/velocity control with event-driven feedback via CAN bus
- **Automated homing**: Simultaneous multi-axis homing with coarse/fine approach and limit sensors
- **Dual camera vision**: ZED X Mini stereo cameras with YOLO rebar crossing detection
- **Vision-guided tying**: Detect crossings → coordinate transform → stage positioning → automatic tying
- **Calibration system**: Multi-linear regression (pixel + depth → robot XY mm) with cross terms
- **Wireless remote control**: Iron-MD CAN protocol (4 joysticks, 24 buttons)
- **Dual CAN bus**: can2 (1 Mbps, motors), can3 (250 kbps, remote)
- **Gripper integration**: Seengrip Modbus RTU control
- **Limit sensors**: FASTECH EZI-IO Modbus TCP (16 input channels)
- **Headless operation**: Systemd service-based, log-based operation without monitor

## System Architecture

```
                    ┌───────────────────────────┐
                    │  Iron-MD Wireless Remote   │
                    │   (CAN3 @ 250 kbps)        │
                    └─────────────┬─────────────┘
                                  │
                    ┌─────────────▼─────────────┐
                    │   Jetson AGX Orin          │
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

| Motor ID | Model | Function | Control Type | Measured Specs |
|----------|-------|----------|--------------|----------------|
| 0x141 | X4-36 | Left drive | Velocity (0xA2) | Accel 3,000 dps/s |
| 0x142 | X4-36 | Right drive | Velocity (0xA2) | Accel 3,000 dps/s |
| 0x143 | X4-36 | Lateral traverse | Position (0xA4) | 360 CPR encoder |
| 0x144 | X4-10 | X-axis stage | Position (0xA4) | 4.497 deg/mm, 411 mm stroke |
| 0x145 | X4-10 | Y-axis stage | Position (0xA4) | 4.462 deg/mm, 288 mm stroke |
| 0x146 | X4-10 | Z-axis (up/down) | Position (0xA4) | 13.45 deg/mm |
| 0x147 | X4-10 | Yaw (rotation) | Position (0xA4) | 399° full stroke (0x92 multi-turn) |

#### Motor Direction Mapping (Confirmed 2026-03-09)

- **X-axis (0x144)**: Motor positive = physical xMin → workspace (xMax) is negative direction
- **Y-axis (0x145)**: Motor positive = physical yMax → workspace (yMax) is positive direction
- Coordinate conversion: `X = home_ref - mm × deg_per_mm`, `Y = home_ref + mm × deg_per_mm`

#### Stage Motor Speed/Acceleration (Tested 2026-03-09)

| Parameter | Setting | Actual |
|-----------|---------|--------|
| Max speed command | 400 dps | Output shaft saturates at ~230 dps |
| Position acceleration | 5,000 dps/s | Effective ~400 dps/s (torque-limited) |
| 800° travel time | - | 2.90s (trapezoidal profile) |

> Commands above 400 dps yield no speed improvement due to motor physical limits (RMD X4-10, gear ratio 12.6:1, rated 238 rpm).

### Camera Setup

Two ZED X Mini cameras mounted in an inward-facing V configuration:

| Camera | Serial | Position (mm) | Tilt | Yaw | Coverage |
|--------|--------|---------------|------|-----|----------|
| Right (zedxmini2) | SN used for right | [-200, -100, 108] | 40° | +20° | Left-inward |
| Left (zedxmini1) | SN used for left | [-200, +100, 108] | 40° | -20° | Right-inward |

### Vision Calibration System

Multi-linear regression with cross terms: `(pixel_u, pixel_v, depth_mm) → (robot_X_mm, robot_Y_mm)`

| Camera | Data Points | X R² | Y R² | Mean Error |
|--------|-------------|------|------|------------|
| Right (zedxmini2) | 78 pairs | 0.9965 | 0.9811 | 6.72 mm |
| Left (zedxmini1) | 24 pairs | 0.9987 | 0.9879 | 5.67 mm |

Calibration files: `calibration_result_left.yaml`, `calibration_result_right.yaml`

## Package Structure

### rebar_base_control
Core robot control nodes for the rebar tying robot.

| Node | Description |
|------|-------------|
| `can_parser.py` | CAN message receiver with motor monitoring (temp, current, speed) |
| `can_sender.py` | ROS2 to CAN message transmitter (0xA2/0xA4/0x92 commands) |
| `drive_controller.py` | Differential drive kinematics (`/cmd_vel` to motor commands) |
| `joint_controller.py` | Stage/yaw motor control with homing state machine |
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
| `rebar_detection_node.py` | YOLO-based rebar crossing detection with calibration coordinate transform |
| `tying_orchestrator_node.py` | Automated tying sequence: detect → coordinate transform → stage move → tie |
| `dual_camera_recorder_node.py` | Dual ZED X Mini data collection for training |

**Detection strategies**: `cross` (each camera detects far-side points), `merge` (combine both cameras), `single` (one camera only)

**Tying orchestrator features**:
- Home-relative coordinate system (robust to motor zero point changes)
- Per-point XY stage positioning with goal-based completion (position tolerance 1.0°)
- Z-axis descent/ascent for tying head engagement
- Yaw pose change with mid-angle waypoint for dual-camera coverage
- Range filtering (0 ~ max_stage_mm) to skip out-of-range points
- Configurable speed, timeout, settling parameters

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

## Homing System

Simultaneous multi-axis homing with coarse/fine approach:

1. **Coarse approach**: All axes move toward home limit sensors at 100 dps
2. **Backoff**: On sensor trigger, briefly reverse (0.5s)
3. **Fine approach**: Slow approach at 30 dps for precision
4. **Zero set**: Record home reference angles (0x92 multi-turn position)
5. **Ready position**: Move to configured ready offsets

Yaw homing uses 0x90 absolute encoder (16-bit, persistent across power cycles) instead of multi-turn.

## Quick Start

### Prerequisites

- **OS**: Ubuntu 22.04 LTS (Jetson AGX Orin)
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
# Restart service
sudo systemctl restart robot-control

# Check status
sudo systemctl status robot-control

# View logs
journalctl -u robot-control -f
```

#### Method 2: Launch Files

```bash
# Full system (base control + vision)
ros2 launch rebar_control full_system.launch.py

# Base control only
ros2 launch rebar_base_control base_system.launch.py

# Vision system only
ros2 launch rebar_vision vision_tying.launch.py
```

## Remote Controller Mapping (Iron-MD)

### Joysticks (Analog)

| Joystick | Function | Motor | Range |
|----------|----------|-------|-------|
| AN3 | Forward/Backward | 0x141, 0x142 | -1.0 to 1.0 |
| AN4 | Left/Right turn | 0x141, 0x142 | -1.0 to 1.0 |
| AN1 | X-axis stage | 0x144 | ±200 dps |
| AN2 | Y-axis stage | 0x145 | ±200 dps |

> **Note**: Drive motors (0x141, 0x142) are mounted 180° rotated, so AN3/AN4 mapping is inverted in code.

### Buttons (Digital)

| Button | Function | Description |
|--------|----------|-------------|
| S10 | Manual mode | Switch to manual (remote) control |
| S13 | Brake toggle | Motor brake release/lock |
| S14 | Homing | Start simultaneous multi-axis homing sequence |
| S17 | Lateral + | 0x143 +360° rotation (one full lead screw turn) |
| S18 | Lateral - | 0x143 -360° rotation |
| S20 | Auto mode | Switch to autonomous tying operation |
| S21 | Work seq 1 | Z descend + gripper close |
| S22 | Work seq 2 | Trigger + gripper open + Z ascend |
| S23 | Yaw + | 0x147 +45° rotation |
| S24 | Yaw - | 0x147 -45° rotation |

## Configuration

### Tying Orchestrator: `src/rebar_vision/config/tying_orchestrator.yaml`

| Parameter | Value | Description |
|-----------|-------|-------------|
| stage_max_speed_dps | 400.0 | Optimal XY speed (output shaft saturates at ~230 dps) |
| position_tolerance_deg | 1.0 | Goal tolerance (~0.22 mm) |
| max_stage_timeout | 10.0 s | Move timeout |
| max_stage_x_mm | 411.0 | X stroke limit |
| max_stage_y_mm | 288.0 | Y stroke limit (interference margin) |
| max_yaw_deg | 399.0 | Yaw full stroke |
| z_speed_dps | 200.0 | Z-axis speed |

### CAN Device Config: `src/rebar_base_control/config/can_devices.yaml`

Key parameters (measured 2026-03-04, updated 2026-03-09):
- Stage X: 4.497 deg/mm, 411 mm stroke
- Stage Y: 4.462 deg/mm, 288 mm stroke
- Stage Z: 13.45 deg/mm
- Yaw: 399° full stroke (0x92 multi-turn)
- Drive speed scale: 25% of rated (testing)

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

### Motor Error Codes (0x9A Status Query)

| Bit | Code | Description |
|-----|------|-------------|
| 0 | 0x0001 | Low voltage protection |
| 1 | 0x0002 | Over voltage protection |
| 2 | 0x0004 | Over current protection |
| 3 | 0x0008 | Over temperature (MOS) |
| 4 | 0x0010 | Magnetic encoding error |
| 5 | 0x0020 | Hall encoding error |
| 6 | 0x0040 | Motor temp sensor error |
| 8 | 0x0100 | Motor stall |
| 9 | 0x0200 | Motor block |

Clear errors with 0x76 (System Reset) or 0x77 (Clear Error) CAN command.

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
| `calibrate_detection.py` | Vision calibration data collection and regression |
| `visualize_detection.py` | Detection result visualization |
| `check_motor_alarm.py` | Check motor error/alarm status (0x9A) |
| `cleanup_ros2_nodes.sh` | Clean up orphaned ROS2 processes |
| `robot_control_service.sh` | Systemd service management helper |

## Safety Features

- **Emergency stop**: Iron-MD remote E-stop button (immediate all-motor stop)
- **Limit sensors**: 7 sensors on EZI-IO for overtravel prevention (X/Y/Z min/max + Yaw home)
- **Homing interlock**: Axes must be homed before autonomous operation
- **Brake system**: Automatic brake engagement on power loss
- **Error detection**: Motor overtemp/overcurrent auto-detection and logging
- **Stroke limits**: Software-enforced stage travel limits based on measured values
- **Range filtering**: Out-of-range detection points automatically skipped during tying

## License

MIT License

## Team

- **KOCETI Robotics Team**
- GitHub: [@jino123-koceti](https://github.com/jino123-koceti)

---

**Last Updated**: 2026-03-10
**ROS2 Version**: Humble
**Platform**: Jetson AGX Orin (Ubuntu 22.04 LTS, aarch64)
