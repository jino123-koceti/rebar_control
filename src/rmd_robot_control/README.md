# RMD Robot Control Package

ROS2-based 7-motor robot control package using RMD-X4 actuators for velocity control and position control via SocketCAN.

## Overview

- **Velocity control**: 0x141, 0x142 motors as differential drive
- **Position control**: 0x143, 0x144, 0x145, 0x146, 0x147 individual joint control
- **Unified state management**: All motor states monitored in one node
- **CAN communication**: SocketCAN interface to RMD-X4 motors

## Package Structure

```
rmd_robot_control/
├── rmd_robot_control/
│   ├── can_manager.py              # CAN communication manager
│   ├── position_control_node.py    # Unified position/velocity control node
│   ├── robot_control_node.py       # Legacy integrated control node
│   ├── robot_control_gui.py        # PyQt5 GUI control interface
│   └── motor_test.py               # Motor test node
├── config/
│   └── robot_control.yaml
├── launch/
│   ├── robot_control.launch.py
│   ├── gui_control.launch.py
│   ├── cmd_vel_only.launch.py
│   └── position_only.launch.py
└── README.md
```

## Installation

```bash
sudo apt install ros-humble-joint-state-publisher python3-pyqt5
cd ~/ros2_ws
colcon build --packages-select rmd_robot_control
source install/setup.bash
```

## Usage

```bash
# Full system launch
ros2 launch rmd_robot_control robot_control.launch.py

# GUI control (requires display)
ros2 launch rmd_robot_control gui_control.launch.py

# Velocity control only
ros2 launch rmd_robot_control cmd_vel_only.launch.py

# Position control only
ros2 launch rmd_robot_control position_only.launch.py
```

### Motor Test

```bash
# Communication test (no motor movement)
ros2 run rmd_robot_control safe_motor_test --ros-args -p test_mode:=communication

# Status read test
ros2 run rmd_robot_control safe_motor_test --ros-args -p test_mode:=status_read
```

## Motor ID Mapping

| Motor ID | Control | Function |
|----------|---------|----------|
| 0x141 | Velocity (0xA2) | Left drive wheel |
| 0x142 | Velocity (0xA2) | Right drive wheel |
| 0x143 | Position (0xA4) | Lateral traverse |
| 0x144 | Velocity (0xA2) | X-axis stage |
| 0x145 | Velocity (0xA2) | Y-axis stage |
| 0x146 | Position (0xA4) | Z-axis (up/down) |
| 0x147 | Position (0xA4) | Yaw (rotation) |

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Differential drive command |
| `/joint_trajectory` | trajectory_msgs/JointTrajectory | Joint trajectory command |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | sensor_msgs/JointState | Unified joint states |
| `/motor_status` | std_msgs/Float64MultiArray | Motor status info |
| `motor_0x14X_position` | std_msgs/Float32 | Individual motor position |
| `motor_0x14X_rpm` | std_msgs/Float32 | Individual motor RPM |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can2` | CAN interface name |
| `left_motor_id` | 0x141 | Left motor CAN ID |
| `right_motor_id` | 0x142 | Right motor CAN ID |
| `wheel_radius` | 0.1 m | Wheel radius |
| `wheel_base` | 0.5 m | Wheel separation |
| `max_linear_vel` | 1.0 m/s | Max linear velocity |
| `max_angular_vel` | 1.0 rad/s | Max angular velocity |

## RMD-X4 Protocol

- **Command ID**: 0x140 + motor number
- **Response ID**: 0x240 + motor number
- **Velocity command**: 0xA2 (RPM)
- **Position command**: 0xA4 (degrees)
- **Multi-turn angle read**: 0x92 (0.01 deg/LSB)
- **Status read**: 0x9C, 0x9A

## Troubleshooting

```bash
# Activate CAN interface
sudo ip link set can2 up type can bitrate 1000000

# Monitor CAN messages
candump can2

# Filter specific motor IDs
candump can2 | grep "141\|142\|143\|144\|145\|146\|147"
```

## License

MIT
