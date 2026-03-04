# Rebar Vision - Rebar Crossing Detection System

YOLO-based rebar crossing detection and 3D coordinate transformation for automated tying.

## Package Components

### Nodes

| Node | Description |
|------|-------------|
| `rebar_detection_node.py` | YOLO inference + depth sampling + 3D coordinate transform |
| `tying_orchestrator_node.py` | Automated tying state machine: detect, move stage, tie |
| `dual_camera_recorder_node.py` | Dual ZED X Mini synchronized data collection |

### Detection Strategies

The detection node supports three strategies via the `detection_strategy` parameter:

- **`cross`** (default): Left camera detects far-side (Y-) points, right camera detects far-side (Y+) points. Avoids depth minimum distance issues and tool occlusion.
- **`merge`**: Both cameras detect all points, results merged with duplicate removal (20 mm threshold).
- **`single`**: Single camera only.

### Detection Pipeline

```
1. Grab cached latest frames (RGB + Depth) from dual ZED cameras
2. YOLO inference -> bounding boxes + confidence scores
3. Extract center pixel (u, v) of each detection
4. Sample depth from depth image (5x5 median filter)
5. Pixel to camera 3D: X_cam = (u-cx)*Z/fx, Y_cam = (v-cy)*Z/fy
6. Camera to robot frame: R(pitch, roll, yaw) @ P_cam + translation
7. Merge dual camera results (duplicate removal within 20 mm)
8. Sort into 2x3 grid: split by X into 2 rows, sort each row by Y
```

### Tying Orchestrator State Machine

```
IDLE -> DETECTING -> MOVING_TO_POINT -> WAITING_STAGE
  -> EXECUTING_S21 -> WAITING_S21
  -> EXECUTING_S22 -> WAITING_S22
  -> ADVANCING -> (next point or COMPLETE)
  -> COMPLETE -> IDLE
```

Triggered when `/control_mode` changes to `"tying"`.

---

## Installation

```bash
cd ~/ros2_ws
pip3 install ultralytics  # YOLO
colcon build --packages-select rebar_vision
source install/setup.bash
```

---

## Usage

### Data Collection

```bash
# Launch dual camera data collection
ros2 launch rebar_vision data_collection.launch.py

# Manual capture trigger
ros2 topic pub /rebar/recorder/trigger std_msgs/Bool "data: true" --once

# Auto capture (every 2 seconds)
ros2 launch rebar_vision data_collection.launch.py \
    save_mode:=auto auto_interval:=2.0
```

### Vision-Guided Tying

```bash
# Launch detection + orchestrator
ros2 launch rebar_vision vision_tying.launch.py

# Manual detection test
ros2 service call /rebar/detect_crossings \
    rebar_base_interfaces/srv/DetectCrossings \
    "{camera_selection: 0, confidence_threshold: 0.3, expected_count: 6}"

# View detection overlay image
ros2 topic echo /rebar/detection_image
```

---

## Configuration

### Camera Extrinsics: `config/camera_extrinsics.yaml`

| Camera | Position (mm) | Pitch | Roll | Yaw |
|--------|---------------|-------|------|-----|
| Left (zedxmini1) | [-200, +100, 108] | 40 deg | 0 deg | -20 deg |
| Right (zedxmini2) | [-200, -100, 108] | 40 deg | 0 deg | +20 deg |

### Tying Orchestrator: `config/tying_orchestrator.yaml`

| Parameter | Value | Description |
|-----------|-------|-------------|
| `deg_per_mm_x` | 2.698 | X-axis: 1133.29 deg / 420 mm (measured) |
| `deg_per_mm_y` | 2.677 | Y-axis: 803.09 deg / 300 mm (measured) |
| `stage_max_speed_dps` | 200.0 | Maximum stage speed (deg/s) |
| `max_stage_x_mm` | 420.0 | X-axis stroke (measured) |
| `max_stage_y_mm` | 300.0 | Y-axis stroke (measured) |
| `max_yaw_deg` | 243.0 | Yaw rotation range (measured) |
| `position_tolerance_deg` | 5.0 | Position tolerance (~1.9 mm) |
| `stage_settling_time` | 1.5 | Post-move settling time (seconds) |

---

## ROS2 API

### Topics (Subscribed)

| Topic | Type | Description |
|-------|------|-------------|
| `/zedxmini1/zed_node/rgb/image_rect_color` | sensor_msgs/Image | Left camera RGB |
| `/zedxmini1/zed_node/depth/depth_registered` | sensor_msgs/Image | Left camera depth |
| `/zedxmini2/zed_node/rgb/image_rect_color` | sensor_msgs/Image | Right camera RGB |
| `/zedxmini2/zed_node/depth/depth_registered` | sensor_msgs/Image | Right camera depth |
| `/control_mode` | std_msgs/String | Mode switch trigger for orchestrator |
| `/sequence_status` | std_msgs/String | Tying sequence completion status |

### Topics (Published)

| Topic | Type | Description |
|-------|------|-------------|
| `/rebar/detection_image` | sensor_msgs/Image | Debug overlay with detections |
| `/rebar/detected_grid` | RebarGrid | Detected 2x3 crossing grid |
| `/joint_control` | JointControl | Stage motor commands (0x144, 0x145) |
| `/sequence_cmd` | std_msgs/String | Auto S21/S22 trigger |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/rebar/detect_crossings` | DetectCrossings | Request crossing detection |

---

## Data Storage Structure

```
/home/test/dataset/rebar_YYYYMMDD_HHMMSS/
├── left_camera/
│   ├── rgb/          # PNG images
│   ├── depth/        # 16-bit PNG (uint16, millimeters)
│   └── camera_info.json
├── right_camera/
│   ├── rgb/
│   ├── depth/
│   └── camera_info.json
└── metadata/
    ├── session_info.json
    └── frame_info.json
```

### Depth Image Format

- **Format**: 16-bit PNG (uint16)
- **Unit**: Millimeters
- **Range**: 100 mm to 8000 mm (camera), 0 to 65535 mm (storage)
- **Invalid value**: 0 (measurement failed / out of range)

```python
import cv2
depth = cv2.imread('frame_0000.png', cv2.IMREAD_UNCHANGED)  # uint16
distance_mm = depth[y, x]
distance_m = distance_mm / 1000.0
```

---

## Hardware

### ZED X Mini Cameras

| Spec | Value |
|------|-------|
| Model | Stereolabs ZED X Mini |
| Resolution | 1280x720 (HD720) @ 15 FPS |
| Depth range | 0.1 m to 8.0 m |
| Depth mode | ULTRA |
| FOV | 90 deg (H) x 60 deg (V) |

### Camera Mounting (Inward V-Configuration)

```
     ┌─────────────────┐
     │   Robot Body     │
     └─────────────────┘
           │
    ┌──────┴──────┐
    │             │
 [CAM1]        [CAM2]
 (Left)       (Right)
  ↘ 40°      ↙ 40°
    ╲       ╱
     ╲  ▼  ╱   <- Rebar
      ╲   ╱
       ╲ ╱
```

---

## Troubleshooting

### ZED cameras not detected

```bash
ros2 topic list | grep zedxmini
# If empty, restart ZED nodes
ros2 launch zed_wrapper two_zedxmini.launch.py
```

### Sync errors (4-topic timestamp mismatch)

Adjust `slop` parameter in `dual_camera_recorder_node.py` (default: 50 ms).

### Depth images appear black

16-bit PNGs don't display properly in standard image viewers. Use:
```bash
python3 -c "
import cv2
depth = cv2.imread('frame_0000.png', cv2.IMREAD_UNCHANGED)
print(f'Range: {depth[depth>0].min()}-{depth.max()} mm')
"
```

---

**Last Updated**: 2026-03-04
**Version**: 0.2.0 (Detection + Orchestrator)
