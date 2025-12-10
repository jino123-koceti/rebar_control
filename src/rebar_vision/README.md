# Rebar Vision - 철근 교차점 인식 시스템

YOLO 기반 철근 배근 교차점 검출 및 3D 좌표 변환 시스템

## 📦 패키지 구성

### 현재 Phase: **Phase 1 - 데이터 수집** ✅

- `dual_camera_recorder_node.py`: 듀얼 ZED X mini 카메라 데이터 수집 노드

### 향후 개발 예정

- `rebar_detector_node.py`: YOLO 기반 교차점 검출
- `rebar_coordinate_transformer_node.py`: 2D→3D 좌표 변환
- `rebar_fusion_node.py`: 좌우 카메라 검출 결과 융합

---

## 🚀 설치

```bash
cd ~/ros2_ws
colcon build --packages-select rebar_vision
source install/setup.bash
```

---

## 📊 데이터 수집 사용법

### 1. 수동 캡처 모드 (기본)

```bash
# 1. 런치 실행
ros2 launch rebar_vision data_collection.launch.py

# 2. 로봇을 원하는 위치에 배치

# 3. 캡처 트리거 (별도 터미널)
ros2 topic pub /rebar/recorder/trigger std_msgs/Bool "data: true" --once

# 4. 반복: 로봇 이동 → 캡처
# ...

# 5. 종료 (Ctrl+C)
# 자동으로 session_info.json 저장됨
```

### 2. 자동 캡처 모드 (N초마다)

```bash
# 2초마다 자동 캡처
ros2 launch rebar_vision data_collection.launch.py \
    save_mode:=auto \
    auto_interval:=2.0 \
    rebar_spacing:=200 \
    rebar_pattern:=orthogonal
```

### 3. 연속 녹화 모드 (N FPS)

```bash
# 10 FPS로 연속 녹화
ros2 launch rebar_vision data_collection.launch.py \
    save_mode:=continuous \
    fps_limit:=10.0 \
    rebar_spacing:=200
```

### 4. 커스텀 세션 이름

```bash
ros2 launch rebar_vision data_collection.launch.py \
    session_name:=test_100mm_diagonal \
    rebar_spacing:=100 \
    rebar_pattern:=diagonal
```

---

## 📁 저장 구조

```
/home/test/dataset/rebar_20251203_143052/
├─ left_camera/
│  ├─ rgb/
│  │  ├─ frame_0000.png
│  │  ├─ frame_0001.png
│  │  └─ ...
│  ├─ depth/
│  │  ├─ frame_0000.png (16-bit, uint16, millimeters)
│  │  └─ ...
│  └─ camera_info.json
│
├─ right_camera/
│  ├─ rgb/
│  ├─ depth/
│  └─ camera_info.json
│
└─ metadata/
   ├─ session_info.json
   └─ frame_info.json
```

### Depth 이미지 포맷

- **형식**: 16-bit PNG (uint16)
- **단위**: 밀리미터(mm)
- **범위**:
  - 카메라: 100mm ~ 8000mm (0.1m ~ 8.0m)
  - 저장: 0 ~ 65535mm (0 ~ 65.535m)
- **변환**: ZED 32FC1 (미터) → uint16 (밀리미터)
- **무효값**: 0 (측정 불가/범위 밖)

```python
# Depth 이미지 읽기 예시
import cv2
import numpy as np
import json

# 1. Depth 이미지 로드
depth = cv2.imread('frame_0000.png', cv2.IMREAD_UNCHANGED)  # uint16
distance_mm = depth[y, x]  # 밀리미터 단위
distance_m = distance_mm / 1000.0  # 미터 단위

# 2. 카메라 정보 로드
with open('camera_info.json', 'r') as f:
    cam_info = json.load(f)

# 3. 픽셀 좌표 → 3D 좌표 변환
def pixel_to_3d(u, v, depth_mm, cam_info):
    fx = cam_info['camera_matrix']['fx']
    fy = cam_info['camera_matrix']['fy']
    cx = cam_info['camera_matrix']['cx']
    cy = cam_info['camera_matrix']['cy']

    depth_m = depth_mm / 1000.0
    X = (u - cx) * depth_m / fx
    Y = (v - cy) * depth_m / fy
    Z = depth_m
    return X, Y, Z

# 예: 픽셀 (480, 300)의 3D 좌표
X, Y, Z = pixel_to_3d(480, 300, depth[300, 480], cam_info)
print(f"3D: ({X:.3f}, {Y:.3f}, {Z:.3f}) meters")
```

**완전한 예제**: `depth_to_3d_example.py` 참고

### session_info.json 예시

```json
{
  "session_id": "rebar_20251203_143052",
  "total_frames": 85,
  "rebar_config": {
    "spacing": 200,
    "pattern": "orthogonal",
    "crossing_points": 6
  },
  "camera_setup": {
    "left": {
      "serial_number": "56755054",
      "position": [-200, 100, 108],
      "rotation": [40, 0, -20],
      "facing": "right_inward"
    },
    "right": {
      "serial_number": "54946194",
      "position": [-200, -100, 108],
      "rotation": [40, 0, 20],
      "facing": "left_inward"
    }
  }
}
```

---

## 🎮 ROS2 API

### Topics (구독)

| Topic | Type | Description |
|-------|------|-------------|
| `/zedxmini1/zed_node/rgb/image_rect_color` | sensor_msgs/Image | 좌측 카메라 RGB |
| `/zedxmini1/zed_node/depth/depth_registered` | sensor_msgs/Image | 좌측 카메라 Depth |
| `/zedxmini2/zed_node/rgb/image_rect_color` | sensor_msgs/Image | 우측 카메라 RGB |
| `/zedxmini2/zed_node/depth/depth_registered` | sensor_msgs/Image | 우측 카메라 Depth |
| `/rebar/recorder/trigger` | std_msgs/Bool | 수동 캡처 트리거 |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/rebar/recorder/start` | std_srvs/Trigger | 녹화 시작 |
| `/rebar/recorder/stop` | std_srvs/Trigger | 녹화 중지 |
| `/rebar/recorder/save_session` | std_srvs/Trigger | 세션 정보 저장 |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `save_path` | string | `/home/test/dataset` | 저장 경로 |
| `save_mode` | string | `manual` | 캡처 모드 (manual/auto/continuous) |
| `auto_interval` | double | 2.0 | 자동 캡처 간격 (초) |
| `fps_limit` | double | 10.0 | 연속 모드 FPS |
| `rebar_spacing` | int | 200 | 철근 간격 (mm) |
| `rebar_pattern` | string | `orthogonal` | 배근 패턴 (orthogonal/diagonal) |
| `session_name` | string | `` | 커스텀 세션 이름 |
| `save_depth` | bool | true | Depth 이미지 저장 여부 |

---

## 📷 하드웨어 사양

### ZED X Mini 카메라

| 항목 | 사양 |
|------|------|
| 모델 | Stereolabs ZED X Mini |
| 해상도 | 1280×720 (HD720) @ 15 FPS |
| Depth 범위 | **0.1m ~ 8.0m** (100mm ~ 8000mm) |
| Depth 모드 | ULTRA (launch 파라미터) |
| FOV | 90° (H) × 60° (V) |

### 카메라 배치 (대향 설치)

```
     ┌─────────────────┐
     │   Robot Body    │
     └─────────────────┘
           │
    ┌──────┴──────┐
    │             │
[CAM1]          [CAM2]
(Left)         (Right)
↘ 40°         ↙ 40°
    ╲       ╱
     ╲  ▼  ╱  <- 철근
      ╲   ╱
       ╲ ╱
```

---

## 🔧 트러블슈팅

### ZED 카메라가 인식되지 않음

```bash
# ZED 토픽 확인
ros2 topic list | grep zedxmini

# 없으면 ZED 노드 재시작
ros2 launch zed_wrapper two_zedxmini.launch.py
```

### 저장 경로 권한 오류

```bash
sudo mkdir -p /home/test/dataset
sudo chown -R $USER:$USER /home/test/dataset
```

### 동기화 오류 (4개 토픽 타임스탬프 불일치)

```bash
# slop 파라미터 조정 (dual_camera_recorder_node.py)
# 현재: slop=0.05 (50ms)
# 증가: slop=0.1 (100ms)
```

### Depth 이미지가 검은색으로만 보임

**원인**: 16-bit PNG는 일반 이미지 뷰어에서 제대로 표시되지 않습니다.

**해결**:
- `*_visualization.png` 파일 확인 (자동 생성되는 컬러 이미지)
- 또는 Python으로 확인:

```bash
# 시각화 도구 사용
python3 /home/test/ros2_ws/view_depth_image.py frame_0000.png

# 또는 데이터 확인
python3 -c "
import cv2
depth = cv2.imread('frame_0000.png', cv2.IMREAD_UNCHANGED)
print(f'Range: {depth[depth>0].min()}-{depth.max()}mm')
"
```

### Depth 데이터 활용 방법

**완전한 예제 실행**:
```bash
python3 /home/test/ros2_ws/depth_to_3d_example.py
```

**주요 활용 사례**:
1. **YOLO 검출 → 3D 좌표**: 철근 교차점 픽셀 → 실제 3D 위치
2. **Point Cloud 생성**: 전체 장면의 3D 모델
3. **거리 측정**: 두 교차점 사이의 실제 거리
4. **정확도**: 1mm 단위 (16-bit uint16 저장)

---

## 📈 데이터 수집 체크리스트

### 필수 조건별 데이터

- [ ] 200mm 간격 직교 (80장)
- [ ] 150mm 간격 직교 (50장)
- [ ] 100mm 간격 직교 (50장)
- [ ] 사선 배근 45° (40장)
- [ ] 다양한 조명 (30장)

**총 목표: 250장 × 2 (좌우) = 500장**

### 로봇 위치 변화

각 조건마다 5-10 포지션:
- 중앙
- 전방 ±100mm
- 후방 ±100mm
- 좌우 ±50mm

---

## 📝 다음 단계

1. **데이터 수집 완료** (현재)
2. **데이터 정리** - 이미지 이름 변경 및 디렉토리 구조 정리
3. **라벨링** - Roboflow로 교차점 바운딩 박스
4. **YOLO 학습** - YOLOv8n 모델 학습
5. **ROS2 인식 노드 개발** - 실시간 검출 및 좌표 변환

---

## 📚 참고 문서

- [REBAR_VISION_DEVELOPMENT_PLAN.md](../../REBAR_VISION_DEVELOPMENT_PLAN.md) - 전체 개발 계획
- [ZED SDK Documentation](https://www.stereolabs.com/docs)
- [YOLOv8 Documentation](https://docs.ultralytics.com)

---

**작성일**: 2025-12-03  
**버전**: 0.1.0 (Phase 1)
