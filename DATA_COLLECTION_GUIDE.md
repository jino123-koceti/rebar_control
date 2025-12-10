# 데이터 수집 빠른 시작 가이드

## ✅ Phase 1 완료 - 듀얼 카메라 녹화 노드

---

## 🎬 실행 방법

### **방법 1: 수동 캡처 (권장)**

```bash
# ============================================
# Terminal 1: 데이터 수집 노드 시작
# ============================================
cd ~/ros2_ws
source install/setup.bash

ros2 launch rebar_vision data_collection.launch.py \
    rebar_spacing:=200 \
    rebar_pattern:=orthogonal \
    session_name:=test_001

# 출력 예시:
# ============================================================
# Dual Camera Recorder Node Started
# ============================================================
# Session ID: test_001_20251203_143052
# Save Path: /home/test/dataset/test_001_20251203_143052
# Mode: manual
# Rebar Spacing: 200 mm
# Rebar Pattern: orthogonal
# Save Depth: True
# ============================================================
# Waiting for trigger...
#   ros2 topic pub /rebar/recorder/trigger std_msgs/Bool "data: true" --once


# ============================================
# Terminal 2: 수동 캡처 트리거
# ============================================
cd ~/ros2_ws
source install/setup.bash

# 위치 1: 중앙
ros2 topic pub /rebar/recorder/trigger std_msgs/Bool "data: true" --once

# 로봇 이동 (전방 100mm)

# 위치 2: 전방
ros2 topic pub /rebar/recorder/trigger std_msgs/Bool "data: true" --once

# 로봇 이동 (후방 200mm)

# 위치 3: 후방
ros2 topic pub /rebar/recorder/trigger std_msgs/Bool "data: true" --once

# ... 반복 ...
```

---

### **방법 2: 자동 캡처 (2초마다)**

```bash
ros2 launch rebar_vision data_collection.launch.py \
    save_mode:=auto \
    auto_interval:=2.0 \
    rebar_spacing:=200 \
    session_name:=auto_test

# 자동으로 2초마다 캡처됨
# 로봇을 천천히 이동시키면서 데이터 수집
```

---

## 📊 데이터 수집 계획

### **1단계: 200mm 간격 직교 배근 (기본)**

```bash
# 세션 시작
ros2 launch rebar_vision data_collection.launch.py \
    rebar_spacing:=200 \
    rebar_pattern:=orthogonal \
    session_name:=200mm_ortho

# 다양한 위치에서 80장 촬영:
# - 중앙: 10장
# - 전방 50mm: 10장
# - 전방 100mm: 10장
# - 후방 50mm: 10장
# - 후방 100mm: 10장
# - 좌측 50mm: 10장
# - 우측 50mm: 10장
# - 대각선: 10장
```

### **2단계: 150mm 간격**

```bash
# 철근 간격 조정 → 150mm

ros2 launch rebar_vision data_collection.launch.py \
    rebar_spacing:=150 \
    rebar_pattern:=orthogonal \
    session_name:=150mm_ortho

# 50장 촬영
```

### **3단계: 100mm 간격**

```bash
# 철근 간격 조정 → 100mm

ros2 launch rebar_vision data_collection.launch.py \
    rebar_spacing:=100 \
    rebar_pattern:=orthogonal \
    session_name:=100mm_ortho

# 50장 촬영
```

### **4단계: 사선 배근**

```bash
# 철근을 45도 각도로 배치

ros2 launch rebar_vision data_collection.launch.py \
    rebar_spacing:=200 \
    rebar_pattern:=diagonal \
    session_name:=200mm_diagonal

# 40장 촬영
```

---

## 📁 데이터 확인

```bash
# 저장된 데이터 확인
cd /home/test/dataset

# 최신 세션 확인
ls -lht | head

# 세션 내용 확인
cd <session_name>
tree

# 출력 예시:
# .
# ├── left_camera
# │   ├── rgb
# │   │   ├── frame_0000.png
# │   │   ├── frame_0001.png
# │   │   └── ...
# │   ├── depth
# │   │   ├── frame_0000.png
# │   │   └── ...
# │   └── camera_info.json
# ├── right_camera
# │   ├── rgb
# │   ├── depth
# │   └── camera_info.json
# └── metadata
#     ├── session_info.json
#     └── frame_info.json

# 이미지 개수 확인
echo "Left RGB: $(ls left_camera/rgb/ | wc -l)"
echo "Right RGB: $(ls right_camera/rgb/ | wc -l)"

# 메타데이터 확인
cat metadata/session_info.json | jq
```

---

## 🔍 데이터 품질 체크

### **이미지 미리보기 (eog 또는 ristretto)**

```bash
# RGB 이미지 확인
eog left_camera/rgb/frame_0000.png &
eog right_camera/rgb/frame_0000.png &

# 확인 사항:
# ✅ 교차점이 6개 모두 보이는가?
# ✅ 초점이 맞는가? (블러 없음)
# ✅ 밝기가 적절한가? (너무 어둡거나 밝지 않음)
# ✅ 철근이 화면 밖으로 나가지 않는가?
```

### **Depth 이미지 확인**

```bash
# Python으로 Depth 시각화
python3 << EOF
import cv2
import numpy as np

depth_left = cv2.imread('left_camera/depth/frame_0000.png', cv2.IMREAD_ANYDEPTH)
depth_right = cv2.imread('right_camera/depth/frame_0000.png', cv2.IMREAD_ANYDEPTH)

print(f"Left depth range: {depth_left.min()} - {depth_left.max()} mm")
print(f"Right depth range: {depth_right.min()} - {depth_right.max()} mm")

# 시각화
depth_viz = cv2.normalize(depth_left, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
cv2.imwrite('depth_viz.png', depth_viz)
print("Saved depth_viz.png")
EOF

eog depth_viz.png
```

---

## 🎯 다음 단계

### **데이터 정리 (라벨링 준비)**

```bash
# 스크립트 작성 예정 (Phase 1.3)
# organize_dataset.py 실행 시:
#
# 입력: /home/test/dataset/session_folders/
# 출력: 
#   dataset/
#   ├─ images/
#   │  ├─ left_0000.png
#   │  ├─ right_0000.png
#   │  └─ ...
#   └─ labels/  (라벨링 후 생성)
```

---

## 💡 팁 & 트릭

### **빠른 캡처 (별칭 설정)**

```bash
# ~/.bashrc에 추가
alias capture='ros2 topic pub /rebar/recorder/trigger std_msgs/Bool "data: true" --once'

# 사용
source ~/.bashrc
capture  # 한 번에 캡처!
```

### **저장 공간 확인**

```bash
# 디스크 용량 확인
df -h /home/test/dataset

# 예상 용량:
# - RGB (1280x720 PNG): ~0.5 MB/장
# - Depth (16-bit): ~1.8 MB/장
# - 좌우 합계: ~4.6 MB/프레임
# - 250 프레임: ~1.15 GB
```

### **백업**

```bash
# 외장 SSD에 백업
rsync -avh --progress /home/test/dataset/ /media/usb/rebar_dataset_backup/
```

---

## ⚠️ 주의사항

1. **ZED 카메라 워밍업**
   - 첫 10-20장은 화질이 불안정할 수 있음
   - 몇 장 버리고 실제 수집 시작 권장

2. **조명 일관성**
   - 같은 세션 내에서는 조명 조건 유지
   - 조명 변화는 별도 세션으로 분리

3. **로봇 안정화**
   - 로봇 이동 후 1-2초 대기
   - 진동이 멈춘 후 캡처

4. **철근 배치 확인**
   - 교차점이 화면 중앙에 위치하도록
   - 6개 교차점 모두 보이는지 확인

---

## 📞 문제 해결

### ZED 노드가 시작되지 않음

```bash
# ZED 드라이버 확인
ls /dev/video*

# ZED SDK 버전 확인
/usr/local/zed/tools/ZED_Explorer

# 수동 실행 테스트
ros2 launch zed_wrapper two_zedxmini.launch.py
```

### 토픽이 보이지 않음

```bash
# 토픽 리스트 확인
ros2 topic list | grep zedxmini

# 토픽 Hz 확인
ros2 topic hz /zedxmini1/zed_node/rgb/image_rect_color
```

### 저장이 안 됨

```bash
# 권한 확인
ls -l /home/test/dataset

# 로그 확인
ros2 run rebar_vision dual_camera_recorder --ros-args --log-level debug
```

---

**작성일**: 2025-12-03  
**Phase**: 1.1 완료 ✅
