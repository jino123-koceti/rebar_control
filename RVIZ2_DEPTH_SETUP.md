# RViz2에서 Depth 이미지 시각화 설정 가이드

## 문제 상황
RViz2에서 `/zedxmini2/zedxmini2_node/depth/depth_registered` 토픽을 표시하면 검은 화면만 보임

## 원인
ZED depth 이미지는 **32FC1 인코딩** (float32, 단위: 미터)을 사용하며, 값 범위가 보통 0.2m ~ 10m입니다.
RViz2의 기본 시각화 설정으로는 이 범위를 제대로 표시하지 못할 수 있습니다.

## 해결 방법

### 방법 1: RViz2 Image Display 설정 조정

1. **RViz2 실행**
   ```bash
   rviz2
   ```

2. **Image Display 추가**
   - 좌측 하단 "Add" 버튼 클릭
   - "By display type" → "Image" 선택
   - "OK" 클릭

3. **토픽 설정**
   - Image Display 확장
   - "Topic" → `/zedxmini2/zedxmini2_node/depth/depth_registered` 선택

4. **시각화 설정 조정** (중요!)
   - "Normalize Range" 체크박스 **활성화** ✅
   - "Min Value": `0.0` (또는 비워두기)
   - "Max Value": `3.0` (3미터까지 표시, 필요시 조정)
   - "Color Scheme":
     - `Intensity`: 흑백 (밝을수록 가까움)
     - `RGB8`: 컬러맵 사용 시

### 방법 2: 대체 토픽 사용

ZED는 여러 depth 관련 토픽을 제공합니다:

```bash
# 사용 가능한 토픽 확인
ros2 topic list | grep depth
```

추천 토픽:
- `/zedxmini2/zedxmini2_node/depth/depth_registered` - 메인 depth (32FC1)
- `/zedxmini2/zedxmini2_node/depth/depth_registered/compressed` - 압축된 버전
- `/zedxmini2/zedxmini2_node/depth/depth_registered/compressedDepth` - 압축된 depth

### 방법 3: 실시간 시각화 스크립트 사용 (권장)

OpenCV 기반 시각화 도구 사용:

```bash
cd /home/test/ros2_ws
source install/setup.bash
python3 check_depth_live.py
```

**기능:**
- 3가지 시각화 방식 동시 표시
  - Normalized: 자동 범위 조정
  - 0-3m: 고정 범위 (3m 이내)
  - Color: 컬러맵 (JET)
- 실시간 통계 표시
- 키보드 단축키:
  - `s`: 스냅샷 저장 (/tmp/depth_*.png)
  - `q`: 종료

### 방법 4: Depth 값 범위 확인

실제 depth 값 범위를 먼저 확인:

```bash
cd /home/test/ros2_ws
source install/setup.bash
python3 test_depth_values.py
```

출력 예시:
```
Depth values (meters):
  Min: 0.271m
  Max: 1.029m
  Mean: 0.650m
```

→ 이 값을 보고 RViz2의 Max Value를 적절히 조정하세요 (예: 1.5m 또는 2.0m)

## 일반적인 문제와 해결

### 1. 여전히 검은 화면이 보임
- **확인사항:**
  - "Normalize Range" 활성화 여부
  - Max Value가 너무 크게 설정되지 않았는지 (10m 이상)
  - 카메라가 실제로 물체를 보고 있는지 (유효한 depth 값이 있는지)

```bash
# Depth 토픽이 발행되고 있는지 확인
ros2 topic hz /zedxmini2/zedxmini2_node/depth/depth_registered

# 첫 메시지 확인 (경고: 출력이 매우 클 수 있음)
ros2 topic echo /zedxmini2/zedxmini2_node/depth/depth_info --once
```

### 2. 너무 밝거나 너무 어두움
- **해결:** Max Value 조정
  - 가까운 물체 위주: `1.0` ~ `2.0`
  - 넓은 범위: `3.0` ~ `5.0`

### 3. 픽셀이 거의 없음
- **원인:** ZED 카메라의 depth 추정은 텍스처가 있는 영역에서만 작동
- **해결:**
  - 카메라를 물체에 가까이 이동
  - 조명 개선
  - launch 파일에서 `depth_confidence` 값 낮추기 (현재: 50)

## ZED Depth 설정 파라미터

현재 설정 (`data_collection.launch.py`):
```python
'depth_mode': 'ULTRA',           # 품질 (NEURAL > ULTRA > QUALITY > PERFORMANCE)
'depth_confidence': '50',        # 신뢰도 (낮을수록 더 많은 픽셀, 노이즈 증가)
'depth_texture_conf': '50',      # 텍스처 신뢰도
'openni_depth_mode': 'true',     # 16-bit mm 단위 (false: float 미터)
```

신뢰도를 낮춰서 더 많은 depth 픽셀을 얻으려면:
```bash
ros2 launch rebar_vision data_collection.launch.py \
  depth_confidence:=30 \
  depth_texture_conf:=30
```

## 저장된 Depth 이미지 확인

녹화된 데이터의 depth 이미지를 확인하려면:

```bash
python3 << 'EOF'
import cv2
import numpy as np
import glob

sessions = sorted(glob.glob('/home/test/dataset/rebar_*'))
if sessions:
    latest = sessions[-1]
    depth_files = sorted(glob.glob(f'{latest}/right_camera/depth/*.png'))

    if depth_files:
        depth = cv2.imread(depth_files[0], cv2.IMREAD_UNCHANGED)

        print(f"Saved depth format: uint16 (millimeters)")
        print(f"  Min: {depth[depth>0].min()}mm")
        print(f"  Max: {depth.max()}mm")
        print(f"  Mean: {depth[depth>0].mean():.1f}mm")

        # 시각화
        depth_vis = (depth / depth.max() * 255).astype(np.uint8)
        cv2.imshow('Saved Depth', depth_vis)
        cv2.waitKey(0)
EOF
```

## 문제 해결 체크리스트

- [ ] Depth 토픽이 발행되고 있는가? (`ros2 topic hz`)
- [ ] RViz2에서 "Normalize Range" 활성화했는가?
- [ ] Max Value를 적절히 설정했는가? (1-3m)
- [ ] 카메라가 물체를 보고 있는가? (유효한 depth 값 존재)
- [ ] `check_depth_live.py`로 시각화되는가?
- [ ] ZED 카메라 런치 파일이 실행 중인가?

## 참고

- ZED depth는 스테레오 비전 기반이므로 텍스처가 없는 영역(흰 벽 등)에서는 값이 없을 수 있습니다
- 최소 거리: 약 0.2m (ZED X mini 기준)
- 최대 거리: 약 15m (조건에 따라 다름)
