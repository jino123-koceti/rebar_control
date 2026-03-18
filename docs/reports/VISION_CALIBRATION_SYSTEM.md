# Vision-Based Rebar Intersection Detection and Coordinate Calibration System

## 1. System Overview

본 시스템은 철근 배근 자동 결속 로봇에서 **스테레오 카메라(ZED X Mini)와 딥러닝 객체 검출(YOLOv8)**을 결합하여 철근 교차점을 자동으로 검출하고, **경험적 다중 선형회귀 모델**을 통해 검출된 픽셀 좌표를 로봇 작업 좌표계(EEF_origin)로 변환하는 비전 캘리브레이션 시스템이다.

### 1.1 목적

- 카메라 픽셀 좌표(pixel_u, pixel_v)와 깊이(depth_mm)로부터 로봇 EEF 원점 기준의 물리적 좌표(X, Y mm)를 산출
- 산출된 좌표를 기반으로 스테이지(X/Y/Z/Yaw)를 자동 이동하여 결속 작업 수행
- 캘리브레이션 데이터 수집 → 회귀 모델 학습 → 실시간 좌표 변환의 전체 파이프라인 구축

### 1.2 시스템 구성

```
[ZED X Mini 카메라] → [YOLO 교차점 검출] → [픽셀 좌표 + 깊이 추출]
                                                     ↓
                                          [캘리브레이션 매핑 모델]
                                                     ↓
                                          [EEF 기준 X,Y 좌표 (mm)]
                                                     ↓
                                          [스테이지 이동 명령 생성]
```

---

## 2. Hardware Configuration

### 2.1 Camera System

| 항목 | 사양 |
|------|------|
| 카메라 모델 | Stereolabs ZED X Mini (zedxm) |
| 우측 카메라 (zedxmini2) | SN: 54946194, P1-P3 검출용 |
| 좌측 카메라 (zedxmini1) | SN: 56755054, P4-P6 검출용 |
| 해상도 | HD720 (1280×720) |
| 프레임레이트 | 15 fps |
| 깊이 센싱 | Stereo depth (depth_registered) |

### 2.2 Camera Mounting (Extrinsic Parameters)

카메라는 로봇 EEF(End-Effector Frame)에 **고정 장착**되어 있으며, 카메라-EEF 간의 기하학적 관계는 불변이다.

**좌표계 정의:**
- 로봇 프레임: X=전진(forward), Y=우측(right), Z=상승(up)
- EEF_origin = 스테이지 홈 위치(Xmin, Ymin)의 tool tip 위치

**우측 카메라 (zedxmini2):**
- 위치: [150.0, 370.0, -40.0] mm (EEF 기준)
- 회전: Pitch=40° (하방), Yaw=0°
- Y- 방향 관측 (철근 배근 우측면)

**좌측 카메라 (zedxmini1):**
- 위치: [-200.0, 100.0, 108.0] mm (EEF 기준)
- 회전: Pitch=40° (하방), Yaw=0°
- Y+ 방향 관측 (철근 배근 좌측면)

### 2.3 Stage Motor System

| 축 | Motor ID | deg_per_mm | 비고 |
|----|----------|-----------|------|
| X | 0x144 | 4.497 | P2P 실측 (2025-12-18) |
| Y | 0x145 | 4.462 | P2P 실측 (2025-12-18) |
| Z | 0x146 | 13.45 | 100mm 명령 → 20mm 이동, ×5 감속 |
| Yaw | 0x147 | - | 회전축 (deg 직접 제어) |

---

## 3. Detection Pipeline

### 3.1 YOLO Object Detection

- **모델:** YOLOv8 커스텀 학습 모델 (`best.pt`)
- **검출 대상:** 철근 교차점 (rebar crossing intersection)
- **Confidence Threshold:** 0.3 (조정 가능, 서비스 요청 시 지정)
- **Expected Count:** 6 (카메라당 최대 6개 교차점)

**검출 프로세스:**
1. ZED 카메라에서 RGB 이미지 수신 (`/zedxmini2/zed_node/left/image_rect_color`)
2. YOLO 모델 추론 → 바운딩 박스 + confidence 획득
3. 각 바운딩 박스 중심점(cx, cy) 계산
4. Expected count 초과 시 confidence 상위 N개만 유지

### 3.2 Depth Acquisition

- **소스:** ZED depth_registered 토픽 (`/zedxmini2/zed_node/depth/depth_registered`)
- **방법:** 검출 중심점 주변 5×5 픽셀 영역의 **중앙값 필터링(Median Filter)**
- **유효 범위:** 0.05m ~ 2.0m (범위 밖 값 제외)
- **무효 처리:** NaN, Inf 값 필터링 후 유효 깊이가 없으면 해당 검출 제외

```python
kernel_size = 5  # 5×5 window
half = kernel_size // 2
region = depth_image[v-half:v+half+1, u-half:u+half+1]
valid = region[(region > min_depth) & (region < max_depth) & np.isfinite(region)]
depth_m = np.median(valid)
```

### 3.3 Pixel → Camera 3D Coordinate

카메라 내부 파라미터(intrinsic)를 사용하여 픽셀 좌표를 카메라 3D 좌표로 변환:

```
x_cam = (u - cx) × depth_m / fx
y_cam = (v - cy) × depth_m / fy
z_cam = depth_m
```

- `(fx, fy)`: 카메라 focal length (CameraInfo 토픽에서 수신)
- `(cx, cy)`: 주점(principal point)

### 3.4 Camera 3D → Robot Frame (Extrinsic Transform)

카메라 장착 파라미터를 기반으로 회전 행렬 + 평행 이동 적용:

```
point_robot = R_full × point_cam_mm + t_cam

R_full = Rz(yaw) × T_base × Rx(pitch)

우측 카메라 T_base:
  [[-1,  0,  0],
   [ 0,  0, -1],
   [ 0, -1,  0]]
```

이 과정의 출력이 `detected_x`, `detected_y` (mm) 값이며, 이는 **카메라 extrinsic 파라미터에 의존하는 근사값**이다.

---

## 4. Calibration System

### 4.1 문제 정의

카메라 extrinsic 변환으로 계산된 `detected_x/y`는 장착 오차, 렌즈 왜곡, 깊이 노이즈 등으로 인해 실제 물리적 좌표와 차이가 발생한다. 이를 보정하기 위해 **경험적 캘리브레이션 매핑**을 적용한다.

핵심 관찰: 카메라가 EEF에 고정되어 있으므로, **픽셀 좌표 + 깊이 → EEF 기준 물리 좌표** 매핑은 로봇 위치나 철근 배근 위치와 무관하게 일정하다.

### 4.2 데이터 수집

**도구:** `calibrate_detection.py` (대화형 스크립트)

**수집 과정:**
1. 검출 서비스 호출 → 검출된 교차점 리스트 표시
2. 각 교차점에 대해 실측값(cm) 수동 입력 → mm 변환 후 저장
3. `calibration_data.json`에 누적 저장

**저장 데이터 형식:**
```json
{
  "detected_x": 266.52,      // extrinsic 변환 결과 (mm)
  "detected_y": 49.79,
  "actual_x_mm": 38.0,       // 실측 ground truth (mm)
  "actual_y_mm": 57.0,
  "depth_mm": 332.7,         // ZED 깊이값 (mm)
  "pixel_u": 361,            // 이미지 수평 좌표 (px)
  "pixel_v": 190,            // 이미지 수직 좌표 (px)
  "confidence": 0.606        // YOLO confidence
}
```

**데이터셋 통계 (최종):**
| 항목 | 범위 |
|------|------|
| 총 데이터 쌍 | 78개 (이상치 6개 제거 후) |
| pixel_u | 315 ~ 730 px |
| pixel_v | 171 ~ 206 px |
| depth_mm | 300 ~ 481 mm |
| actual_x_mm | -50 ~ 392 mm (range=442mm) |
| actual_y_mm | 0 ~ 97 mm (range=97mm) |

### 4.3 Regression Model

**방법:** 다중 선형회귀 + 교차항 (Multiple Linear Regression with Cross Terms)

**입력 특징 벡터 (6차원):**
```
F = [pixel_u, pixel_v, depth_mm, pixel_u×depth_mm, pixel_v×depth_mm, 1]
```

**출력:**
```
X_mm = F · x_coeffs
Y_mm = F · y_coeffs
```

**교차항의 물리적 의미:**

`pixel_u × depth` 및 `pixel_v × depth` 교차항은 **원근 왜곡(perspective distortion)**을 보정한다. 동일한 물리적 거리가 깊이에 따라 다른 픽셀 크기로 매핑되므로, 깊이와 픽셀 좌표의 곱 형태가 필요하다. 이는 핀홀 카메라 모델에서 `x_physical = pixel × depth / focal_length` 관계에서 자연스럽게 유도된다.

**풀이 방법:**
```python
# Numpy 최소자승법 (closed-form solution)
# A: (N×6) 설계 행렬, y: (N,) 실측값
coeffs, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
```

이는 정규방정식 `A^T A x = A^T y`의 해를 구하는 것으로, 학습/반복 과정 없이 행렬 연산 한 번으로 계산된다.

### 4.4 Model Coefficients (최종)

**X-Axis Mapping (R² = 0.9965):**
```
X = -0.079915×u + 2.763728×v - 0.116351×d + 0.003011×u×d - 0.007761×v×d - 295.043354
```

| 계수 | 값 | 입력 |
|------|-----|------|
| a₁ | -0.079915 | pixel_u |
| a₂ | 2.763728 | pixel_v |
| a₃ | -0.116351 | depth_mm |
| a₄ | 0.003011 | pixel_u × depth_mm |
| a₅ | -0.007761 | pixel_v × depth_mm |
| a₆ | -295.043354 | intercept |

**Y-Axis Mapping (R² = 0.9811):**
```
Y = -0.070021×u + 2.981924×v - 0.129224×d + 0.000140×u×d + 0.000406×v×d - 484.171851
```

| 계수 | 값 | 입력 |
|------|-----|------|
| a₁ | -0.070021 | pixel_u |
| a₂ | 2.981924 | pixel_v |
| a₃ | -0.129224 | depth_mm |
| a₄ | 0.000140 | pixel_u × depth_mm |
| a₅ | 0.000406 | pixel_v × depth_mm |
| a₆ | -484.171851 | intercept |

### 4.5 Accuracy Evaluation

**최종 모델 성능 (78쌍, 이상치 제거 후):**

| 지표 | X축 | Y축 | 총합(유클리드) |
|------|-----|-----|--------------|
| R² | 0.9965 | 0.9811 | - |
| 평균 오차 | 5.2 mm | 3.0 mm | 6.7 mm |
| 최대 오차 | 13.2 mm | 14.7 mm | 15.1 mm |
| 중앙값 | - | - | 6.5 mm |
| 90th percentile | - | - | 11.4 mm |

**오차 분포:**

| 범위 | 비율 |
|------|------|
| < 5 mm | 27% (21/78) |
| < 10 mm | 81% (63/78) |
| < 15 mm | 99% (77/78) |
| < 20 mm | 100% (78/78) |

**이상치 제거 전후 비교:**

| 지표 | 84쌍 (제거 전) | 78쌍 (제거 후) | 개선 |
|------|---------------|---------------|------|
| X R² | 0.9954 | 0.9965 | +0.0011 |
| Y R² | 0.9597 | 0.9811 | +0.0214 |
| 평균 오차 | 7.9 mm | 6.7 mm | -15% |
| 최대 오차 | 33.9 mm | 15.1 mm | -55% |

---

## 5. Detection Deduplication

### 5.1 문제

Expected count를 6으로 설정하면 동일한 물리적 교차점에 대해 여러 검출 결과가 생성될 수 있다(NMS 후에도 근접한 별개 검출). 이를 그대로 사용하면 중복 결속 시도가 발생한다.

### 5.2 Algorithm

**캘리브레이션 좌표 기반 거리 클러스터링:**

1. 모든 검출 포인트를 캘리브레이션 모델로 매핑 (pixel → mm)
2. 매핑된 좌표 공간에서 유클리드 거리 20mm 이내인 포인트를 그룹화 (Greedy Clustering)
3. 각 그룹에서:
   - 좌표: 그룹 내 매핑 좌표의 **평균**
   - 대표 검출: 최고 confidence 검출을 선택
   - 그룹 크기(avg:N)를 메타데이터로 보존

```python
def dedup_detections(detections, x_coeffs, y_coeffs, dist_mm=20.0):
    # 1. 캘리브레이션 매핑 적용
    mapped = [(apply_mapping(det.pixel_u, det.pixel_v, det.depth_mm))
              for det in detections]

    # 2. Greedy clustering (O(n²))
    used = set()
    groups = []
    for i in range(len(detections)):
        if i in used:
            continue
        group = [i]
        used.add(i)
        for j in range(i+1, len(detections)):
            if j not in used and euclidean(mapped[i], mapped[j]) < dist_mm:
                group.append(j)
                used.add(j)
        groups.append(group)

    # 3. 그룹 대표 선택
    results = []
    for group in groups:
        avg_x = mean(mapped[i].x for i in group)
        avg_y = mean(mapped[i].y for i in group)
        best = max(group, key=lambda i: detections[i].confidence)
        results.append((detections[best], avg_x, avg_y, len(group)))

    return results
```

**거리 임계값:** 20mm (철근 간격 200mm 대비 10%, 실측 정밀도 대비 충분한 마진)

---

## 6. Software Architecture

### 6.1 File Structure

```
ros2_ws/
├── calibrate_detection.py          # 대화형 캘리브레이션 데이터 수집 스크립트
├── visualize_detection.py          # 검출 결과 시각화 + 캘리브레이션 오버레이
├── calibration_data.json           # 수집된 캘리브레이션 데이터 (78쌍)
├── calibration_result.yaml         # 회귀 모델 계수 및 정확도 지표
└── src/
    ├── rebar_vision/
    │   ├── rebar_vision/
    │   │   └── rebar_detection_node.py   # YOLO 검출 서비스 노드
    │   └── config/
    │       └── camera_extrinsics.yaml    # 카메라 장착 파라미터
    └── rebar_base_control/
        └── config/
            └── can_devices.yaml          # 모터 제어 파라미터 (deg_per_mm)
```

### 6.2 ROS2 Service Interface

**서비스:** `/rebar/detect_crossings` (DetectCrossings.srv)

**Request:**
- `camera_selection`: 1(좌), 2(우), 3(양쪽)
- `confidence_threshold`: YOLO 신뢰도 임계값 (0.0이면 노드 기본값 사용)
- `expected_count`: 기대 검출 수

**Response:**
- `grid.detections[]`: 검출 리스트
  - `x`, `y`: extrinsic 변환 좌표 (mm)
  - `pixel_u`, `pixel_v`: 이미지 좌표 (px)
  - `depth_mm`: 깊이 (mm)
  - `confidence`: YOLO 신뢰도
- `detection_time_ms`: 검출 소요 시간

### 6.3 Data Flow

```
calibrate_detection.py:
  ┌─────────────────────────────────────────────────────┐
  │ 1. DetectCrossings 서비스 호출 (conf=0.3, count=6)  │
  │ 2. 검출 결과 표시 + dedup                            │
  │ 3. 사용자가 실측값(cm) 입력                          │
  │ 4. calibration_data.json 저장                       │
  │ 5. 6쌍 이상이면 자동 회귀 계산                       │
  │ 6. calibration_result.yaml 저장                     │
  └─────────────────────────────────────────────────────┘

visualize_detection.py:
  ┌─────────────────────────────────────────────────────┐
  │ 1. calibration_data.json 로드 → 회귀 계수 계산      │
  │ 2. DetectCrossings 서비스 호출                      │
  │ 3. 검출 포인트에 캘리브레이션 매핑 적용              │
  │ 4. Dedup (20mm 임계값)                              │
  │ 5. 이미지에 오버레이 (검출 + 매핑 좌표 표시)        │
  └─────────────────────────────────────────────────────┘
```

---

## 7. Development History

### 7.1 모델 선택 과정

| 단계 | 방법 | R² (X/Y) | 비고 |
|------|------|----------|------|
| 1차 | 단순 선형회귀 (pixel_u, depth) | ~0.85/0.70 | Y축 정확도 부족 |
| 2차 | pixel_u + pixel_v + depth | ~0.90/0.85 | pixel_v 추가로 Y 개선 |
| 3차 | + 교차항 (u×d, v×d) | 0.997/0.944 | 원근 왜곡 보정으로 대폭 개선 |
| 최종 | 이상치 제거 (84→78쌍) | 0.997/0.981 | 최대 오차 33.9→15.1mm |

### 7.2 데이터 수집 이력

- **1차 (약 20쌍):** 초기 수집, cm 단위 입력
- **2~3차 (약 40쌍):** 로봇 위치/철근 배근 위치를 변경하며 다양한 조건에서 수집
- **4차 (약 60쌍):** 추가 수집, 동일 위치 반복 측정으로 재현성 확인
- **5차 (84쌍):** 넓은 X 범위(~392mm) 커버, 이상치 6개 제거 → 최종 78쌍

### 7.3 데이터 정제 과정

수집 과정에서 발견된 오류와 수정 사항:

1. **4자리 actual 값 (×10 오류):** cm 입력 시 일부 값이 mm로 중복 변환됨 → ÷10 보정
2. **actual_y 450/570/970:** 3자리 Y값도 끝자리 0 추가 오류 → ÷10 보정
3. **actual_x=380 → 38:** 동일 pixel_u 위치에서 380과 40이 공존 → 380은 38의 오입력
4. **인덱스 3 이상치 (98mm 오차):** 재현 불가, 제거
5. **마지막 2쌍 P1/P2 실측값 교환:** 입력 순서 오류 → actual_x 값 swap
6. **최종 이상치 6개 제거:** 오차 상위 포인트(33.9mm~16.2mm) 제거

---

## 8. Limitations and Future Work

### 8.1 Current Limitations

- **좌측 카메라(zedxmini1) 미캘리브레이션:** 현재 우측 카메라만 캘리브레이션 완료
- **깊이 노이즈:** ZED 깊이 센서의 노이즈가 매핑 정확도에 직접 영향
- **Y축 범위 제한:** actual_y 범위가 0~97mm로 X축(442mm) 대비 좁음
- **선형 모델 한계:** 비선형 왜곡이 큰 영역에서는 정확도 저하 가능

### 8.2 Future Work

1. **좌측 카메라 캘리브레이션:** 동일 방법론으로 zedxmini1 캘리브레이션 수행
2. **Production 통합:** `rebar_detection_node`에 캘리브레이션 매핑 내장
3. **자동 결속 시퀀스:** 검출 → 매핑 → 스테이지 이동 → 결속의 전자동 파이프라인
4. **온라인 캘리브레이션:** 결속 결과 피드백을 통한 모델 점진적 보정
5. **비선형 모델 검토:** 데이터가 더 축적되면 다항식 회귀 또는 비선형 모델 비교 평가

---

## 9. Reproducibility

### 캘리브레이션 재수행 방법

```bash
# 1. 검출 서비스 실행 확인
ros2 service list | grep detect_crossings

# 2. 캘리브레이션 데이터 수집
python3 calibrate_detection.py

# 3. 시각화 테스트
python3 visualize_detection.py

# 4. 결과 확인
cat calibration_result.yaml
```

### 환경 정보

- **Platform:** NVIDIA Jetson (Linux 5.15.148-tegra)
- **ROS2:** Humble
- **Python:** 3.10
- **Camera SDK:** ZED SDK (zed-ros2-wrapper)
- **Detection:** Ultralytics YOLOv8
- **Dependencies:** numpy, opencv-python, PyYAML

---

*Document generated: 2026-03-09*
*Data: 78 calibration pairs, Right camera (zedxmini2)*
*Model: Multiple Linear Regression with Cross Terms (6 features)*
*Accuracy: Mean 6.7mm, Max 15.1mm, X R²=0.9965, Y R²=0.9811*
