# 철근 교차점 인식 시스템 개발 계획

## 📋 프로젝트 개요

**목표**: 듀얼 ZED X mini 카메라로 철근 배근 교차점을 YOLO 인식하고, 로봇 좌표계로 변환하여 자동 결속 수행

**하드웨어**:
- ZED X mini 듀얼 카메라 (대향 배치)
  - 좌측: SN 56755054 (40° 하향, 높이 108mm)
  - 우측: SN 54946194 (40° 하향, 높이 108mm)
- 작업 영역: 철근 간격 100/150/200mm

**개발 기간**: 7-8주 예상

---

## 🎯 Phase 1: 데이터 수집 시스템 (Week 1-2)

### 1.1 듀얼 카메라 녹화 노드 개발
- [ ] `rebar_vision` 패키지 생성
- [ ] `dual_camera_recorder_node.py` 작성
  - [ ] 좌우 RGB + Depth 4채널 동기화
  - [ ] message_filters.ApproximateTimeSynchronizer 사용
  - [ ] 저장 경로: `/home/test/dataset/rebar_YYYYMMDD/`
  - [ ] 메타데이터 기록 (session_info.json, frame_info.json)
- [ ] 수동 캡처 트리거 서비스
- [ ] Launch 파일 작성

**예상 출력**:
```
/home/test/dataset/rebar_20251203/
├─ left_camera/
│  ├─ rgb/frame_0000.png
│  ├─ depth/frame_0000.png
│  └─ camera_info.json
├─ right_camera/
│  ├─ rgb/frame_0000.png
│  ├─ depth/frame_0000.png
│  └─ camera_info.json
└─ metadata/
   ├─ session_info.json
   └─ frame_info.json
```

### 1.2 데이터 수집 실행
- [ ] 200mm 간격 직교 배근: 80장
- [ ] 150mm 간격: 50장
- [ ] 100mm 간격: 50장
- [ ] 사선 배근 (45°): 40장
- [ ] 다양한 조명 조건: 30장
- [ ] **총 250장 × 2 (좌우) = 500장**

### 1.3 데이터셋 정리
- [ ] 이미지 이름 변경 스크립트
  - `left_camera/rgb/frame_0000.png` → `images/left_0000.png`
  - `right_camera/rgb/frame_0000.png` → `images/right_0000.png`
- [ ] YOLO 포맷 디렉토리 구조 생성
  ```
  dataset/
  ├─ images/
  ├─ labels/  (라벨링 후 생성)
  └─ data.yaml
  ```

---

## 🎓 Phase 2: YOLO 모델 학습 (Week 3-4)

### 2.1 데이터 라벨링
- [ ] Roboflow 프로젝트 생성
- [ ] 좌측 이미지 라벨링 (250장)
  - 클래스: `rebar_crossing`
  - 교차점마다 바운딩 박스
- [ ] 우측 이미지 라벨링 (250장)
  - 같은 교차점이지만 다른 2D 좌표
- [ ] Train/Val/Test 분할 (70/20/10)
- [ ] Export → YOLOv8 format

### 2.2 YOLO 학습
- [ ] YOLOv8n 모델 학습 스크립트 작성
  - 입력: 500장 (좌우 통합)
  - Epochs: 100-150
  - 이미지 크기: 640
  - 데이터 증강 최소화 (고정 각도)
- [ ] 학습 모니터링 (mAP, Precision, Recall)
- [ ] 최적 모델 선택
- [ ] 모델 Export (ONNX, TensorRT)

**목표 성능**:
- mAP@0.5: > 0.90
- Precision: > 0.85
- Recall: > 0.85

---

## 🤖 Phase 3: ROS2 인식 노드 개발 (Week 5)

### 3.1 YOLO 추론 노드
- [ ] `rebar_detector_node.py` 작성
  - [ ] 좌측 카메라 RGB 구독
  - [ ] 우측 카메라 RGB 구독
  - [ ] YOLO 모델 로드 (각각 독립 추론)
  - [ ] 검출 결과 발행
    - `/rebar/detections_left` (vision_msgs/Detection2DArray)
    - `/rebar/detections_right`
  - [ ] 시각화 이미지 발행 (디버깅용)

### 3.2 좌표 변환 노드
- [ ] `rebar_coordinate_transformer_node.py` 작성
  - [ ] 2D 검출 결과 구독
  - [ ] Depth 이미지 구독
  - [ ] 카메라 intrinsic 파라미터 로드
  - [ ] 2D → 3D 변환 (픽셀 → 카메라 좌표계)
  - [ ] 3D → 로봇 좌표계 변환 (TF2 사용)
  - [ ] 좌우 결과 발행
    - `/rebar/points_left` (geometry_msgs/PointStamped[])
    - `/rebar/points_right`

### 3.3 융합 노드
- [ ] `rebar_fusion_node.py` 작성
  - [ ] 좌우 3D 포인트 구독
  - [ ] 융합 알고리즘 (거리 기반 매칭)
    - 임계값: 20mm 이내 동일 점으로 간주
    - 양쪽 검출: 평균 좌표, confidence=high
    - 한쪽만 검출: 해당 좌표, confidence=medium
  - [ ] 최종 목표 좌표 발행
    - `/rebar/target_points` (rebar_msgs/RebarTarget[])
      - position: [x, y, z]
      - confidence: high/medium/low
      - left_seen: bool
      - right_seen: bool

---

## 🔧 Phase 4: 카메라 Calibration (Week 5)

### 4.1 Intrinsic Calibration
- [ ] ZED SDK에서 intrinsic 파라미터 추출
- [ ] JSON 파일로 저장
  ```json
  {
    "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
    "distortion": [k1, k2, p1, p2, k3],
    "resolution": [1280, 720]
  }
  ```

### 4.2 Extrinsic Calibration (카메라 → 로봇)
- [ ] 체커보드 calibration 스크립트
- [ ] 좌측 카메라 TF 정의
  - `robot_base_link` → `zedxmini1_camera_link`
- [ ] 우측 카메라 TF 정의
  - `robot_base_link` → `zedxmini2_camera_link`
- [ ] URDF 또는 static_transform_publisher 설정

---

## 🚀 Phase 5: 통합 및 테스트 (Week 6)

### 5.1 전체 시스템 Launch
- [ ] `rebar_vision.launch.py` 작성
  ```python
  - zed_dual_cameras
  - rebar_detector_node (left + right)
  - rebar_coordinate_transformer_node
  - rebar_fusion_node
  - rviz2 (visualization)
  ```

### 5.2 시각화
- [ ] RViz2 설정
  - [ ] 카메라 이미지 (좌/우)
  - [ ] 검출 바운딩 박스 오버레이
  - [ ] 3D 포인트 클라우드
  - [ ] 융합된 목표 좌표 (Marker)
- [ ] 실시간 디버깅 토픽
  - `/rebar/debug/left_detections_viz`
  - `/rebar/debug/right_detections_viz`
  - `/rebar/debug/fusion_markers`

### 5.3 성능 테스트
- [ ] 실시간 처리 속도 측정 (목표: >10 FPS)
- [ ] 좌표 정확도 검증 (±5mm)
- [ ] 다양한 조건 테스트
  - 간격 변화 (100/150/200mm)
  - 배근 패턴 (직교/사선)
  - 조명 변화

---

## 🎮 Phase 6: 로봇 통합 (Week 7)

### 6.1 작업 계획 노드
- [ ] `rebar_work_planner_node.py`
  - [ ] 목표 좌표 구독 (`/rebar/target_points`)
  - [ ] 작업 순서 결정 (가까운 순서)
  - [ ] 로봇 경로 계획
  - [ ] 작업 명령 발행
    - `/robot/goto_target` (geometry_msgs/PoseStamped)

### 6.2 기존 제어 시스템 연동
- [ ] `iron_md_teleop_node` 수정
  - 자동 모드 추가
  - `/rebar/target_points` 구독
  - XYZ 스테이지 자동 이동
- [ ] 작업 시퀀스 통합
  - 이동 → Z축 하강 → 그리퍼 닫기 → 트리거 → 상승

---

## 📊 Phase 7: 최적화 및 배포 (Week 8)

### 7.1 성능 최적화
- [ ] TensorRT 변환 (YOLO 추론 속도 ↑)
- [ ] 멀티스레딩 (좌우 추론 병렬 처리)
- [ ] 메모리 최적화

### 7.2 안전 기능
- [ ] 리미트 센서 연동
- [ ] 비상 정지 통합
- [ ] 에러 핸들링
  - 검출 실패 시 재시도
  - 카메라 연결 끊김 감지

### 7.3 문서화
- [ ] API 문서 (Doxygen)
- [ ] 사용자 매뉴얼
- [ ] 트러블슈팅 가이드

---

## 📁 파일 구조

```
ros2_ws/src/rebar_vision/
├─ rebar_vision/
│  ├─ __init__.py
│  ├─ dual_camera_recorder_node.py      # Phase 1
│  ├─ rebar_detector_node.py            # Phase 3.1
│  ├─ rebar_coordinate_transformer_node.py  # Phase 3.2
│  ├─ rebar_fusion_node.py              # Phase 3.3
│  ├─ rebar_work_planner_node.py        # Phase 6.1
│  └─ utils/
│     ├─ camera_utils.py
│     ├─ coordinate_transform.py
│     └─ fusion_algorithm.py
│
├─ launch/
│  ├─ data_collection.launch.py
│  ├─ rebar_vision.launch.py
│  └─ full_system.launch.py
│
├─ config/
│  ├─ left_camera_info.json
│  ├─ right_camera_info.json
│  ├─ detector_params.yaml
│  └─ fusion_params.yaml
│
├─ models/
│  ├─ rebar_crossing_yolov8n.pt
│  └─ rebar_crossing_yolov8n.onnx
│
├─ rviz/
│  └─ rebar_vision.rviz
│
├─ scripts/
│  ├─ organize_dataset.py
│  ├─ train_yolo.py
│  └─ calibrate_cameras.py
│
├─ package.xml
├─ setup.py
└─ README.md
```

---

## 🔍 메시지 타입 정의

### Custom Messages (rebar_msgs)

```python
# RebarTarget.msg
geometry_msgs/Point position  # 3D 좌표 (로봇 기준)
string confidence             # "high", "medium", "low"
bool left_seen                # 좌측 카메라 검출 여부
bool right_seen               # 우측 카메라 검출 여부
float32 left_confidence       # 좌측 신뢰도 (0-1)
float32 right_confidence      # 우측 신뢰도 (0-1)
```

---

## 🎯 마일스톤

| Week | 목표 | 완료 기준 |
|------|------|-----------|
| 1-2  | 데이터 수집 | 500장 RGB+Depth 저장 완료 |
| 3-4  | YOLO 학습 | mAP > 0.90 달성 |
| 5    | ROS2 노드 개발 | 좌우 독립 추론 + 융합 성공 |
| 6    | 로봇 통합 | 자동 작업 시퀀스 동작 |
| 7-8  | 최적화 & 배포 | 실시간 처리 >10 FPS |

---

## ⚠️ 주의사항

1. **데이터 품질이 가장 중요**
   - 다양한 조건 수집 (간격, 각도, 조명)
   - 라벨링 정확도 (교차점 중심 맞춤)

2. **좌표계 정의 명확히**
   - TF 트리 검증
   - Calibration 정확도 체크

3. **실시간 성능 확보**
   - YOLO 추론 속도: <50ms/frame
   - 전체 파이프라인: <100ms

4. **안전 우선**
   - 비상 정지 항상 작동
   - 에러 발생 시 안전 상태 복귀

---

## 📝 다음 단계

✅ **지금부터 시작**: Phase 1.1 - 듀얼 카메라 녹화 노드 개발

코드 작성 순서:
1. `rebar_vision` 패키지 생성
2. `dual_camera_recorder_node.py` 작성
3. Launch 파일 작성
4. 테스트 실행

---

**작성일**: 2025-12-03  
**최종 수정**: 2025-12-03
