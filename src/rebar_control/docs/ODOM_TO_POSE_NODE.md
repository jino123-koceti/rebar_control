# odom_to_pose 노드

## 개요

ZED X 카메라의 `nav_msgs/Odometry` 메시지를 `geometry_msgs/PoseStamped`로 변환하여 발행하는 단순 변환 노드입니다.

## 용도

- **단일 ZED 카메라 환경**: 하나의 ZED X만 사용하는 경우
- **간단한 테스트 환경**: 듀얼 ZED 없이 기본적인 주행 제어 테스트 시

> **Note:** 듀얼 ZED 카메라(전진/후진 전환)가 필요한 경우 `pose_mux` 노드를 사용하세요.

---

## 토픽

### 구독 (Subscriptions)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/zed/zed_node/odom` (기본값) | `nav_msgs/Odometry` | ZED X 카메라의 Odometry |

### 발행 (Publications)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/robot_pose` (기본값) | `geometry_msgs/PoseStamped` | 변환된 로봇 위치 |

---

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| `input_odom_topic` | string | `/zed/zed_node/odom` | 입력 Odometry 토픽 |
| `output_pose_topic` | string | `/robot_pose` | 출력 PoseStamped 토픽 |
| `output_frame` | string | `odom` | 출력 메시지의 frame_id |
| `queue_size` | int | 10 | 구독/발행 큐 크기 |

---

## 실행 방법

### 1. 단독 실행

```bash
ros2 run rebar_control odom_to_pose
```

### 2. 파라미터 지정 실행

```bash
ros2 run rebar_control odom_to_pose \
  --ros-args \
  -p input_odom_topic:=/custom/odom \
  -p output_pose_topic:=/custom/pose \
  -p output_frame:=map
```

### 3. Launch 파일로 실행 (단일 ZED 모드)

```bash
ros2 launch rebar_control control_system.launch.py use_dual_zed:=false
```

### 4. Launch 파일로 실행 (커스텀 토픽)

```bash
ros2 launch rebar_control control_system.launch.py \
  use_dual_zed:=false \
  single_zed_odom_topic:=/custom/zed/odom
```

---

## 동작 원리

### 메시지 변환

```
nav_msgs/Odometry → geometry_msgs/PoseStamped
  - header.stamp: odom.header.stamp (타임스탬프 유지)
  - header.frame_id: 파라미터로 지정 (기본: 'odom')
  - pose: odom.pose.pose (PoseWithCovariance에서 Pose 추출)
```

### 발행 전략

- **콜백 기반 즉시 발행**: Odometry 수신 즉시 변환하여 발행
- **최소 지연**: 변환 오버헤드 < 1ms
- **타임스탬프 유지**: 원본 Odometry의 타임스탬프 보존

---

## 성능 특성

| 항목 | 값 |
|------|-----|
| 발행 주파수 | 입력 Odometry 주파수와 동일 (일반적으로 10~30Hz) |
| 변환 지연 | < 1ms |
| CPU 사용량 | < 1% |
| 메모리 사용량 | ~10MB |

---

## 통합 테스트

### 토픽 확인

```bash
# 노드 실행
ros2 run rebar_control odom_to_pose

# 다른 터미널에서 토픽 확인
ros2 topic list | grep -E '(odom|robot_pose)'

# 메시지 내용 확인
ros2 topic echo /robot_pose
```

### 발행 주파수 측정

```bash
ros2 topic hz /robot_pose
```

예상 출력:
```
average rate: 20.001
    min: 0.049s max: 0.051s std dev: 0.00039s window: 21
```

### 메시지 변환 검증

```bash
# Odometry position
ros2 topic echo /zed/zed_node/odom --field pose.pose.position

# PoseStamped position (동일해야 함)
ros2 topic echo /robot_pose --field pose.position
```

---

## 시스템 통합

### control_system.launch.py와의 통합

`control_system.launch.py`는 자동으로 ZED 환경을 감지하여 적절한 노드를 실행합니다:

- **듀얼 ZED (기본)**: `pose_mux` 노드 실행
- **단일 ZED**: `odom_to_pose` 노드 실행 (launch arg: `use_dual_zed:=false`)

### 하위 노드와의 연결

```
[ZED X Camera]
    ↓ /zed/zed_node/odom (Odometry)
[odom_to_pose]
    ↓ /robot_pose (PoseStamped)
├─→ [rebar_controller] - 경로 추종 제어
└─→ [zenoh_client] - UI 전송
```

---

## 트러블슈팅

### 문제: /robot_pose가 발행되지 않음

**원인:** ZED 카메라 노드가 실행되지 않았거나 토픽 이름 불일치

**해결:**
```bash
# ZED 토픽 확인
ros2 topic list | grep zed

# odom_to_pose 파라미터 확인
ros2 param list /odom_to_pose
ros2 param get /odom_to_pose input_odom_topic
```

### 문제: 발행 주파수가 너무 낮음

**원인:** ZED 카메라의 Odometry 발행 주파수가 낮음

**해결:**
```bash
# ZED Odometry 주파수 확인
ros2 topic hz /zed/zed_node/odom

# ZED 카메라 설정 확인 (zed_wrapper 설정 파일)
```

### 문제: Frame ID가 잘못됨

**원인:** output_frame 파라미터 설정 필요

**해결:**
```bash
ros2 run rebar_control odom_to_pose \
  --ros-args -p output_frame:=map
```

---

## 관련 노드

- **pose_mux**: 듀얼 ZED 카메라 환경용 (전진/후진 자동 전환)
- **rebar_controller**: /robot_pose를 구독하여 경로 추종 제어
- **zenoh_client**: /robot_pose를 UI로 전송

---

## 개발자 정보

- **패키지**: rebar_control
- **파일**: `rebar_control/odom_to_pose.py`
- **진입점**: `odom_to_pose = rebar_control.odom_to_pose:main`
- **작성일**: 2025-12-23
