# Rebar Controller 개발 내용 정리

## 개요

ROS2 Humble 기반 Rebar 로봇 내비게이션 시스템 개발 문서입니다.
Jetson 플랫폼에서 Dual ZED X 카메라(전방/후방)를 활용한 VSLAM 기반 자율주행을 구현합니다.

## 시스템 구성

### 주요 노드
- **rebar_controller**: 메인 제어 노드 (PID 기반 주행 제어)
- **navigator**: 미션 상태 머신 및 웨이포인트 관리
- **pose_mux**: 전방/후방 카메라 선택 및 좌표 변환
- **joint_controller**: 횡이동(lateral) 모터 제어

### 통신 토픽
| 토픽 | 타입 | 설명 |
|------|------|------|1
| `/mission/waypoint_array` | WaypointArray | 전체 경로 배치 전송 |
| `/mission/target_pose` | PoseStamped | 개별 웨이포인트 (하위호환) |
| `/cmd_vel` | Twist | 주행 속도 명령 |
| `/zed_front/odom` | Odometry | 전방 카메라 VSLAM |
| `/zed_back/odom` | Odometry | 후방 카메라 VSLAM |

## 핵심 기능

### 1. tire_roller 스타일 배치 웨이포인트 전송

전체 경로를 한 번에 전송하고, `rebar_controller`가 내부 인덱스로 진행 관리:

```python
# WaypointArray 메시지 구조
float32[] x              # X 좌표 배열 (meters)
float32[] y              # Y 좌표 배열 (meters)
uint8[] motion_type      # Motion type (0=DIFFERENTIAL, 1=LATERAL)
float32[] max_speed      # 최대 속도 (m/s 또는 dps)
```

### 2. 모션 타입

| 타입 | 값 | 설명 |
|------|---|------|
| MOTION_DIFFERENTIAL | 0 | 전진/후진 (차동 구동) |
| MOTION_LATERAL | 1 | 순수 횡이동 (회전 없음) |
| MOTION_HYBRID | 2 | 복합 모션 (미래 확장용) |

### 3. 후진 모드 자동 감지

목표가 로봇 뒤에 있을 때 자동으로 후진 모드 전환:
- **감지 조건**: heading_error > 150°
- **잠금 메커니즘**: 한번 후진 모드가 시작되면 목표 도달까지 유지
- **목표 지나침 감지**: heading_error < 30° AND 거리 증가 시 전진 모드로 전환

### 4. Dual 카메라 pose_mux

`cmd_vel.linear.x` 부호에 따라 카메라 자동 선택:
- **양수 (전진)**: `/zed_front/odom` 사용
- **음수 (후진)**: `/zed_back/odom` 사용 (좌표 변환 적용)

후방 카메라 좌표 변환:
- Position: X → -X, Y → -Y
- Orientation: 변환 없음 (heading_error ~180°로 후진 감지 유지)

## PID 제어 파라미터

### Distance PID (거리 제어)
| 파라미터 | 값 | 설명 |
|----------|-----|------|
| kp_linear | 1.0 | 비례 게인 (0.5 → 1.0 증가) |
| ki_linear | 0.05 | 적분 게인 (정상상태 오차 보정) |
| kd_linear | 0.1 | 미분 게인 |
| integral_limit | 0.2 | Anti-windup 한계값 |

### Heading PID (방향 제어)
| 파라미터 | 값 | 설명 |
|----------|-----|------|
| kp_angular | 1.0 | 비례 게인 |
| ki_angular | 0.1 | 적분 게인 (heading 정상상태 오차 보정) |
| kd_angular | 0.2 | 미분 게인 |
| integral_heading_limit | 0.3 | Anti-windup 한계값 |

### 속도 제한
| 파라미터 | 값 | 설명 |
|----------|-----|------|
| max_linear_vel | 0.5 m/s | 최대 선속도 |
| max_angular_vel | 1.0 rad/s | 최대 각속도 |
| distance_tolerance | 25 mm | 목표 도달 허용 오차 |
| heading_tolerance | 0.1 rad (~6°) | 방향 허용 오차 |

## 해결된 이슈

### 1. 후진 시 목표 근처에서 회전 발생
**원인**: heading_error ±180° 경계에서 불안정 (angular_vel 스파이크)

**해결책**: 목표 근접 시 각속도 감소
```python
# 후진 시
if distance < 50mm:
    angular_vel *= 0.2  # 20%로 감소
elif distance < 100mm:
    angular_vel *= 0.5  # 50%로 감소
```

### 2. 목표 근처에서 접근 속도가 너무 느림
**원인**: PID 비례 게인(kp=0.5)만으로는 짧은 거리에서 속도 부족

**해결책**:
- kp_linear: 0.5 → 1.0 (응답성 향상)
- ki_linear: 0.0 → 0.05 (정상상태 오차 보정)
- Anti-windup 적용 (integral_limit = 0.2)

### 3. 전진/후진 시 대각선 이동
**원인**: ki_angular = 0.0으로 heading 오차 누적 보정 없음

**해결책**:
- ki_angular: 0.0 → 0.1
- integral_heading_limit = 0.3 (Anti-windup)
- 짧은 거리(15cm 이하)에서 각속도 50%로 감소

## VSLAM 정보 로깅

제어 루프(20Hz)에서 실시간 로깅:

```
[전진/후진] VSLAM: (x, y) yaw=각도° | 목표: (x, y) | 거리오차: mm, 헤딩오차: ° | cmd: lin=속도, ang=속도
```

**출력 예시**:
```
[전진] VSLAM: (0.150, 0.002) yaw=2.3° | 목표: (0.600, 0.000) | 거리오차: 450.1mm, 헤딩오차: -0.5° | cmd: lin=0.450, ang=-0.005
[후진] VSLAM: (0.580, -0.003) yaw=1.8° | 목표: (0.000, 0.000) | 거리오차: 580.2mm, 헤딩오차: 178.5° | cmd: lin=-0.500, ang=0.012
```

## 테스트 결과

### 테스트 미션: (0,0) → (0.6,0) → (0,0)
- 전진: (0,0) → (0.6,0) - 성공
- 횡이동: 1회 완료 - 성공
- 후진: (0.6,0) → (0,0) - 성공

## 파일 구조

```
src/
├── rebar_control/
│   └── rebar_control/
│       ├── rebar_controller.py   # 메인 제어 노드
│       ├── navigator.py          # 미션 상태 머신
│       └── pose_mux.py           # 카메라 선택/변환
├── rebar_base_control/
│   └── rebar_base_control/
│       └── joint_controller.py   # 횡이동 모터 제어
└── rebar_base_interfaces/
    └── msg/
        ├── Waypoint.msg          # 개별 웨이포인트
        └── WaypointArray.msg     # 배치 웨이포인트 배열
```

## 향후 개선 사항

1. **속도 프로파일**: 사다리꼴 프로파일 적용 (현재는 PID 기반)
2. **경로 평활화**: 웨이포인트 간 곡선 보간
3. **장애물 회피**: VSLAM + LiDAR 융합
4. **MOTION_HYBRID**: 전진+횡이동 복합 모션 구현

---
*최종 업데이트: 2025-12-26*
