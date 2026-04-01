# Tying Orchestrator 리팩토링 + 검출 로직 개선 계획

작성일: 2026-04-01

## 1. 배경

### 현재 문제
- `tying_orchestrator_node.py`가 **1978줄**로 비대, 단일 파일에 모든 로직 집중
- 검출 시 양쪽 카메라를 동시 호출하나, **Right 자세에서 Left 카메라가 교차점을 못 봄**
  - YOLO는 Left 카메라에서 11~13개 검출하지만, Right 자세에서는 0~1개만 검출
  - 보간으로 포인트를 추론하지만 정확도 저하
- detection_node의 `_filter_far_side` 제거 완료 (2026-04-01), orchestrator가 필터링 담당

### 데이터 근거 (2026-04-01 시험)
- Right 자세: right cam 12~13개 검출 OK, left cam 0~4개 (부족)
- Left 자세: left cam 15개 검출 OK, right cam 0~5개 (부족)
- 동일 위치에서 calibrate_gbr.py(left cam)로 3개 검출 확인 → 자세 문제 확정

---

## 2. 목표

### Phase A: 모듈 분리 (리팩토링)
orchestrator를 기능별 모듈로 분리하여 유지보수성 향상

### Phase B: 검출 로직 개선
자세별 해당 카메라만 검출 → 자세변경 후 재검출하는 2단계 검출/결속 구현

---

## 3. Phase A: 모듈 분리

### 3.1 현재 구조 (1978줄, 단일 파일)

```
tying_orchestrator_node.py
├── 데이터 클래스: ActionType, Action, TyingState (L47~82)
├── __init__: 파라미터, 구독/발행, 상태변수 (L87~350)
├── 파일 로거 (L353~370)
├── 콜백: obstacle, mission_cmd, control_mode, homing, motor_feedback (L374~590)
├── 상태머신 루프 (L591~690)
├── 검출 처리: _start_detection, _handle_detecting, _finalize (L695~880)
├── 포인트 처리: _process_camera_points, _build_final_points (L885~1055)
├── 클러스터링/보간/추세선 (L1062~1230)
├── 액션큐: _build_action_queue, _optimize, _append_tying, _append_pose_change (L1231~1465)
├── 액션실행: _execute_current_action, _advance, _check_goal (L1470~1600)
├── 트리거: _handle_trigger_phase (L1600~1640)
├── 모터 명령: _send_xy, _send_yaw, _send_z, _send_trigger (L1642~1695)
├── Z축 토크 모니터링 (L1699~1805)
└── 완료/에러/취소/상태전이/피드백 (L1808~1978)
```

### 3.2 분리 대상 모듈

#### (1) `detection_manager.py` - 검출 관리
**추출 대상 메서드:**
- `_handle_detecting()` → 멀티검출 서비스 호출 + 누적 관리
- `_finalize_multi_detection()` → 누적 데이터 최종 처리
- `_process_camera_points()` → 카메라별 클러스터링+보간+추세선
- `_build_final_points()` → 범위 필터링 + 라벨링
- `_cluster_points()` → 거리 기반 클러스터링
- `_interpolate_missing()` → 누락 포인트 보간
- `_fit_trendline()` → 추세선 피팅+보정

**인터페이스:**
```python
class DetectionManager:
    def __init__(self, node, params):
        """node: ROS2 노드 참조 (서비스 클라이언트, 로거, flog 접근)"""

    def start(self, camera_side: str):
        """검출 시작. camera_side: 'right', 'left', 'both'"""

    def update(self) -> Optional[List[Point]]:
        """상태머신 루프에서 매 tick 호출.
        검출 완료 시 포인트 리스트 반환, 진행 중이면 None"""

    def reset(self):
        """상태 초기화"""
```

#### (2) `action_builder.py` - 액션 큐 생성
**추출 대상 메서드:**
- `_build_action_queue()` → 포인트→액션큐 변환
- `_optimize_z_xy_overlap()` → Z↑+XY 연속동작 최적화
- `_append_tying_actions()` → 단일 포인트 결속 액션 (XY→Z↓→트리거→Z↑)
- `_append_pose_change_right_to_left()` → R→L 자세변경 시퀀스
- `_append_pose_change_left_to_right()` → L→R 자세변경 시퀀스

**인터페이스:**
```python
class ActionBuilder:
    def __init__(self, params):
        """params: 스테이지 파라미터 (deg_per_mm, yaw offsets 등)"""

    def build(self, points, current_pose, tying_direction) -> List[Action]:
        """포인트 리스트 + 현재 자세 → 액션 큐 생성"""

    def build_pose_change(self, from_pose, to_pose) -> List[Action]:
        """자세변경 액션만 생성"""
```

#### (3) `z_torque_monitor.py` - Z축 토크 모니터링
**추출 대상 메서드:**
- `_start_z_torque_monitor()` → 모니터링 시작, 임계값 계산
- `_stop_z_torque_monitor()` → 타이머 정지
- `_poll_z_torque()` → 0x9C 폴링 + 파일로그
- `_log_z_descent_summary()` → 통계 요약 기록
- `_handle_z_overload()` → 과부하 시 Z정지 + 트리거 스킵

**인터페이스:**
```python
class ZTorqueMonitor:
    def __init__(self, node, params):
        """node: ROS2 노드 참조 (타이머, 퍼블리셔)"""

    def start(self, z_wait_time, speed_percent):
        """하강 시작 시 모니터링 시작"""

    def stop(self):
        """모니터링 중지"""

    def update_feedback(self, current_mA, speed_dps):
        """motor_feedback 콜백에서 호출"""

    def check_overload(self, elapsed) -> Optional[str]:
        """WAITING_Z에서 매 tick 호출. 과부하 시 'OVERLOAD_ACCEL'/'OVERLOAD_DECEL' 반환"""

    def get_summary(self) -> str:
        """하강 완료 시 통계 요약"""
```

### 3.3 분리 후 orchestrator 구조 (~600줄 목표)

```
tying_orchestrator_node.py (~600줄)
├── __init__: 파라미터, 모듈 초기화
├── 콜백: obstacle, mission_cmd, control_mode, homing, motor_feedback
├── 상태머신 루프 (_state_machine_loop)
├── 액션 실행 (_execute_current_action, _advance)
├── 모터 명령 (_send_xy, _send_yaw, _send_z, _send_trigger)
├── 트리거 (_handle_trigger_phase)
└── 완료/에러/취소/상태전이/피드백

detection_manager.py (~500줄)
├── 멀티검출 서비스 호출 + 누적
├── 클러스터링 / 보간 / 추세선
└── 범위 필터링 + 라벨링

action_builder.py (~300줄)
├── 액션 큐 생성
├── 자세변경 시퀀스
└── Z↑+XY 최적화

z_torque_monitor.py (~150줄)
├── 0x9C 폴링 + 임계값 판정
├── 2단계 감지 (가속/감속)
└── 파일로그 기록
```

### 3.4 공유 데이터 (`tying_common.py`)
- `ActionType`, `Action`, `TyingState` enum/dataclass
- 모든 모듈에서 import

---

## 4. Phase B: 검출 로직 개선

### 4.1 현재 방식 (문제)
```
Right 자세 → right+left 동시 검출 → left 0~1개 → 보간으로 때움
→ 자세변경 → left 포인트 결속 (부정확)
```

### 4.2 개선 방식

#### Right 자세에서 시작하는 경우 (forward)
```
1. Right 자세 확인
2. right cam만 멀티검출 (5회) → 클러스터링 → 3개 포인트
3. Right 포인트 결속 (X 큰→작은 순)
4. 자세변경 (Right → Left)
5. left cam만 멀티검출 (5회) → 클러스터링 → 3개 포인트
6. Left 포인트 결속 (X 작은→큰 순)
7. 결속 완료 → 다음 웨이포인트
```

#### Left 자세에서 시작하는 경우 (reverse)
```
1. Left 자세 확인
2. left cam만 멀티검출 (5회) → 클러스터링 → 3개 포인트
3. Left 포인트 결속 (X 큰→작은 순)
4. 자세변경 (Left → Right)
5. right cam만 멀티검출 (5회) → 클러스터링 → 3개 포인트
6. Right 포인트 결속 (X 작은→큰 순)
7. 결속 완료 → 다음 웨이포인트
```

### 4.3 결속 순서 규칙
- **첫 번째 자세**: X 큰 → 작은 (자세변경 시 X=home 근처에서 끝나도록)
- **두 번째 자세**: X 작은 → 큰 (자세변경 후 X=home 근처에서 시작)
- 이렇게 하면 자세변경 전후 XY 이동거리 최소화

### 4.4 상태머신 변경

현재:
```
IDLE → DETECTING → EXECUTING_ACTION → ... → COMPLETE
```

변경:
```
IDLE → DETECTING_PHASE1 → EXECUTING_PHASE1 → ...
     → POSE_CHANGING → DETECTING_PHASE2 → EXECUTING_PHASE2 → ...
     → COMPLETE
```

또는 더 간단하게: 기존 COMPLETE에서 2차 검출로 재진입
```
IDLE → DETECTING → EXECUTING → ... → PHASE1_DONE
     → (자세변경 액션) → DETECTING → EXECUTING → ... → COMPLETE
```

### 4.5 DetectionManager 변경사항
- `start(camera_side='right')` → 해당 카메라만 호출
- 양쪽 동시 호출 모드 제거
- 조기 종료 조건: 해당 카메라의 클러스터만 체크

---

## 5. 구현 순서

### Step 1: 공통 데이터 추출
- `tying_common.py` 생성: ActionType, Action, TyingState

### Step 2: z_torque_monitor.py 분리
- 가장 독립적인 모듈, 의존성 적음
- orchestrator에서 호출 인터페이스 연결

### Step 3: action_builder.py 분리
- 액션 큐 생성 로직 추출
- 자세변경 시퀀스 포함

### Step 4: detection_manager.py 분리
- 멀티검출 + 클러스터링 + 보간 + 추세선 추출
- camera_side 파라미터 추가

### Step 5: orchestrator 경량화
- 분리된 모듈 import 및 조합
- 상태머신 + 콜백 + 모터 명령만 남김

### Step 6: 검출 로직 개선 (Phase B)
- DetectionManager에 단일 카메라 모드 구현
- orchestrator에 2단계 검출/결속 흐름 구현
- _handle_complete에서 2차 검출 재진입 로직

### Step 7: 빌드 및 동작 테스트
- colcon build
- 수동 결속 시작으로 2단계 검출/결속 확인

---

## 6. Z축 토크 모니터링 현황 (참고)

### 확정된 파라미터 (2026-04-01 시험)
- 80% 속도 기준 정상 하강 데이터 (19회 수집)
- 가속 구간 (0~650ms): 전류 0→2700mA, 속도 0→750dps
- 전환점 (~676ms): 전류 급락 → 0~40mA
- 감속 구간 (676~926ms): 전류 370~880mA (정상), 1780~2470mA (충돌)

### 2단계 감지 로직
- 가속 구간: `accel_limit = 500 + speed% × 30` (80%에서 2900mA)
- 감속 구간: `decel_limit = 1200mA`, `decel_ratio = 0.75`, 연속 2회 초과

### 알려진 이슈
- `decel_ratio=0.65`에서 오탐 1건 → 0.75로 수정 완료
- 100% 속도에서는 가속 구간 전류가 이미 최대치 → 80% 이하 권장

---

## 7. 파일 경로

```
src/rebar_vision/rebar_vision/
├── tying_orchestrator_node.py  (리팩토링 대상)
├── tying_common.py             (신규: 공통 데이터)
├── detection_manager.py        (신규: 검출 관리)
├── action_builder.py           (신규: 액션 큐 생성)
└── z_torque_monitor.py         (신규: Z축 토크 모니터링)
```
