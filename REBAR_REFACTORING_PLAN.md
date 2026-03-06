# Rebar Control System Refactoring Plan

## 목표
Tire Roller의 검증된 아키텍처를 참고하여 현재 개발 중인 Rebar 제어 시스템을 계층화된 구조로 리팩토링

---

## ✅ 진행 상황

# Rebar Control System Refactoring Plan (간결본)

## 목표
- Tire Roller 아키텍처를 참고해 Rebar 제어를 2계층 구조로 리팩토링 (하드웨어 추상화 + 상위 제어)

## 진행 상태
- Phase 1 (인터페이스 정의): 완료
- Phase 2 (하드웨어 추상화, rebar_base_control): 완료
- Phase 3 (상위 제어, rebar_control): 진행 중

## 하드웨어 추상화 계층 (rebar_base_control)
- 노드: can_parser, can_sender, drive_controller, modbus_controller, authority_controller, navigator_base
- 설정: config/can_devices.yaml, config/modbus_devices.yaml
- 상태머신: idle, manual, auto, navigating, tying, emergency_stop (python-statemachine)

## 상위 제어 계층 (rebar_control)
- 노드: zenoh_client, navigator, rebar_controller, rebar_publisher
- 통신 흐름: UI(Zenoh) ↔ zenoh_client ↔ /mission/command → navigator → /mission/target_pose → rebar_controller → /cmd_vel → drive_controller → CAN
- 상태 발행: rebar_publisher가 /mission/status(JSON)을 생성, zenoh_client가 msgpack으로 UI에 전송

## ZED Odometry 연동 계획
- 단일 ZED일 때: /zed/odom을 /robot_pose로 remap하여 rebar_controller에 공급
- 듀얼 ZED(전진/후진 방향별 센서)일 때: 변환/스위칭 노드 사용
  - 입력: front ZED odom, back ZED odom, 방향 신호(예: cmd_vel 부호 또는 별도 토픽)
  - TF: 두 ZED의 base_link 상대 변환을 static TF로 등록해 공통 프레임(odom)으로 정렬
  - 로직: 전진 시 front, 후진 시 back을 선택해 /robot_pose(PoseStamped, frame=odom) 발행; 저속 구간은 히스테리시스로 바운싱 방지
  - 파라미터 예: front_odom_topic, back_odom_topic, direction_source, velocity_deadband, output_topic(/robot_pose)

## 남은 작업 (요약)
- rebar_publisher: 미션 진행률/필드 보강(waypoint 진행 등) 완료됨
- ZED 연동: odom → /robot_pose 공급, 듀얼 ZED일 경우 변환/스위칭 노드 구현 및 launch 연결
- navigator: PAUSE/RESUME 세부 로직, 상태 출력 보강
- 테스트: 통합 실행 시 /robot_pose 유무 확인, UI 연동, vcan 기반 안전 테스트

## 일정 가이드
- Phase 3 상위 제어 마무리: 2~3일 예상
- Phase 4 통합/검증: 2~3일 예상

## 주의사항
- 단계별 통합 테스트 후 다음 단계 진행
- 하드웨어 연동 전 vcan 등으로 안전 확인
  - [x] 통합 JSON 상태 생성 및 /mission/status 발행
  - [ ] 상세 필드/진행률 반영 (waypoint 진행, mission_status 등) — 추후 보강 필요

**Phase 3-3: ZED X Odometry 연동**
- [ ] ZED X Wrapper 상태 확인
  - [ ] zed-ros2-wrapper 실행 확인
  - [ ] /zed/odom 토픽 확인
  - [ ] TF (odom → base_link) 확인

- [ ] Odometry 변환 (옵션)
  - [ ] /zed/odom → /robot_pose 변환 노드 작성 (필요시)
  - [ ] 또는 topic remap 설정

**Phase 3-4: 미션 관리 구현**
- [x] `navigator.py` 구현 (기본 동작 완료)
  - [x] State Machine 정의 (idle, planning, navigating, mission_done, emergency_stop + stop/recover)
  - [x] /mission/command 구독 (E-STOP, GO_HOME, START/STOP/ABORT, WAYPOINTS:<json>)
  - [x] 웨이포인트 관리 및 순회, /mission/target_pose 발행
  - [x] /mission/feedback 발행 (기본 진행 정보)
  - [ ] PAUSE/RESUME 등 중단/재개 로직 미구현 → 추후 보강 필요

**Phase 3-5: 경로 추종 제어 구현**
- [x] `rebar_controller.py` 구현 (PID 기반 단순 제어)
  - [x] /robot_pose, /mission/target_pose 구독
  - [x] 거리/헤딩 계산 후 /cmd_vel 발행
  - [x] 목표 도달 시 /mission/waypoint_reached 알림
  - [x] 20Hz 제어 루프

**Phase 3-6: Launch 및 Config**
- [x] `launch/control_system.launch.py` 작성 (zenoh_client, navigator, rebar_controller, rebar_publisher 포함)
- [x] `config/zenoh_config.yaml` 작성 (zenoh 키/토픽 매핑 포함)

- [ ] 통합 Launch 파일 (옵션)
  - [ ] `launch/full_system.launch.py`
  - [ ] Phase 2 (base_system) 포함
  - [ ] Phase 3 (control_system) 포함

**Phase 3-7: 테스트**
- [ ] 빌드 테스트
  - [ ] colcon build --packages-select rebar_control
  - [ ] 의존성 확인

- [ ] 노드별 단위 테스트
  - [ ] zenoh_client: UI 명령 echo 테스트
  - [ ] rebar_publisher: 상태 메시지 생성 확인
  - [ ] navigator: 웨이포인트 로드 테스트
  - [ ] rebar_controller: 단순 목표 추종 테스트

- [ ] 통합 테스트
  - [ ] Phase 2 + Phase 3 동시 실행
  - [ ] Laptop UI 테스트 스크립트 (Zenoh 명령 전송)
  - [ ] ZED X Odometry 데이터 수신 확인

#### 메시지 추가 필요 (rebar_base_interfaces)

- [ ] WorkArea.msg 추가
  ```
  float32 min_x
  float32 min_y
  float32 max_x
  float32 max_y
  ```

- [ ] MissionCommand.msg 추가 (옵션, String 대신)
  ```
  string command
  string data
  ```

- [ ] MissionStatus.msg 추가 (옵션, String JSON 대신)
  ```
  string control_mode
  string mission_status
  geometry_msgs/Pose2D position
  float32 speed
  float32 heading
  float32 battery
  int32 current_waypoint
  int32 total_waypoints
  string[] errors
  ```

#### 다음 단계
Phase 4: UI 개발 (PyQt 기반, Laptop)
Phase 5: 통합 테스트 및 검증

### Phase 4: 통합 및 검증
- [ ] 전체 시스템 Launch 파일
  - [ ] `launch/rebar_system.launch.py` (모든 노드 통합)

- [ ] 기존 스크립트 업데이트
  - [ ] `integrated_control_debug.sh` 수정
  - [ ] 새로운 launch 파일 사용

- [ ] 문서화
  - [ ] 아키텍처 다이어그램
  - [ ] 각 노드 README
  - [ ] 메시지/액션 인터페이스 문서
  - [ ] 설정 파라미터 가이드

- [ ] 성능 테스트
  - [ ] 600mm 전진 정확도 (목표: ±10mm)
  - [ ] 600mm 후진 정확도 (목표: ±10mm)
  - [ ] Heading 유지 성능
  - [ ] 응답 지연 시간 측정

- [ ] 기존 코드 정리
  - [ ] `position_control_node.py` 백업 후 삭제
  - [ ] `iron_md_teleop_node.py` 백업 후 삭제
  - [ ] `precision_navigation_node.py` 백업 후 삭제
  - [ ] `cmd_vel_relay.py` 삭제
  - [ ] `ezi_io_node.py` 백업 후 삭제
  - [ ] `seengrip_node.py` 백업 후 삭제

### Phase 5: 추가 기능 (Optional)
- [ ] RQT UI 개발
  - [ ] S20 모드 제어 패널
  - [ ] 상태 모니터링
  - [ ] 수동 명령 입력

- [ ] 로깅 및 모니터링
  - [ ] 주행 데이터 기록
  - [ ] 에러 로그 수집
  - [ ] 성능 메트릭 분석

- [ ] Vision 시스템 (rebar_vision)
  - [ ] ZED 기반 철근 인식
  - [ ] 위치 보정 알고리즘

---

## 예상 개발 일정

| Phase | 작업 내용 | 예상 공수 |
|-------|----------|----------|
| Phase 1 | 메시지 인터페이스 정의 | 1일 |
| Phase 2 | 하드웨어 추상화 계층 | 3-4일 |
| Phase 3 | 상위 제어 계층 | 2-3일 |
| Phase 4 | 통합 및 검증 | 2-3일 |
| Phase 5 | 추가 기능 (Optional) | 3-5일 |

**총 예상 개발 기간:** 약 2주

---

## 리팩토링 시 주의사항

### 1. 점진적 마이그레이션
- 한 번에 모든 코드를 변경하지 말고 단계적으로 진행
- 각 Phase 완료 후 반드시 통합 테스트 수행
- 기존 코드는 백업 후 삭제

### 2. 하드웨어 특성 유지
- **Differential Drive 모터 방향:**
  - 0x141 (왼쪽): 그대로
  - 0x142 (오른쪽): 반전 필요 (모터가 서로 마주보고 장착)
- **CAN 버스:**
  - CAN2 (1Mbps): 모터 통신
  - CAN3 (250kbps): 리모콘
- **Modbus:**
  - Seengrip: RTU, 115200 baud, /dev/ttyUSB0
  - EZI-IO: TCP, 192.168.1.100

### 3. 기존 PID 파라미터 유지
```yaml
# precision_nav.yaml
heading_pid:
  kp: 0.5
  ki: 0.0
  kd: 0.1

distance_pid:
  kp: 0.003
  ki: 0.0
  kd: 0.0001
```

### 4. 의존성 추가
```bash
# python-statemachine 설치
pip3 install python-statemachine
```

### 5. 테스트 우선
- 각 노드 구현 시 단위 테스트 작성
- 통합 전 개별 노드 동작 검증
- 실제 하드웨어 테스트 전 시뮬레이션 테스트

---

## 기대 효과

### 1. 코드 품질 향상
- 계층화된 구조로 유지보수성 향상
- 단일 책임 원칙 준수
- 테스트 용이성 증가

### 2. 재사용성 증가
- 하드웨어 계층과 제어 계층 분리
- 다른 로봇 프로젝트에도 활용 가능
- 모듈 단위 교체 용이

### 3. 확장성 향상
- 새로운 기능 추가 용이
- Vision 시스템 통합 준비
- RQT UI 개발 기반 마련

### 4. 안정성 향상
- State Machine 기반 안전한 상태 관리
- Action Server의 취소 기능
- 명확한 에러 처리

---

## 참고 자료

### Tire Roller 코드 위치
```
/home/koceti/ros2_ws/src/tire_roller/
├── tire_roller_basecontrol/
│   ├── authority_controller.py
│   ├── drive_controller.py
│   └── navigator.py (State Machine)
└── tire_roller_control/
    ├── navigator.py (High-level)
    └── base_controller.py
```

### 현재 Rebar 코드 위치
```
/home/koceti/ros2_ws/src/
├── rebar_control/
├── rmd_robot_control/
├── ezi_io_ros2/
└── seengrip_ros2/
```

### 핵심 파일
- **State Machine 예제:** `tire_roller_basecontrol/navigator.py`
- **Action Server 예제:** `tire_roller_control/base_controller.py`
- **CAN 통신 예제:** `tire_roller_basecontrol/authority_controller.py`

---

## 문의 및 지원

리팩토리 과정에서 문제 발생 시:
1. 각 Phase별 체크리스트 확인
2. Tire Roller 코드 참고
3. 단위 테스트로 문제 격리
4. 로그 분석 및 디버깅

**리팩토링 완료 목표:** 안정적이고 확장 가능한 Rebar 제어 시스템 구축
