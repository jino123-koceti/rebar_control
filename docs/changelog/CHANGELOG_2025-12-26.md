# 2025-12-26 작업 내역

## 1. Lateral Motor (0x143) 엔코더 캘리브레이션 업데이트

### 배경
하드웨어 재조립 후 12시 방향(홈 위치) 엔코더 값 변경

### 변경 내용
**파일:** `rebar_base_control/joint_controller.py`

| 항목 | 이전 값 | 새 값 |
|------|---------|-------|
| `home_angle_94` | 154.94° | **33.50°** |
| `home_encoder_90` | 47282 | **24399** |

### 캘리브레이션 방법
```bash
python3 read_home_position.py
# 출력: 0x94 Angle: 33.50°, 0x90 Encoder: 24399
```

---

## 2. ZED Camera Argus 에러 해결

### 문제
`full_system.launch.py` 실행 시 zed_back 노드가 Argus InvalidState 에러로 크래시

### 원인
커스텀 `zed_minimal_odom.launch.py`와 `zed_odom_only.yaml` 설정이 문제

### 해결
**파일:** `rebar_control/launch/full_system.launch.py`

zed_wrapper 기본 launch 파일 직접 사용:
```python
zed_front = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('zed_wrapper'),
            'launch',
            'zed_camera.launch.py'  # 기본 launch 사용
        ])
    ]),
    launch_arguments={
        'camera_name': 'zed_front',
        'camera_model': 'zedx',
        'serial_number': '45320958',
    }.items(),
    condition=IfCondition(LaunchConfiguration('use_zed'))
)
```

---

## 3. Navigator 웨이포인트 자동 시작 버그 수정

### 문제
웨이포인트 로드 시 자동으로 미션 시작됨 (START_MISSION 대기 안함)

### 해결
**파일:** `rebar_control/navigator.py`

웨이포인트 로드 후 planning 상태로 대기, 명시적 START_MISSION 필요:
```python
if "waypoints" in data:
    self.load_waypoints_from_json(command)
    # 자동 시작하지 않음 - START_MISSION 명령 대기
    return
```

---

## 4. Rebar Controller 미션 리셋 버그 수정

### 문제
미션 완료 후 새 미션 시작 시 오프셋이 리셋되지 않음

### 해결
**파일:** `rebar_control/rebar_controller.py`

`/mission/feedback` 구독하여 mission_done/idle 시 상태 리셋:
```python
def feedback_callback(self, msg: String):
    feedback = json.loads(msg.data)
    state = feedback.get('state', '')

    if state in ('mission_done', 'idle'):
        self.first_waypoint_of_mission = True
        self.mission_offset_x = 0.0
        self.mission_offset_y = 0.0
        self.reference_heading = None
        # ... 기타 상태 리셋
```

---

## 5. Dual ZED Camera Odometry 180° 변환 구현

### 배경
- zed_front: 로봇 전방 장착 (전진 시 사용)
- zed_back: 로봇 후방 장착 (후진 시 사용, 180° 반대 방향)

### 문제
후진 시 zed_back의 odom이 반대 방향으로 측정됨:
- 로봇 1m 전진 → front: +1m, back: -1m

### 해결
**파일:** `rebar_control/pose_mux.py`

zed_back odom에 180° 변환 적용:
```python
def transform_back_odom(self, odom: Odometry) -> Odometry:
    """Back camera odom을 front frame으로 변환 (180° 회전)"""
    transformed = Odometry()

    # Position: X, Y 부호 반전
    transformed.pose.pose.position.x = -odom.pose.pose.position.x
    transformed.pose.pose.position.y = -odom.pose.pose.position.y

    # Orientation: 180° Z축 회전 (quaternion 연산)
    # q_180z = (0, 0, 1, 0)
    qx, qy, qz, qw = odom orientation
    transformed.orientation.x = -qy
    transformed.orientation.y = qx
    transformed.orientation.z = qw
    transformed.orientation.w = -qz

    return transformed
```

### 테스트 결과
| 동작 | front odom | back odom (원본) | back odom (변환 후) |
|------|------------|-----------------|-------------------|
| 1m 전진 | +0.88m | -0.88m | **+0.88m** |
| 1m 후진 | -0.87m | +0.89m | **-0.89m** |

---

## 6. Waypoint 로그 중복 출력 개선

### 문제
동일 waypoint가 반복 발행될 때마다 로그 출력 (5Hz → 매우 많은 로그)

### 해결
**파일:** `rebar_control/rebar_controller.py`

새 waypoint일 때만 로그 출력:
```python
def waypoint_callback(self, msg):
    is_new_waypoint = (
        self.target_waypoint is None or
        abs(self.target_waypoint.x - msg.x) > 0.001 or
        abs(self.target_waypoint.y - msg.y) > 0.001 or
        self.target_waypoint.motion_type != msg.motion_type
    )

    if is_new_waypoint:
        self.get_logger().info(f"Waypoint received: ...")
```

---

## 7. 파일 변경 요약

### 수정된 파일
| 파일 | 변경 내용 |
|------|----------|
| `joint_controller.py` | 엔코더 캘리브레이션 값 업데이트 (33.50°, 24399) |
| `full_system.launch.py` | ZED 기본 launch 사용 |
| `navigator.py` | 웨이포인트 로드 시 자동 시작 제거 |
| `rebar_controller.py` | 미션 리셋 로직 추가, 로그 중복 제거 |
| `pose_mux.py` | zed_back 180° 변환 추가 |

---

## 8. 데이터 흐름 (Dual ZED)

```
                    ┌─────────────┐
                    │  zed_front  │ (전방 카메라)
                    │  /zed_front │
                    │  /zed_node  │
                    │   /odom     │
                    └──────┬──────┘
                           │
                           ▼
┌─────────────┐     ┌─────────────┐     ┌──────────────┐
│  zed_back   │────▶│  pose_mux   │────▶│ /robot_pose  │
│ /zed_back   │     │             │     │ (PoseStamped)│
│ /zed_node   │     │ 180° 변환   │     └──────┬───────┘
│  /odom      │     │ (back only) │            │
└─────────────┘     └─────────────┘            ▼
                                        ┌──────────────┐
                                        │   rebar_     │
                                        │  controller  │
                                        └──────────────┘
```

### 카메라 전환 로직 (pose_mux)
- `cmd_vel.linear.x > 0` (전진) → zed_front 사용
- `cmd_vel.linear.x < 0` (후진) → zed_back 사용 (180° 변환)
- 정지 시 → 마지막 방향 유지 (기본: front 선호)

---

**작성:** Claude Code (Opus 4.5)
**날짜:** 2025-12-26
