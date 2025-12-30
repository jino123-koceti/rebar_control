
● 좋습니다. 구체적인 설계를 제안하겠습니다:

  📐 설계 방안: 모터별 제어 방식 설정

  1. 전체 아키텍처

  ┌─────────────────────────────────────────────────────────┐
  │ joint_controller.py                                     │
  │  - 모터별 설정 (MOTOR_CONFIGS)                          │
  │  - 0x143: 싱글턴 (0x94)                                 │
  │  - 0x144~0x147: 멀티턴 (0x92)                           │
  └─────────────────────────────────────────────────────────┘
           │ JointControl                    │ Encoder Request
           ↓                                 ↓
  ┌──────────────────────┐           ┌─────────────────────┐
  │ can_sender.py        │           │ can_parser.py       │
  │  - 0xA4 전송         │←──────────│  - 0x90/0x92/0x94   │
  │  (모터 구분 안 함)   │   CAN Bus │    모두 파싱        │
  └──────────────────────┘           └─────────────────────┘

  2. joint_controller.py 구성

  # ========================================
  # 모터별 제어 방식 설정
  # ========================================
  MOTOR_CONFIGS = {
      0x143: {
          'name': 'lateral',
          'type': 'single_turn',      # 싱글턴
          'encoder_cmd': 0x94,         # 0x94 사용
          'home_angle': 154.94,        # 12시 방향 (고정값)
          'rotation_step': 360.0,      # 1회전 = 360°
          'tolerance': 2.0,            # 위치 허용 오차 (°)
      },
      0x144: {
          'name': 'joint_1',
          'type': 'multi_turn',        # 멀티턴
          'encoder_cmd': 0x92,         # 0x92 사용
          'home_angle': None,          # 부팅 시 현재 위치로 캘리브레이션
          'rotation_step': 360.0,
          'tolerance': 2.0,
      },
      0x145: {
          'name': 'joint_2',
          'type': 'multi_turn',
          'encoder_cmd': 0x92,
          'home_angle': None,
          'rotation_step': 360.0,
          'tolerance': 2.0,
      },
      # ... 0x146, 0x147
  }

  # ========================================
  # 모터별 상태 추적
  # ========================================
  self.motor_states = {
      0x143: {
          'angle_94': None,            # 싱글턴 각도
          'home': 154.94,              # 고정 홈
          'calibrated': True,          # 홈 고정값이므로 항상 True
      },
      0x144: {
          'angle_92': None,            # 멀티턴 누적 각도
          'home': None,                # 부팅 시 캘리브레이션
          'calibrated': False,
      },
      # ...
  }

  # ========================================
  # 주기적 각도 폴링 (0.2초)
  # ========================================
  def _periodic_angle_request(self):
      for motor_id, config in MOTOR_CONFIGS.items():
          if config['encoder_cmd'] == 0x94:
              self._request_single_circle_angle(motor_id)  # 0x94
          elif config['encoder_cmd'] == 0x92:
              self._request_output_angle(motor_id)         # 0x92
          # (0x90도 추가 가능)

  # ========================================
  # 피드백 처리 (can_parser → joint_controller)
  # ========================================
  def _joint_feedback_callback(self, msg):
      motor_id = msg.joint_id
      config = MOTOR_CONFIGS.get(motor_id)

      if config['type'] == 'single_turn':
          # 0x94 값 업데이트
          self.motor_states[motor_id]['angle_94'] = msg.current_position

          # 홈 검증 (154.94° 근처인지)
          if abs(msg.current_position - config['home_angle']) < 1.0:
              self.motor_states[motor_id]['home'] = config['home_angle']
              self.motor_states[motor_id]['calibrated'] = True

      elif config['type'] == 'multi_turn':
          # 0x92 값 업데이트
          self.motor_states[motor_id]['angle_92'] = msg.current_position

          # 홈 자동 캘리브레이션 (첫 부팅 시)
          if not self.motor_states[motor_id]['calibrated']:
              self.motor_states[motor_id]['home'] = msg.current_position
              self.motor_states[motor_id]['calibrated'] = True
              self.get_logger().info(
                  f"✅ 0x{motor_id:03X} 홈 캘리브레이션: 
  {msg.current_position:.2f}°"
              )

  # ========================================
  # S17/S18 버튼: 1회전 명령
  # ========================================
  def _execute_home_rotation(self, motor_id, direction):
      config = MOTOR_CONFIGS[motor_id]
      state = self.motor_states[motor_id]

      if not state['calibrated']:
          self.get_logger().warn(f"0x{motor_id:03X} 아직 캘리브레이션 안됨")
          return

      # ===== 싱글턴 모터 (0x143) =====
      if config['type'] == 'single_turn':
          current = state['angle_94']
          home = state['home']

          # Step 1: 홈이 아니면 홈으로 복귀
          if abs(current - home) > config['tolerance']:
              self.get_logger().info(f"🏠 홈으로 복귀: {current:.1f}° → 
  {home:.1f}°")
              self._send_joint_command_abs(motor_id, home, 200.0,
  JointControl.MODE_ABSOLUTE)
              return

          # Step 2: 홈에서 1회전
          rotation = config['rotation_step'] if direction == '+' else
  -config['rotation_step']
          target = home + rotation  # 예: 154.94 + 360 = 514.94

          self.get_logger().info(
              f"🔄 1회전: {home:.1f}° → {target:.1f}° ({rotation:+.0f}°)"
          )
          self._send_joint_command_abs(motor_id, target, 200.0,
  JointControl.MODE_ABSOLUTE)

      # ===== 멀티턴 모터 (0x144~0x147) =====
      elif config['type'] == 'multi_turn':
          current = state['angle_92']
          home = state['home']

          # Step 1: 홈이 아니면 홈으로 복귀
          if abs(current - home) > config['tolerance']:
              self.get_logger().info(f"🏠 홈으로 복귀: {current:.1f}° → 
  {home:.1f}°")
              self._send_joint_command_abs(motor_id, home, 200.0,
  JointControl.MODE_ABSOLUTE)
              return

          # Step 2: 홈에서 1회전 후 복귀
          rotation = config['rotation_step'] if direction == '+' else
  -config['rotation_step']
          target = home + rotation  # 예: -203.5 + (-360) = -563.5

          self.get_logger().info(
              f"🔄 1회전: {home:.1f}° → {target:.1f}° ({rotation:+.0f}°)"
          )
          # 1회전만 하고 복귀는 안 함 (다음 클릭 시 복귀)
          self._send_joint_command_abs(motor_id, target, 200.0,
  JointControl.MODE_ABSOLUTE)

          # TODO: 1회전 후 자동 복귀하려면?
          # - 타이머 사용: 회전 완료 후 home으로 재전송
          # - 또는 두 단계: target → home (2번 명령)

  3. can_parser.py 수정 (선택사항)

  현재 0x90/0x92/0x94 모두 같은 필드(current_position)에 담고 있는데,
  구분하려면:

  # Option A: 현재대로 유지 (joint_controller가 모터별로 해석)
  feedback_msg.current_position = angle_deg

  # Option B: 필드 추가 (JointFeedback.msg에 encoder_type 추가)
  feedback_msg.encoder_type = command_type  # 0x90/0x92/0x94
  feedback_msg.current_position = angle_deg

  추천: Option A (현재 구조 유지) - joint_controller가 모터별로 요청한 값이
  뭔지 알고 있으므로

  4. can_sender.py

  변경 없음! 현재 그대로:
  position_cmd = int(target_deg * 100.0)  # degree → 0.01°/LSB
  # 0xA4로 전송 (싱글턴/멀티턴 구분 안 함)

  0xA4 명령은 angle 값을 그대로 받아서:
  - 154.94° → 154.94°로 이동
  - 514.94° → 1회전 후 154.94°로 이동 (희망적 해석)
  - -563.5° → -563.5° 위치로 이동

  ---
  📋 정리

  | 항목  | 0x143 (횡이동)   | 0x144~0x147 (관절) |
  |-----|---------------|------------------|
  | 타입  | single_turn   | multi_turn       |
  | 읽기  | 0x94 (0-360°) | 0x92 (누적 각도)     |
  | 홈   | 154.94° (고정)  | 부팅 시 현재 위치       |
  | 1회전 | 514.94° 전송    | home + 360° 전송   |
  | 복귀  | 자동? (테스트 필요)  | 다음 클릭 또는 타이머     |

  남은 이슈:
  1. 0xA4에 514.94° 보내면 1회전 후 복귀하는지 테스트 필요
  2. 멀티턴 모터: 1회전 후 자동 복귀 로직 (타이머? 2단계 명령?)

  이렇게 구성하면 어떨까요?
