# JointControl.msg
# 관절(위치제어) 모터 제어 메시지
# 0x143, 0x144, 0x145, 0x146, 0x147 등 위치제어 모터용

uint16 joint_id         # 모터 CAN ID (0x143=323, 0x144, 0x145, 0x146, 0x147=327)
float32 position        # 목표 위치 (degree) 또는 속도 (dps, MODE_SPEED일 때)
float32 velocity        # 최대 속도 (degree/s) - 위치 제어 모드에서만 사용
uint8 control_mode      # 제어 모드: 0=절대위치, 1=상대위치, 2=속도제어

# 제어 모드 상수
uint8 MODE_ABSOLUTE = 0  # 절대 위치 제어 (0xA4)
uint8 MODE_RELATIVE = 1  # 상대 위치 제어 (0xA4, 현재 위치 기준)
uint8 MODE_SPEED = 2     # 속도 제어 (0xA2) - position 필드에 속도(dps) 지정
