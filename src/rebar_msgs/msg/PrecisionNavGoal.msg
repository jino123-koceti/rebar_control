# Precision Navigation Goal Message
# 정밀 주행 목표 명령

uint8 TYPE_LINEAR = 0   # 직진/후진
uint8 TYPE_ANGULAR = 1  # 회전 (미사용, 추후 확장용)

uint8 type              # 명령 타입
float32 distance        # 이동 거리 (mm, type=LINEAR)
float32 angle           # 회전 각도 (deg, type=ANGULAR, 미사용)
float32 max_velocity    # 최대 속도 (m/s)
