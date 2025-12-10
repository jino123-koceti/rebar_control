# Precision Navigation Status Message
# 정밀 주행 상태

uint8 STATE_IDLE = 0
uint8 STATE_VALIDATE = 1
uint8 STATE_MOVING = 2
uint8 STATE_REACHED = 3
uint8 STATE_ERROR = 4

uint8 state            # 현재 상태
bool active            # 활성화 플래그 (MOVING일 때 true)
string error_msg       # 에러 메시지 (STATE_ERROR 시)
