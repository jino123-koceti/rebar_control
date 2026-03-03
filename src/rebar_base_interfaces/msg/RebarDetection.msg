# RebarDetection.msg
# 단일 철근 교차점 검출 결과

float32 x                   # 로봇 프레임 X (mm)
float32 y                   # 로봇 프레임 Y (mm)
float32 z                   # 로봇 프레임 Z (mm)
float32 confidence          # YOLO 신뢰도 (0.0~1.0)
float32 depth_mm            # 원본 깊이값 (mm)
uint16 pixel_u              # 이미지 중심 X
uint16 pixel_v              # 이미지 중심 Y
uint8 camera_id             # 0=left, 1=right
