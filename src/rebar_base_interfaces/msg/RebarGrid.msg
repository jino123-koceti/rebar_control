# RebarGrid.msg
# 2x3 교차점 그리드 (결속 순서로 정렬됨)
#
# 그리드 레이아웃 (위에서 본 모습, 로봇 전방이 X+):
#   [0] [1] [2]    <- row 0 (가까운 쪽)
#   [3] [4] [5]    <- row 1 (먼 쪽)

RebarDetection[] detections  # 정렬된 검출 결과 (최대 6개)
uint8 grid_rows              # 행 수 (기본 2)
uint8 grid_cols              # 열 수 (기본 3)
uint8 total_detected         # 필터링 전 총 검출 수
bool valid                   # 유효 포인트 확보 여부
string error_message         # 에러 메시지
