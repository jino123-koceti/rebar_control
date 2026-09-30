# 안전 판정 결과 (L2 safety_node → L1 최종 차단)
#
# 판정은 safety_node 가 하고, **차단은 모터 직전(L1)에서** 한다.
# 상위에서 막으면 "무엇이 명령했든" 을 보장할 수 없다 (YEAR3_ARCHITECTURE.md §2).
#
# ⚠ 방향별로 막는다. 범퍼가 눌렸다는 건 이미 부딪혔다는 뜻이고, 전 방향을 막으면
#   빠져나올 수단까지 사라져 그 자리에 갇힌다 (2차년도 bumper_node·deck_edge 가 같은 결론).

std_msgs/Header header

# 전체 정지 — 아무 축도 움직이면 안 된다
bool estop                  # 리모콘 비상정지 (송신기 꺼짐·START 전 포함)
bool stop_switch            # 하부 STOP 스위치
bool inputs_stale           # 안전 입력이 갱신되지 않는다 (센서·노드 두절)

# 주행 방향별 차단 (해당 방향으로만 못 간다)
bool block_forward
bool block_backward
bool block_left             # 좌선회 차단
bool block_right            # 우선회 차단

# 축 방향별 차단. 예: ["x+", "z-"] — 리미트에 닿은 축의 그 방향만 막는다
string[] blocked_axes

# 사람이 읽을 이유 (로그·UI 용)
string reason
