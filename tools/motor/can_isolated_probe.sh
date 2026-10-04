#!/bin/bash
# X 를 **혼자** 떼어 can3 에 물린 뒤 보레이트를 훑는다.
#
#   1. X 의 CAN 선만 can3 에 연결한다 (can2 는 건드리지 않는다)
#   2. sudo bash tools/motor/can_isolated_probe.sh
#
# ⚠ can2 는 손대지 않으므로 다른 모터·서비스·호밍 원점에 영향이 없다.
# ⚠ can3 는 리모콘 버스다 — 리모콘 수신이 잠시 끊긴다 (끝나면 250k 로 복원).
#
# 왜 혼자 떼어야 하나: 멀쩡한 7개가 1Mbps 로 말하므로, can2 를 다른 보레이트로
# 내리면 그 7개가 오류 폭풍을 일으켜(4만 건) X 의 응답이 묻힌다.
set -u
PY=/home/koceti/ros2_ws/src/rebar_control/tools/motor/can_scan.py
[ "$(id -u)" -eq 0 ] || { echo "sudo 로 실행하세요"; exit 1; }
ORIG=250000        # can3 원래 보레이트 (리모콘)

for BR in 1000000 500000 250000 125000; do
  echo "== can3 보레이트 ${BR}"
  ip link set can3 down 2>/dev/null
  ip link set can3 type can bitrate "$BR" restart-ms 100 || { echo "  설정 실패"; continue; }
  ip link set can3 up || { echo "  올리기 실패"; continue; }
  sleep 1
  python3 "$PY" can3 2>&1 | sed 's/^/  /'
  echo "  버스 오류: $(ip -s -d link show can3 | grep -A1 bus-errors | tail -1 | awk '{print $2}')"
done

echo "== can3 를 ${ORIG} 로 복원 (리모콘)"
ip link set can3 down 2>/dev/null
ip link set can3 type can bitrate "$ORIG" restart-ms 100
ip link set can3 up
echo "끝 — 어느 보레이트에서 응답했는지 보세요."
echo "  응답 있음 → ROM 통신설정 문제 (그 보레이트로 접속해 1Mbps·0x145 로 되돌린다)"
echo "  전부 침묵 → 트랜시버·보드 고장 (교체)"
