#!/bin/bash
# X(0x145) 가 다른 보레이트로 응답하는지 훑는다 — ROM 통신설정 어긋남 vs 트랜시버 고장
#
#   sudo bash tools/motor/can_bitrate_probe.sh
#
# ⚠ 서비스를 내렸다 올린다. 끝나면 can2 를 1Mbps 로 복원하고 서비스를 다시 띄운다.
# ⚠ 호밍 원점은 어차피 서비스 재시작으로 사라진다 — 끝나면 다시 호밍해야 한다.
set -u
PY=/home/koceti/ros2_ws/src/rebar_control/tools/motor/can_scan.py
[ "$(id -u)" -eq 0 ] || { echo "sudo 로 실행하세요"; exit 1; }

echo "== 서비스 정지 (CAN 을 단독으로 쓴다)"
systemctl stop rebar-teleop.service
sleep 2

for BR in 1000000 500000 250000 125000; do
  echo "== 보레이트 ${BR}"
  ip link set can2 down 2>/dev/null
  ip link set can2 type can bitrate "$BR" restart-ms 100 || { echo "  설정 실패"; continue; }
  ip link set can2 up || { echo "  올리기 실패"; continue; }
  sleep 1
  python3 "$PY" 2>&1 | sed 's/^/  /'
  echo "  버스 오류: $(ip -s -d link show can2 | grep -A1 bus-errors | tail -1 | awk '{print $2}')"
done

echo "== can2 를 1Mbps 로 복원"
ip link set can2 down 2>/dev/null
ip link set can2 type can bitrate 1000000 restart-ms 100
ip link set can2 up
echo "== 서비스 재기동"
systemctl start rebar-teleop.service
echo "끝 — 0x145 가 어느 보레이트에서 응답했는지 보세요."
echo "  응답 있음 → ROM 통신설정 문제 (복구 가능)"
echo "  전부 침묵 → 트랜시버·보드 고장 (교체)"
echo "⚠ 서비스가 재시작됐으니 호밍을 다시 하세요."
