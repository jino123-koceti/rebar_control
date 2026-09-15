#!/bin/bash
# 재부팅 후 자동 기동 검증 — 2026-09-15
echo "===== 부팅 후 자동 기동 확인 ====="
echo "가동 시간: $(uptime -p)"
echo
echo "--- 1. can2 ---"
ip -details link show can2 2>/dev/null | grep -E "can state|bitrate" || echo "  ✗ can2 없음 (PCAN-USB 연결 확인)"
echo
echo "--- 2. 서비스 ---"
for s in can2-up rebar-teleop; do
    printf "  %-16s enabled=%-10s active=%s\n" "$s" \
        "$(systemctl is-enabled $s 2>&1)" "$(systemctl is-active $s 2>&1)"
done
echo
echo "--- 3. ROS 노드 ---"
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/koceti/ros2_ws/install/setup.bash 2>/dev/null
timeout 15 ros2 node list 2>/dev/null | sed 's/^/  /' || echo "  ✗ 노드 없음"
echo
echo "--- 4. 조작 토픽 ---"
timeout 15 ros2 topic list 2>/dev/null | grep -E "^/cmd_vel$|^/lateral/step$" | sed 's/^/  /' || echo "  ✗ 토픽 없음"
echo
echo "--- 5. 모터 응답 (하부 4축) ---"
timeout 30 python3 - <<'PY'
import socket,struct,time
try:
    s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW); s.bind(('can2',))
except Exception as e:
    print(f"  ✗ CAN 소켓 실패: {e}"); raise SystemExit
for m in range(0x141,0x149):
    ok=0
    for _ in range(10):
        s.settimeout(0.04)
        try:
            while True: s.recv(16)
        except Exception: pass
        try: s.send(struct.pack("IB3x8s",m,8,bytes([0x9A]+[0]*7)))
        except Exception: break
        t=time.time()+0.08
        while time.time()<t:
            s.settimeout(max(0.004,t-time.time()))
            try:
                cid,_,d=struct.unpack("IB3x8s",s.recv(16))
                if (cid&0x7FF)==m+0x100: ok+=1; break
            except Exception: break
    grp="하부" if m<=0x144 else "상부"
    print(f"  {grp} 0x{m:03X}: {ok:>2}/10")
PY
echo
echo "===== 판정 ====="
echo "서비스 둘 다 active + 노드 2개 + /cmd_vel 보이면 정상입니다."
echo "조작: ros2 run rmd_robot_control teleop_keyboard"
