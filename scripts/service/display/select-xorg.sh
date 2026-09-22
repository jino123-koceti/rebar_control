#!/bin/sh
# 부팅 때 모니터 연결 여부를 보고 X 설정을 고른다 (gdm 보다 먼저 실행).
#
# 왜 (2026-09-22): /etc/X11/xorg.conf 를 dummy 드라이버로 고정해 둔 탓에(9/11, 헤드리스 원격용)
#   모니터를 꽂으면 화면이 가짜 화면에만 그려지고 실제 모니터는 부팅 로고에서 멈춘 것처럼 보였다.
#   → DP 가 연결돼 있으면 NVIDIA 설정, 아니면 dummy 설정.
#
# ⚠ 부팅 **때만** 판단한다. 켜진 뒤 모니터를 꽂으면: sudo systemctl restart gdm (또는 재부팅)
CONF=/etc/X11/xorg.conf
HEADLESS=/etc/X11/xorg.conf.headless
MONITOR=/etc/X11/xorg.conf.monitor

# ★ 모니터 커넥터(card1-DP-1)는 nvidia-drm 이 올라와야 생긴다. 그런데 평소엔 X(gdm)가
#   뜨면서야 로드된다(실측 46.6s) → 그 전에 보면 커넥터가 없어 늘 '모니터 없음'으로 오판했다
#   (2026-09-22 휴대용 모니터 시험에서 22.8s 판단 → headless). 먼저 직접 올린다.
#   옵션은 /etc/modprobe.d 설정을 그대로 따른다(평소 X가 올릴 때와 같음).
modprobe nvidia-drm 2>/dev/null || echo "nvidia-drm 로드 실패(계속 진행)"

# 커넥터가 생기고 연결 감지(HPD)가 들어올 때까지 최대 10초 기다린다
for i in $(seq 20); do
    grep -qx connected /sys/class/drm/card*-*/status 2>/dev/null && break
    sleep 0.5
done

state=disconnected
for s in /sys/class/drm/card*-*/status; do
    [ -r "$s" ] && [ "$(cat "$s")" = connected ] && state=connected && echo "모니터 감지: $s"
done

if [ "$state" = connected ]; then src=$MONITOR; else src=$HEADLESS; fi
if ! cmp -s "$src" "$CONF"; then
    cp "$src" "$CONF"
fi
echo "X 설정: $(basename "$src") (모니터 $state)"
