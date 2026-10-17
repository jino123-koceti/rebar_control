#!/usr/bin/env bash
# 시연 중 카메라가 죽었을 때 쓰는 복구 절차. 가벼운 것부터 차례로 올라간다.
#
# 왜 이 순서인가 (2026-10-15 실측):
#  · 카메라 노드는 **respawn 이 없다** (orbbec 패키지 런치에 설정이 없음).
#    노드만 죽이면 다시 안 올라오니 그 방법은 쓰지 말 것.
#  · 대신 노드는 5.5초마다 스스로 재접속을 시도한다. 그래서 **USB 만 건강해지면
#    사람이 아무것도 안 해도 붙는다.** 복구의 목표는 노드가 아니라 USB 다.
#  · 그날 실제로 고친 것은 **케이블 재체결**이었다. 소프트웨어 재시작을 두 번
#    했지만 소용없었다 — 커널이 1-4.3 허브를 통째로 떨어뜨리고 있었기 때문이다.
#
# 사용:  bash tools/camera_recover.sh          # 진단만
#        bash tools/camera_recover.sh --fix    # 1·2단계까지 자동 실행 (sudo 필요)

set -u
HUB=1-4.3            # 카메라(1-4.3.2)·트리거(1-4.3.4.2)·WiFi 가 매달린 상위 허브
CAM=1-4.3.2
TOPIC=/camera/color/image_raw

say() { printf '\n== %s\n' "$*"; }

say "① 진단 — 어느 층이 깨졌나"
if lsusb | grep -qi 2bc5; then
  echo "  USB 열거: ✅ 보인다"
else
  echo "  USB 열거: ❌ 안 보인다 → 케이블/전원 문제. 3단계로."
fi
n=$(journalctl -k --since "-3min" --no-pager 2>/dev/null \
    | grep -icE "disconnect|disabled by hub|error -71")
echo "  최근 3분 USB 끊김·오류: ${n}건  $([ "$n" -gt 3 ] && echo '← 링크가 불안정하다 (3단계)')"
if timeout 12 ros2 topic hz "$TOPIC" 2>/dev/null | grep -q average; then
  echo "  프레임: ✅ 나온다 — 카메라는 정상이다. 다른 원인을 보라."
  exit 0
fi
echo "  프레임: ❌ 없음"
journalctl -u rebar-teleop.service --since "-2min" --no-pager 2>/dev/null \
  | grep -oE "Device with serial number [A-Z0-9]+ not found" | tail -1 \
  | sed 's/^/  노드: /'

# ⚠⚠ [2026-10-17] **"not found" 의 1순위 원인은 케이블이 아니라 권한이다.**
#   재열거 때 udev 규칙이 가끔 안 먹어서 usbfs 노드가 crw------- root root 로
#   만들어진다. 서비스는 koceti 로 돌고 SDK 는 usbfs 로 장치를 여므로, 권한이
#   없으면 "장치가 없다" 로 보인다. 장치는 lsusb 에 멀쩡히 보인다.
DEVNUM=$(cat /sys/bus/usb/devices/$CAM/devnum 2>/dev/null)
BUSNUM=$(cat /sys/bus/usb/devices/$CAM/busnum 2>/dev/null)
if [ -n "$DEVNUM" ]; then
  NODE=$(printf "/dev/bus/usb/%03d/%03d" "$BUSNUM" "$DEVNUM")
  PERM=$(stat -c '%A %U:%G' "$NODE" 2>/dev/null)
  echo "  usbfs 노드: $NODE  $PERM"
  if [ -r "$NODE" ] && [ -w "$NODE" ]; then
    echo "    권한: ✅ 열 수 있다 → 권한 문제는 아니다"
  else
    echo "    권한: ❌ **못 연다 — 이것이 원인이다**"
    echo "    즉효:  sudo chmod 0666 $NODE   그리고 컨테이너를 다시 띄운다"
    echo "           (SDK 는 한 번 실패하면 권한이 고쳐져도 스스로 복구하지 않는다)"
    echo "    영구:  sudo cp tools/udev/99-orbbec-gemini2l-perm.rules /etc/udev/rules.d/"
    echo "           sudo udevadm control --reload"
  fi
fi

if [ "${1:-}" != "--fix" ]; then
  echo
  echo "  조치하려면: bash tools/camera_recover.sh --fix"
  exit 1
fi

say "② USB 가지만 전기적으로 재시작 (케이블 안 건드림)"
echo "  $HUB 를 unbind → 2초 → bind"
echo "$HUB" | sudo tee /sys/bus/usb/drivers/usb/unbind >/dev/null && sleep 2
echo "$HUB" | sudo tee /sys/bus/usb/drivers/usb/bind   >/dev/null
echo "  재열거 대기 15초 (노드가 스스로 다시 붙는다)"
sleep 15
if timeout 12 ros2 topic hz "$TOPIC" 2>/dev/null | grep -q average; then
  echo "  ✅ 복구됨 — 호밍은 그대로다 (서비스를 안 내렸다)"
  exit 0
fi

say "③ 여기까지 안 되면 **물리적 문제**다"
cat <<'MSG'
  소프트웨어로는 더 할 게 없다. 2026-10-15 에 실제로 들었던 처치는 이것이다:

    1) 상위 허브(1-4.3)의 **Jetson 쪽 케이블**을 빼고 다시 꽂는다.
       하위 장치(카메라·트리거)보다 **이쪽을 먼저** 본다 — 그날 증상이
       한 포트에서 허브 전체로 번졌고, 상단 재체결로 멎었다.
    2) 모터 전원선과 나란히 가는 USB 구간이 있으면 떼어 놓는다
       (커널이 "disabled by hub (EMI?)" 를 찍는다).
    3) 그래도 안 되면 서비스 재시작 — **호밍이 날아가므로 마지막 수단**이다.
       재시작 전에 요축을 2번 자세(리미트 감지 위치)로 돌려 놓을 것.
MSG
exit 1
