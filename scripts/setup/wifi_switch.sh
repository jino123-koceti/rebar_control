#!/bin/bash
# 와이파이 전환 — 실패하면 원래 망으로 자동 복귀한다.
#
# ## 왜
# 이 젯슨의 무선카드(RTL8822CE / rtl88x2ce)는 **연결 중 스캔이 막혀 있다.**
# `nmcli dev wifi list --rescan yes` 도 `iw scan dump` 도 **현재 접속 중인 AP 하나만**
# 보고한다. 즉 목표 AP가 잡히는지 미리 확인할 방법이 없어 **붙어봐야 안다.**
#
# 그런데 원격(SSH/Tailscale)으로 작업 중이면 전환 실패 = 접속 상실이다.
# → 전환 후 인터넷을 확인하고, 안 되면 **원래 망으로 되돌린다.**
#
# 사용:
#   sudo scripts/setup/wifi_switch.sh CNR_L680_33601A
#   sudo scripts/setup/wifi_switch.sh CNR_L680_33601A SVT501_5G   # 복귀망 직접 지정
#
# 진행 상황은 /tmp/wifi_switch.log 에 남는다(접속이 끊겨도 나중에 확인 가능).

set -u
TARGET="${1:?사용법: $0 <전환할SSID> [복귀할SSID]}"
LOG=/tmp/wifi_switch.log
WAIT_LINK=25        # 연결 수립 대기 (초)
WAIT_NET=15         # 인터넷 확인 대기 (초)

log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG"; }

CURRENT=$(nmcli -t -f NAME,TYPE con show --active | awk -F: '$2=="802-11-wireless"{print $1; exit}')
FALLBACK="${2:-$CURRENT}"

log "=== 와이파이 전환 시작 ==="
log "현재: ${CURRENT:-없음}   →   목표: $TARGET   (실패 시 복귀: ${FALLBACK:-없음})"

if [ "$CURRENT" = "$TARGET" ]; then
    log "이미 $TARGET 에 연결돼 있다. 할 일 없음."
    exit 0
fi

# 인터넷 확인.
# ⚠ ICMP(ping)는 막혀 있을 수 있으므로 쓰지 않는다 — SVT501_5G에서 8.8.8.8 ping이
#   100% 손실인데 실제 인터넷은 멀쩡했다.
# ⚠ 특정 도메인 하나에 의존하지 않는다 — CNR_L680_33601A 망은
#   connectivity-check.gstatic.com 을 차단해서, 연결에 성공해도 "인터넷 없음"으로
#   오판해 되돌리는 일이 있었다(2026-08-19).
#   → **IP 직결**(DNS 불필요) + **DNS 질의**를 각각 보고 하나라도 되면 통과시킨다.
net_ok() {
    # ① IP로 직접 HTTPS (DNS·특정도메인 무관)
    for ip in 1.1.1.1 8.8.8.8; do
        curl -sS --max-time 5 -o /dev/null "https://$ip" 2>/dev/null && return 0
    done
    # ② DNS가 살아 있는지 (이름 해석만 되면 망은 정상으로 본다)
    python3 - <<'PY' 2>/dev/null && return 0
import socket, sys
socket.setdefaulttimeout(4)
try:
    socket.gethostbyname('google.com'); sys.exit(0)
except Exception:
    sys.exit(1)
PY
    return 1
}

# 비밀번호가 실제로 저장돼 있는지 **값으로** 확인한다.
# ⚠ `psk-flags: 0`은 "시스템 저장소에 저장한다"는 **정책**일 뿐 값이 있다는 뜻이 아니다.
#   실제로 flags=0인데 값이 비어 있어 `no secrets: No agents were available`로
#   즉시 실패한 사례가 있다(2026-08-19). 값 자체를 확인해야 한다.
if ! nmcli -s -g 802-11-wireless-security.psk con show "$TARGET" 2>/dev/null | grep -q .; then
    log "❌ '$TARGET' 프로필에 **비밀번호가 없다.** 전환 못 함."
    log "   해결: 화면(GUI) 네트워크 메뉴에서 접속하거나, 아래 명령으로 한 번 접속할 것."
    log "   sudo nmcli dev wifi connect '$TARGET' password '<와이파이비밀번호>'"
    log "   (한 번 성공하면 저장되어 다음부터 자동 접속된다)"
    exit 3
fi

log "전환 시도…"
if timeout "$WAIT_LINK" nmcli con up "$TARGET" >>"$LOG" 2>&1; then
    log "링크 수립됨. 인터넷 확인 중…"
    for i in $(seq 1 "$WAIT_NET"); do
        if net_ok; then
            IP=$(nmcli -t -f IP4.ADDRESS dev show | head -1 | cut -d: -f2)
            log "✅ 전환 성공: $TARGET   ($IP)"
            exit 0
        fi
        sleep 1
    done
    log "⚠ 링크는 붙었으나 ${WAIT_NET}초간 인터넷 없음 → 복귀한다"
else
    log "❌ 전환 실패 (AP 없음/인증 실패/타임아웃)"
fi

# ── 복귀 ────────────────────────────────────────────────
if [ -n "$FALLBACK" ] && [ "$FALLBACK" != "$TARGET" ]; then
    log "복귀 시도: $FALLBACK"
    if timeout "$WAIT_LINK" nmcli con up "$FALLBACK" >>"$LOG" 2>&1; then
        log "↩️ 원래 망으로 복귀 완료: $FALLBACK"
    else
        log "🛑 복귀도 실패. **콘솔에서 직접 조치할 것.**"
        exit 2
    fi
else
    log "복귀할 망이 지정되지 않았다. 자동 재접속(autoconnect)에 맡긴다."
fi
exit 1
