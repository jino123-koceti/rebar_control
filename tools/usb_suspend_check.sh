#!/bin/bash
# USB 자동절전이 실제로 꺼졌는지 확인한다. **장비가 필요 없다.**
#
#   tools/usb_suspend_check.sh
#
# 왜 필요한가 — 규칙 파일을 넣어도 `udevadm trigger` 가 먹었는지, 재부팅 뒤에도
# 유지되는지는 **sysfs 를 직접 봐야** 안다. 2026-11-06 에 측면 305 가 하루 두 번
# 버스에서 사라졌고, 원인이 이 가지의 자동절전이었다
# (`tools/udev/99-rebar-usb-nosuspend.rules` 의 측정 근거 참고).
#
#   control=on   이어야 한다 (auto 면 규칙이 안 먹은 것)
#   서스펜드(ms) 는 **규칙 적용 뒤로 늘지 않아야** 한다
#
# 누적값은 **과거 기록**이라 0 이 아닐 수 있다. 진짜 확인은 "더 늘었는가" 이고,
# 그건 두 번 재야 안다:
#
#   tools/usb_suspend_check.sh --watch 120     # 120초 간격으로 두 번 재서 증가분

# ── --watch N : N초 간격으로 두 번 재서 **증가분**을 본다 ──────────────
if [ "$1" = "--watch" ]; then
  n="${2:-120}"
  snap() { for d in /sys/bus/usb/devices/*/; do
             v=$(cat "$d/idVendor" 2>/dev/null); p=$(cat "$d/product" 2>/dev/null)
             case "$v$p" in *2bc5*|*Hub*|*hub*) ;; *) continue;; esac
             echo "$(basename "$d") $(cat "$d/power/runtime_suspended_time" 2>/dev/null)"
           done; }
  echo "  ${n}초 동안 서스펜드가 더 쌓이는지 본다…"
  a=$(snap); sleep "$n"; b=$(snap)
  grew=0
  printf "  %-10s %-14s %-14s %s\n" 포트 "전(ms)" "후(ms)" 증가
  printf "  %s\n" "--------------------------------------------------------"
  while read -r port before; do
    after=$(echo "$b" | awk -v k="$port" '$1==k{print $2}')
    [ -z "$after" ] && after="$before"
    d=$((after - before))
    mark=''; if [ "$d" -gt 0 ]; then mark='  ← ★ 아직 잠든다'; grew=$((grew+1)); fi
    printf "  %-10s %-14s %-14s %+d%s\n" "$port" "$before" "$after" "$d" "$mark"
  done <<< "$a"
  echo
  if [ "$grew" -gt 0 ]; then
    echo "  ★ $grew 개가 아직 잠든다 — 규칙이 그 장치에 안 먹었다"; exit 1
  fi
  echo "  ${n}초 동안 증가 0 ✓  자동절전이 실제로 꺼졌다"
  exit 0
fi

printf "  %-9s %-27s %-9s %-8s %-12s\n" 포트 제품 상태 control "서스펜드(ms)"
printf "  %s\n" "----------------------------------------------------------------------"
bad=0
for d in /sys/bus/usb/devices/*/; do
  v=$(cat "$d/idVendor" 2>/dev/null); p=$(cat "$d/product" 2>/dev/null)
  case "$v$p" in *2bc5*|*Hub*|*hub*) ;; *) continue;; esac
  ctl=$(cat "$d/power/control" 2>/dev/null)
  sus=$(cat "$d/power/runtime_suspended_time" 2>/dev/null)
  st=$(cat "$d/power/runtime_status" 2>/dev/null)
  mark=''
  if [ "$ctl" != "on" ]; then mark='  ← ★ auto (규칙 미적용)'; bad=$((bad+1)); fi
  printf "  %-9s %-27s %-9s %-8s %-12s%s\n" \
    "$(basename "$d")" "${p:0:27}" "$st" "$ctl" "$sus" "$mark"
done
echo
if [ "$bad" -gt 0 ]; then
  echo "  ★ $bad 개가 아직 auto 다. 설치·적용을 확인하라:"
  echo "    sudo cp tools/udev/99-rebar-usb-nosuspend.rules /etc/udev/rules.d/"
  echo "    sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=usb"
  exit 1
fi
echo "  전부 control=on ✓   (서스펜드 누적이 더는 늘지 않아야 한다)"
