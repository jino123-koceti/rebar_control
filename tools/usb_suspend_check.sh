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
