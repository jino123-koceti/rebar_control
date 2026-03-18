#!/bin/bash
# 주행 모터 위치 실시간 모니터링 (로그 파일 기반)

LOG_FILE="/tmp/unified_control_debug.log"
OUTPUT_LOG=${OUTPUT_LOG:-}  # 설정 시 화면 출력과 함께 파일에도 저장

echo "=========================================="
echo "주행 모터 실시간 위치 모니터링"
echo "=========================================="
echo ""
echo "S20 모드에서 AN3으로 360도 회전 명령을 내리세요."
echo "Ctrl+C로 종료"
echo ""

# 종료 트랩을 미리 설정해 Ctrl+C 처리
trap "echo ''; echo '모니터링 종료'; exit" SIGINT SIGTERM

if [ ! -f "$LOG_FILE" ]; then
    echo "⚠️  로그 파일이 없습니다: $LOG_FILE"
    echo "시스템이 실행 중인지 확인하세요:"
    echo "  sudo systemctl status robot-control.service"
    exit 1
fi

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📊 동기화 로그 실시간 모니터링"
echo "   - 위치 업데이트에 Δ각/Δt, dps, (옵션) mm/s를 붙여 출력"
echo "   - mm_per_rev(mm/1회전)을 알고 있다면 실행 전 export MM_PER_REV=값 으로 설정"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [ -n "$OUTPUT_LOG" ]; then
    mkdir -p "$(dirname "$OUTPUT_LOG")"
    echo "📁 모니터링 출력 저장: $OUTPUT_LOG"
fi

MM_PER_REV=${MM_PER_REV:-}
SAMPLE_DT_DEFAULT=${SAMPLE_DT_DEFAULT:-0.05}  # 로그 타임스탬프가 동일 초일 때 사용할 기본 Δt(초)

# 로그 파일을 실시간으로 모니터링하면서 위치 업데이트에 속도 추정치(dps)와 (옵션) mm/s를 붙여서 출력
monitor_stream() {
    stdbuf -oL tail -F "$LOG_FILE" | awk -v mm_per_rev="$MM_PER_REV" '
    function to_epoch(date_str, time_str,    d, t){
        split(date_str, d, "-"); split(time_str, t, ":");
        return mktime(sprintf("%s %s %s %s %s %s", d[1], d[2], d[3], t[1], t[2], t[3]));
    }

    # 모터별 이전 위치/시간 보관
    /0x14[12].*위치 업데이트/ {
        # 예: 2025-12-09 13:44:43 [INFO] 🔄 [S20/AN3] 0x142 위치 업데이트: 1287.3° → 1287.3° (변화: 0.0°)
        date_str=$1; time_str=$2;
        motor=""; new_pos="";
        if(match($0, /0x14[0-9]/)) motor=substr($0, RSTART, RLENGTH);
        if(match($0, /→ [+-]?[0-9.]+°/)) new_pos=substr($0, RSTART+3, RLENGTH-4);

        if(motor!="" && new_pos!=""){
            key=motor;
            now=to_epoch(date_str, time_str);
            if(prev_time[key]>0){
                dt=now-prev_time[key];
                if(dt<=0){ dt=SAMPLE_DT_DEFAULT; }
                dp=new_pos-prev_pos[key];
                if(dt>0){
                    dps=dp/dt;  # deg/s
                    msg=sprintf("%s [Δ각: %.1f°, Δt: %.3fs, 속도: %.2fdps]", $0, dp, dt, dps);
                    if(mm_per_rev != ""){
                        mm_per_deg = mm_per_rev/360.0;
                        mmps = dps * mm_per_deg;
                        msg = sprintf("%s [%.1fmm/s]", msg, mmps);
                    }
                    print msg;
                } else {
                    print $0;
                }
            } else {
                print $0;
            }
            prev_time[key]=now;
            prev_pos[key]=new_pos;
            next;
        }
    }

    # 기타 관심 로그 그대로 출력 (동기화, 명령, 목표 도달 등)
    /\[동기화\]|S20\/AN3.*회전 명령|목표 도달.*0x14[12]/ { print; next; }

    # 그 외는 무시
    { next }
'
}

if [ -n "$OUTPUT_LOG" ]; then
    monitor_stream | tee -a "$OUTPUT_LOG"
else
    monitor_stream
fi
