#!/bin/bash
# 로봇 제어 시스템 로그 관리 (자동 정리)
# cron으로 매일 실행: 0 2 * * * /home/koceti/ros2_ws/cleanup_logs.sh

LOG_DIR="/var/log/robot_control"
DAYS_TO_KEEP=7  # 7일 이상 된 로그 삭제
MAX_LOG_SIZE_MB=500  # 전체 로그 크기 제한 (MB)

log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

log_msg "========== 로그 정리 시작 =========="

# 로그 디렉토리 확인
if [ ! -d "$LOG_DIR" ]; then
    log_msg "로그 디렉토리가 없습니다: $LOG_DIR"
    exit 0
fi

cd "$LOG_DIR"

# 1. 오래된 로그 삭제 (7일 이상)
log_msg "1. ${DAYS_TO_KEEP}일 이상 된 로그 삭제 중..."
OLD_LOGS=$(find . -name "control_*.log" -type f -mtime +${DAYS_TO_KEEP} 2>/dev/null)
if [ -n "$OLD_LOGS" ]; then
    echo "$OLD_LOGS" | xargs rm -f
    log_msg "   삭제된 로그: $(echo "$OLD_LOGS" | wc -l)개"
else
    log_msg "   삭제할 오래된 로그 없음"
fi

# 2. 전체 로그 크기 확인
TOTAL_SIZE_KB=$(du -sk $LOG_DIR | cut -f1)
TOTAL_SIZE_MB=$((TOTAL_SIZE_KB / 1024))

log_msg "2. 전체 로그 크기: ${TOTAL_SIZE_MB} MB / ${MAX_LOG_SIZE_MB} MB"

# 3. 크기 제한 초과 시 오래된 로그부터 삭제
if [ $TOTAL_SIZE_MB -gt $MAX_LOG_SIZE_MB ]; then
    log_msg "   ⚠️ 크기 제한 초과! 오래된 로그 삭제 중..."
    
    # 오래된 로그부터 삭제 (최신 3개는 보존)
    ls -t control_*.log 2>/dev/null | tail -n +4 | while read logfile; do
        rm -f "$logfile"
        log_msg "   삭제: $logfile"
        
        # 크기 재확인
        TOTAL_SIZE_KB=$(du -sk $LOG_DIR | cut -f1)
        TOTAL_SIZE_MB=$((TOTAL_SIZE_KB / 1024))
        
        if [ $TOTAL_SIZE_MB -le $MAX_LOG_SIZE_MB ]; then
            break
        fi
    done
    
    TOTAL_SIZE_KB=$(du -sk $LOG_DIR | cut -f1)
    TOTAL_SIZE_MB=$((TOTAL_SIZE_KB / 1024))
    log_msg "   정리 후 크기: ${TOTAL_SIZE_MB} MB"
fi

# 4. 로그 압축 (7일 이상, 압축 안 된 로그)
log_msg "3. 오래된 로그 압축 중..."
COMPRESS_COUNT=0
find . -name "control_*.log" -type f -mtime +${DAYS_TO_KEEP} ! -name "*.gz" 2>/dev/null | while read logfile; do
    gzip "$logfile" 2>/dev/null && COMPRESS_COUNT=$((COMPRESS_COUNT + 1))
done
log_msg "   압축된 로그: ${COMPRESS_COUNT}개"

# 5. 현재 상태
TOTAL_LOGS=$(ls -1 control_*.log* 2>/dev/null | wc -l)
TOTAL_SIZE_KB=$(du -sk $LOG_DIR | cut -f1)
TOTAL_SIZE_MB=$((TOTAL_SIZE_KB / 1024))

log_msg "========== 로그 정리 완료 =========="
log_msg "현재 로그 파일 수: ${TOTAL_LOGS}개"
log_msg "현재 디스크 사용량: ${TOTAL_SIZE_MB} MB"
log_msg "로그 디렉토리: $LOG_DIR"
