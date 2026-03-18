#!/bin/bash
# 로봇 제어 시스템 로그 뷰어

LOG_DIR="/var/log/robot_control"
LATEST_LOG="$LOG_DIR/control_latest.log"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

show_help() {
    echo "로봇 제어 시스템 로그 뷰어"
    echo ""
    echo "사용법: $0 [옵션]"
    echo ""
    echo "옵션:"
    echo "  -f, --follow        실시간 로그 보기 (tail -f)"
    echo "  -l, --list          모든 로그 파일 목록"
    echo "  -e, --errors        에러 메시지만 필터링"
    echo "  -c, --can           CAN 통신 로그만"
    echo "  -j, --joystick      조이스틱/리모콘 입력만"
    echo "  -m, --motor         모터 제어 로그만"
    echo "  -s, --stats         시스템 상태 통계"
    echo "  -h, --help          도움말"
    echo ""
    echo "예제:"
    echo "  $0 -f               # 실시간 로그"
    echo "  $0 -e               # 에러만 보기"
    echo "  $0 -c | tail -n 50  # 최근 50개 CAN 메시지"
}

show_list() {
    echo -e "${GREEN}=== 로그 파일 목록 ===${NC}"
    if [ -d "$LOG_DIR" ]; then
        ls -lht $LOG_DIR/*.log 2>/dev/null | head -n 10
        echo ""
        echo "전체 로그 개수: $(ls $LOG_DIR/*.log 2>/dev/null | wc -l)"
        echo "디스크 사용량: $(du -sh $LOG_DIR 2>/dev/null | cut -f1)"
    else
        echo "로그 디렉토리가 없습니다: $LOG_DIR"
    fi
}

show_errors() {
    echo -e "${RED}=== 에러 로그 ===${NC}"
    if [ -f "$LATEST_LOG" ]; then
        grep -E "ERROR|⚠️|WARN|error|fail" "$LATEST_LOG" | tail -n 50
    else
        echo "로그 파일이 없습니다."
    fi
}

show_can() {
    echo -e "${BLUE}=== CAN 통신 로그 ===${NC}"
    if [ -f "$LATEST_LOG" ]; then
        grep -E "CAN2|CAN3|📤|📥|0x14[0-9]|0x24[0-9]" "$LATEST_LOG" | tail -n 100
    else
        echo "로그 파일이 없습니다."
    fi
}

show_joystick() {
    echo -e "${YELLOW}=== 리모콘 입력 로그 ===${NC}"
    if [ -f "$LATEST_LOG" ]; then
        grep -E "🎮|리모콘|AN[1-4]|S[0-9]{2}" "$LATEST_LOG" | tail -n 100
    else
        echo "로그 파일이 없습니다."
    fi
}

show_motor() {
    echo -e "${GREEN}=== 모터 제어 로그 ===${NC}"
    if [ -f "$LATEST_LOG" ]; then
        grep -E "모터|motor|position|speed|torque|joint" "$LATEST_LOG" | tail -n 100
    else
        echo "로그 파일이 없습니다."
    fi
}

show_stats() {
    echo -e "${GREEN}=== 시스템 상태 통계 ===${NC}"
    if [ -f "$LATEST_LOG" ]; then
        echo "로그 파일: $LATEST_LOG"
        echo "파일 크기: $(du -h $LATEST_LOG | cut -f1)"
        echo "총 라인 수: $(wc -l < $LATEST_LOG)"
        echo ""
        echo "에러 수: $(grep -c -E "ERROR|error" $LATEST_LOG 2>/dev/null || echo 0)"
        echo "경고 수: $(grep -c -E "WARN|warning|⚠️" $LATEST_LOG 2>/dev/null || echo 0)"
        echo ""
        echo "최근 업데이트: $(stat -c %y $LATEST_LOG | cut -d'.' -f1)"
        echo ""
        echo "실행 중인 프로세스:"
        ps aux | grep -E "position_control|iron_md_teleop|ezi_io_node" | grep -v grep
    else
        echo "로그 파일이 없습니다."
    fi
}

# 인자가 없으면 도움말
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi

# 옵션 파싱
case "$1" in
    -f|--follow)
        echo "실시간 로그 보기 (Ctrl+C로 종료)"
        echo "로그 파일: $LATEST_LOG"
        echo "----------------------------------------"
        tail -f "$LATEST_LOG"
        ;;
    -l|--list)
        show_list
        ;;
    -e|--errors)
        show_errors
        ;;
    -c|--can)
        show_can
        ;;
    -j|--joystick)
        show_joystick
        ;;
    -m|--motor)
        show_motor
        ;;
    -s|--stats)
        show_stats
        ;;
    -h|--help)
        show_help
        ;;
    *)
        echo "알 수 없는 옵션: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
