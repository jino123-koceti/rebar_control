# 철근 결속 로봇 - 헤드리스 운영 가이드

## 📁 로그 시스템

### 로그 위치
- **디렉토리**: `/var/log/robot_control/`
- **최신 로그**: `/var/log/robot_control/control_latest.log` (심볼릭 링크)
- **로그 파일명**: `control_YYYYMMDD_HHMMSS.log`

### 로그 크기 관리
- **자동 정리**: 7일 이상 된 로그 자동 삭제
- **크기 제한**: 최대 500MB (초과 시 오래된 로그부터 삭제)
- **압축**: 7일 이상 된 로그 자동 gzip 압축

---

## 🚀 시스템 운영

### 자동 시작 (부팅 시)
시스템이 부팅되면 자동으로 `integrated_control_debug.sh`가 실행됩니다.

#### 실행되는 노드:
1. **RMD 모터 제어** (can2) - 7개 모터 제어
2. **EZI-IO 센서** (192.168.0.3) - 리미트 센서
3. **Iron-MD 텔레옵** (can3) - 무선 리모콘

#### 프로세스 모니터링:
- 5분마다 자동으로 프로세스 상태 확인
- 노드 종료 시 로그에 에러 기록

---

## 🔍 로그 확인 방법

### 1. 실시간 로그 보기
```bash
./view_logs.sh -f
# 또는
tail -f /var/log/robot_control/control_latest.log
```

### 2. 에러만 보기
```bash
./view_logs.sh -e
```

### 3. CAN 통신 로그
```bash
./view_logs.sh -c
```

### 4. 리모콘 입력 로그
```bash
./view_logs.sh -j
```

### 5. 모터 제어 로그
```bash
./view_logs.sh -m
```

### 6. 시스템 상태 확인
```bash
./view_logs.sh -s
```

### 7. 로그 파일 목록
```bash
./view_logs.sh -l
```

---

## 🧹 로그 관리

### 수동 로그 정리
```bash
./cleanup_logs.sh
```

### 자동 로그 정리 (cron 설정)
매일 새벽 2시에 자동 실행:
```bash
crontab -e
```
다음 줄 추가:
```
0 2 * * * /home/test/ros2_ws/cleanup_logs.sh >> /var/log/robot_control/cleanup.log 2>&1
```

---

## 📊 원격 접속 시 확인 사항

### SSH 접속 후:
```bash
# 1. 시스템 상태 확인
./view_logs.sh -s

# 2. 최근 에러 확인
./view_logs.sh -e

# 3. 프로세스 확인
ps aux | grep -E "position_control|iron_md_teleop|ezi_io_node"

# 4. CAN 인터페이스 확인
ip link show can2
ip link show can3
```

---

## 🛠️ 문제 해결

### 노드가 실행되지 않을 때
```bash
# 1. 로그 확인
./view_logs.sh -e

# 2. 수동으로 재시작
pkill -f "position_control|iron_md_teleop|ezi_io_node"
./integrated_control_debug.sh

# 3. CAN 인터페이스 재시작
sudo ip link set can2 down
sudo ip link set can3 down
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 250000
```

### 로그가 너무 클 때
```bash
# 강제 로그 정리
./cleanup_logs.sh

# 또는 수동 삭제 (7일 이상)
find /var/log/robot_control -name "*.log" -mtime +7 -delete
```

---

## 📝 로그 내용 필터링

### grep 명령어 활용
```bash
LOG_FILE="/var/log/robot_control/control_latest.log"

# 리모콘 입력만
grep '🎮 \[리모콘\]' $LOG_FILE

# ROS2 토픽 발행만
grep '📤 \[ROS2\]' $LOG_FILE

# CAN2 전송만
grep '📤 \[CAN2\]' $LOG_FILE

# cmd_vel 관련
grep 'cmd_vel' $LOG_FILE

# 위치 제어 관련
grep 'joint_' $LOG_FILE

# 에러/경고만
grep -E "ERROR|WARN|⚠️" $LOG_FILE

# 특정 모터 (예: 0x143)
grep '0x143' $LOG_FILE

# 특정 시간대 (예: 14:00-15:00)
grep '14:[0-5][0-9]' $LOG_FILE
```

---

## 🔐 보안 및 백업

### 로그 백업 (주기적)
```bash
# 외부 저장소로 복사
rsync -avz /var/log/robot_control/ /mnt/backup/robot_logs/

# 또는 압축 백업
tar -czf robot_logs_$(date +%Y%m%d).tar.gz /var/log/robot_control/
```

### 로그 읽기 권한
- 로그 파일은 `test` 사용자 소유
- 다른 사용자가 접근 필요 시: `sudo chmod 644 /var/log/robot_control/*.log`

---

## 📞 모니터링 체크리스트

### 매일 확인:
- [ ] 시스템 상태: `./view_logs.sh -s`
- [ ] 에러 로그: `./view_logs.sh -e`
- [ ] 디스크 사용량: `df -h /var/log`

### 매주 확인:
- [ ] 로그 크기: `du -sh /var/log/robot_control`
- [ ] 오래된 로그 정리: `./cleanup_logs.sh`
- [ ] 프로세스 재시작 (필요 시)

### 문제 발생 시:
1. 로그 확인: `./view_logs.sh -e`
2. 프로세스 상태: `./view_logs.sh -s`
3. CAN 인터페이스: `ip link show`
4. 수동 재시작

---

## 📦 파일 구조

```
/home/test/ros2_ws/
├── integrated_control_debug.sh    # 메인 실행 스크립트 (부팅 자동 실행)
├── view_logs.sh                    # 로그 뷰어
├── cleanup_logs.sh                 # 로그 정리 스크립트
└── HEADLESS_OPERATION.md           # 이 문서

/var/log/robot_control/
├── control_latest.log              # 최신 로그 (심볼릭 링크)
├── control_20251030_083000.log     # 타임스탬프 로그
├── control_20251029_120000.log.gz  # 압축된 오래된 로그
└── cleanup.log                     # 로그 정리 이력
```

---

## 💡 팁

1. **실시간 모니터링**: `watch -n 5 "./view_logs.sh -s"` (5초마다 상태 갱신)
2. **에러 알림**: `./view_logs.sh -e | mail -s "로봇 에러" admin@example.com`
3. **로그 스트림**: `./view_logs.sh -f | grep ERROR` (에러만 실시간 출력)
4. **원격 접속**: `ssh test@robot-ip "./view_logs.sh -s"`
