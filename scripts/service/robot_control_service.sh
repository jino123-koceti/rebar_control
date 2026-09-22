#!/bin/bash
# 철근 결속 로봇 통합 제어 시스템 (Full System)
# HAL(base_system) + 상위 제어(control_system) + ZED 카메라 전체 실행

# 로그 디렉토리 설정
LOG_DIR="/var/log/robot_control"
mkdir -p $LOG_DIR 2>/dev/null || true
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/control_${TIMESTAMP}.log"

log_msg() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $LOG_FILE
}

log_msg "========== 철근 결속 로봇 전체 시스템 시작 (Full System) =========="

# ROS2 환경 설정
cd /home/koceti/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

log_msg "ROS2 환경 설정 완료"

# Orbbec Gemini 2L - 별도 프로세스로 실행 (ZED와 camera_container 이름충돌 회피)
# 같은 launch에 넣으면 orbbec 노드가 ZED 컨테이너로 로드돼 스트림 안 됨 → 독립 프로세스
log_msg "Orbbec 카메라 별도 실행 (background)"
# depth_registration:=true → 뎁스를 컬러 프레임에 정합(D2C). yaml이 중첩구조라
# 값이 안 먹혀서(launch 평면병합 한계) launch 인자로 직접 넘긴다. (검출 depth-at-pixel 정합용)
# ⚠️ [2026-08-06] serial_number 필수! 305(좌측)도 Orbbec이라, 2L이 serial 없이 뜨면
#   두 노드가 장치를 놓고 경쟁해 305가 "not found" 실패함. 각자 자기 시리얼만 열게 고정.
#   2L=CPAX1630093, 305=CV2L3600003P.
# ⚠️ [2026-08-13] CPU 부하 감축: 이 컨테이너가 단일 최대 소비자였다(86.6%, 12코어 중).
#   결속 검출은 **정지 상태에서 몇 프레임만** 쓰므로 30fps가 불필요 → 5fps로 낮춤.
#   point_cloud도 미사용(검출은 depth-at-pixel만 씀) → off.
#   depth_registration은 CAD 변환에 필수라 유지.
ros2 launch orbbec_camera gemini2L.launch.py depth_registration:=true serial_number:=CPAX1630093 \
    color_fps:=5 depth_fps:=5 enable_point_cloud:=false >> $LOG_FILE 2>&1 &
#ros2 launch orbbec_camera femto_bolt.launch.py depth_registration:=true >> $LOG_FILE 2>&1 &
ORBBEC_PID=$!
log_msg "Orbbec PID=$ORBBEC_PID (서비스 종료 시 cgroup으로 함께 종료됨)"
sleep 3

# Orbbec Gemini 305 (좌측 횡이동 주행가능영역 판별용) - USB라 GMSL/nvargus 부하 없음.
#   [2026-08-06] 프리즈 무죄 확정(305 없이도 프리즈). 이전 uvc_open -6 무한루프는 USB 케이블/포트
#   교체로 해결됨(정상 28.9Hz 오픈 확인). 좌측 zedxmini GMSL→305 USB로 이전(GMSL 3대로 감축).
# ★ [2026-09-14] **305를 ZED X One 4K(SN 319430526)로 교체 → 이 블록 비활성화.**
#   장치가 빠졌는데 노드는 계속 떠서 5초마다 재시도를 반복하고 있었다:
#     `Device with serial number CV2L3600003P not found`  ← 로그를 채우고 CPU만 먹는다
#   좌측 판정은 이제 full_system.launch.py 의 `zedxone_left` 가 맡는다.
#   ⚠ 305로 되돌릴 일이 있으면 아래 주석을 풀고, deck_edge 의 `side_left_topic` 을
#     `/camera_left/color/image_raw/compressed` 로, `side_left_rotate` 를 180 으로 되돌릴 것
#     (305는 배선상 거꾸로 장착돼 있었다).
# log_msg "Orbbec Gemini 305 (좌측) 별도 실행 (background)"
# # ⚠️ [2026-08-13] CPU 부하 감축: 측면 주행가능 판정은 **RGB seg만** 쓴다(depth 미사용,
# #   4단계 실측으로 rebar_frac만으로 충분함 확인). → depth/point_cloud off, 5fps.
# #   판정은 횡이동 직전 몇 프레임만 필요하므로 저fps로 충분.
# ros2 launch orbbec_camera gemini305.launch.py \
#     camera_name:=camera_left serial_number:=CV2L3600003P \
#     enable_depth:=false enable_point_cloud:=false color_fps:=5 >> $LOG_FILE 2>&1 &
# ORBBEC_LEFT_PID=$!
# log_msg "Orbbec 305 left PID=$ORBBEC_LEFT_PID (cgroup 함께 종료)"

# ⚠️ 305가 케이블 배선상 거꾸로(180°) 장착됨. Gemini 305 드라이버는 color_rotation/flip/mirror를
#   지원 안 함(silently skip 확인).
# ⚠️ [2026-08-13] **회전 재발행 노드 제거** — 매 프레임을 통째로 회전·재발행하느라 CPU 24%를
#   먹고 있었다(12코어 중). 회전은 소비자(측면 판정)가 판정 순간에 cv2.rotate 한 번(수 ms)만
#   하면 되므로, 상시 노드로 돌릴 이유가 없다.
#   → 소비자는 /camera_left/color/image_raw 를 구독하고 side_rotate 파라미터로 회전 처리.
#   (복귀 필요 시: ros2 run rebar_vision image_rotate --ros-args
#     -p input_topic:=/camera_left/color/image_raw
#     -p output_topic:=/camera_left/color/image_rotated -p rotation:=180)

# 프레임 스트리밍 서버 - 학생 PC로 컬러+뎁스 WLAN(TCP :5001) 전송 (background)
# Orbbec 토픽 구독 → 요청 시 최신 프레임 전송. 토픽 대기하므로 카메라보다 먼저 떠도 무방.
log_msg "프레임 스트리밍 서버 실행 (background, TCP :5001)"
python3 scripts/bridge/frame_stream_server.py >> $LOG_FILE 2>&1 &
FRAME_SERVER_PID=$!
log_msg "frame_stream_server PID=$FRAME_SERVER_PID (서비스 종료 시 cgroup으로 함께 종료됨)"

log_msg "실행: ros2 launch rebar_control full_system.launch.py"
# 전체 시스템 launch 실행 (HAL + 상위 제어 + ZED)
# [2026-08-06] 프리즈 원인이 카메라 아님 확정(0x92·CAN경로·동글·모터 전부 배제→주행 고전류/진동 하드웨어).
#   → 좌우 결속 카메라(zedxmini1/2) 원복. use_zedxmini 기본값 true.
#   [2026-08-06] zedxmini1/2를 좌/우 측면 → 전방/후방으로 물리 이동 (zedxmini2=전방, zedxmini1=후방).
#   ※ 시야 스냅샷으로 방향 확인 완료 — zedxmini2가 배근 진행방향을 본다.
#   zed_front/back(ZED X)은 비활성이고 deck_edge front/back_topic이 zedxmini로 재지정됨.
#   좌측 측면 관측은 USB Orbbec 305(camera_left)가 대체.
#   [2026-08-14] GMSL 대수 격리 실험 **종료·기각**: `use_zedxone:=false`로 3대→2대로
#     줄여봤으나 **오히려 20초 만에 사망**. 카메라 수는 프리즈와 무관하다는 결론.
#     → zedxone 복구(우측 횡이동 판정용). ([[robot_freeze_safety]])
#
# ★ [2026-08-18] `use_deck_edge:=true` 상시 기동.
#   왜: launch 기본값이 false여서(8월 초 "미검증 단계" 판단) 매번 손으로 띄워야 했고,
#       빠뜨리면 주행노드가 8초 뒤 `front 판정 수신 없음`으로 ABORT됐다(실제 발생).
#       그 사이 실주행으로 여러 번 검증됐고 판정 시각화(deck_edge_debug)로도 확인됨.
#   비용: CPU 약 47%(12코어 중 1코어분), GPU는 거의 안 씀(GR3D 0%).
#         프리즈 원인이 전원 결합으로 밝혀져 부하를 아낄 이유도 줄었다.
#   ⚠ 이걸 켜면 `drive_controller`가 데크 밖 방향 cmd_vel을 차단한다 —
#     **리모콘 수동주행도 배근 끝에서 막힌다.** 의도된 안전망이지만, 시험 중
#     예상 못한 정지가 나면 이것부터 의심할 것. 끄려면 `use_deck_edge:=false`.
#   ※ `rebar_drive`(자율주행 노드)는 **일부러 안 켠다** — 실행 인자를 매번 바꿔가며
#     쓰는 노드라(모델·tie_classes·tie_every_n·speed…) 서비스에 고정하면 파라미터를
#     바꿀 때마다 서비스를 재시작해야 한다. `AUTONOMOUS_RUN.md` 참조.
# ★ [2026-09-08] 전·후방을 ZED X Mini → **ZED X** 로 교체.
#   연결된 시리얼 45320958 / 46674448 은 launch의 `zed_front`/`zed_back`(use_zed)
#   항목과 **같은 카메라**다. 그래서 use_zedxmini 쪽을 그 시리얼로 돌려놓고
#   `use_zed:=false` 로 중복 기동을 막는다 — 둘 다 켜면 같은 카메라를 동시에
#   열려다 둘 다 실패한다.
#   ⚠ 네임스페이스는 zedxmini1/2 를 **유지**한다. deck_edge·수집도구·문서 등
#     여러 곳이 이 토픽 이름을 쓰고 있어, 이름 변경은 현장 일정 뒤로 미룬다.
# ★ [2026-09-09] 주행모터 속도계획 가감속을 **매 기동마다** 맞춘다.
#   왜: **X4-36은 0x43 쓰기가 ROM에 안 남는다**
#     (실측: 1200으로 쓰고 확인값도 1200이었는데 재부팅 후 공장기본 5000으로 원복).
#     재적용을 안 하면 공장기본 5000으로 돌아가, 매번 다시 튜닝한 값을 잃는다.
#   ⚠ launch보다 **먼저** 해야 한다 — can_sender가 버스를 폴링하기 시작하면
#     0x42/0x43 응답에 폴링 프레임이 섞여 읽기가 불안정해진다.
#   실패해도 주행은 가능하므로 `|| true` 로 서비스를 막지 않는다.
#   ★ [2026-09-14] **양쪽 다 X4-36으로 통일** → 좌우를 다시 **같은 값(1200)** 으로.
#     이전엔 일부러 비대칭(0x141=1200 / 0x142=500)이었다. X4-36↔X4-10 혼합 시절
#     서로 다른 응답을 상쇄해 **실제 램프**를 맞추려던 보정값이다(현장 실측).
#     같은 모델이 된 지금 그 보정을 남겨두면 **우측만 2.4배 굼뜬 램프**가 되어
#     가감속할 때마다 차체가 한쪽으로 틀어진다. → 보정을 걷어낸다.
#     1200은 혼합 시절 X4-36(0x141)에서 실제 램프가 검증된 값이라 그대로 승계한다.
#     ⚠ 이제 **양쪽 다** ROM에 안 남는다(X4-36 공통) → 둘 다 매 기동 재적용 필수.
#     ⚠ 한쪽만 X4-10으로 되돌리면 그쪽을 500으로 (비대칭 복구).
#   ★ [2026-09-14 저녁] **가속과 감속을 분리**한다. 전엔 `--set 1200` 한 번으로
#     index 2(가속)·3(감속)을 **둘 다** 1200으로 썼다. 그런데 둘은 성격이 반대다:
#       · 가속 1200 = 전류 피크를 누르려고 낮춰둔 값 → **올리면 안 된다**
#         ([[robot_freeze_safety]]: 피크 28.3A에서 프리즈 재현)
#       · 감속 1200 = 정지 지연의 **48%** 를 차지 (0.19m/s에서 62mm 중 30mm)
#         감속은 회생이라 전류 피크 문제가 없다 → 올려도 안전
#     → 감속만 3000으로. 0.19 m/s 정지거리 62mm → 26mm (슬루 2.5와 함께).
#     ⚠ X4-36은 0x43이 ROM에 안 남아 **매 기동 재적용**이 필수다. 값을 바꾸려면
#       여기와 `check_drive_pair.py` 의 TARGET_ACCEL 을 **같이** 고칠 것.
#   ★★ [2026-09-14 저녁 · 실측] **0x43은 출력축이 아니라 모터축 기준이다.**
#     지령을 3초 고정했을 때 실제 가속도가 35.7 출력축 dps/s → 1200÷36 = 33.3 과 일치.
#     (출력축 해석이면 1200이라 34배, 슬루 0.25m/s²면 500이라 14배 어긋난다.)
#     ⟹ X4-10(12.5:1) → X4-36(36:1) 교체로 **같은 설정값의 실효 램프가 1/2.9** 이 됐다:
#         X4-10 시절 1200 → 출력축 96 dps/s (0→최고속 5.2초)
#         지금   1200 → 출력축 33 dps/s (0→최고속 14.9초, 정지도 180mm 밀림)
#     혼합 시절 현장에서 맞췄던 비대칭 1200/500 = 2.4배도 기어비 2.88배가 정체였다.
DRIVE_ACCEL=3456    # = 96 출력축 dps/s × 36. X4-10 시절 **검증된 출력축 램프**를 복원.
                    #   ⚠ 이 이상 올리지 말 것 — 전류 피크가 프리즈와 겹쳤던 지점이다
                    #     ([[robot_freeze_safety]] 2026-08-14, 피크 28.3A).
DRIVE_DECEL=12000   # = 333 출력축 dps/s. 감속은 회생이라 전류 피크 제약이 없다.
                    #   0.078 m/s 에서 정지거리 180mm → 약 18mm.
if [ -e /sys/class/net/can2 ]; then
    log_msg "주행모터 가감속 정렬 (0x141/0x142 가속 ${DRIVE_ACCEL} / 감속 ${DRIVE_DECEL} dps/s)"
    for _drive_mid in 0x141 0x142; do
        timeout 60 python3 /home/koceti/ros2_ws/tools/motor/rmd_accel.py --ids $_drive_mid \
            --index 2 --set $DRIVE_ACCEL \
            >> $LOG_FILE 2>&1 || log_msg "  ⚠ $_drive_mid 가속 정렬 실패(무시하고 계속)"
        timeout 60 python3 /home/koceti/ros2_ws/tools/motor/rmd_accel.py --ids $_drive_mid \
            --index 3 --set $DRIVE_DECEL \
            >> $LOG_FILE 2>&1 || log_msg "  ⚠ $_drive_mid 감속 정렬 실패(무시하고 계속)"
    done
fi

# ★ [2026-09-08] `use_auto_tying:=true` — UI 버튼용 실행기.
#   실행기 자체는 아무것도 안 하고, /mission/command 로
#   {"command":"AUTO_TYING","direction":"FWD"|"REV"} 가 오면 그때 rebar_drive를
#   대신 띄운다. 파라미터를 바꿔야 하면 실행기 파라미터(arm/tie_every_n/speed…)만
#   고치면 되고, 위 주석의 "매번 바꿔 쓰니 서비스에 고정 안 한다"는 이유가 해소된다.
#   ⚠ `use_rebar_drive:=true` 와 같이 켜지 말 것 — rebar_drive가 둘이 된다.
#   ⚠ `rebar_drive_arm` 기본이 false(dry-run)라 실제 구동하려면 여기서 true로.
# ★ [2026-09-14] `use_zedxone_left:=true` — **GMSL 4대 동시 시험 중**.
#   되돌리려면 이 줄에서 그것만 지우면 3대 구성(스테레오2 + 우측모노)으로 돌아간다.
#   ⚠ 시험은 **정지 상태에서** 할 것. 확인:
#       python3 tools/vision_test/gmsl4_watch.py --min 5
#   실패 이력(둘 다 "돌아가는 중에 추가로 열기"): 3대 가동 중 4번째 임시기동 →
#   전방 스테레오 사망 / deck_edge on-demand 여닫기 → 스테레오 2대 사망.
#   직접 원인은 대수가 아니라 **자식의 비정상 종료가 Argus 소켓을 끊은 것**이었다.
#   런치 시점 동시 오픈은 그 경로를 안 타므로 될 가능성이 있다 — 그걸 재는 중.
# ★★ [2026-09-22] **4대 동시 시험 결과: 실패 → 3대 구성으로 복귀** (use_zedxone_left 제거).
#   4대일 때 3번 모두 무너짐(좌측 모노 NOT INITIALIZED + 스테레오 CORRUPTED/REBOOTING),
#   가동 후 6~26분, 로봇이 **정지해 있어도** 발생. 3대(좌측 죽어 있던 11:30~19:30)는
#   횡이동 24회·자율결속 2회 동안 무사. 횡이동·모터 스위치와 겹친 건 우연으로 봄.
#   좌측 판정이 없으니 auto_tying_launcher `lateral_fixed_dir='right'`(좌측 끝에서 출발).
#   ▶ 4대 장시간 시험은 좌측 모노가 부팅 때 안 잡혀(캡처카드 전원 차단 필요) 못 함.
#     좌측은 USB 웹캠(HCAM01L 720p 고정초점)으로 교체 예정 → 3대 구성 유지.
ros2 launch rebar_control full_system.launch.py use_zedxmini:=true use_zed:=false use_deck_edge:=true \
    use_zedxone:=true \
    use_auto_tying:=true rebar_drive_arm:=true rebar_drive_tying:=true \
    rebar_drive_lateral:=true \
    use_data_acq:=true 2>&1 | tee -a $LOG_FILE
#   ↑ `rebar_drive_lateral` 이력: 2026-09-16 껐다가 **2026-09-18 되살림**.
#     껐던 이유: 측면 레벨봉 판정이 **seg 오검출**로 좌·우 모두 '불가'를 내
#       (rebar_frac 1.0 인데 레벨봉 0.99/0.73) 시작하자마자 커버리지 종료됐다.
#     지금 괜찮은 이유: `level_rod_use_seg` 기본값을 **False** 로 바꿔(2026-09-16)
#       측면도 **색 기반만** 쓴다. 오검출의 범인이던 seg 소스가 빠졌다.
#     ⚠ 남은 위험: 좌측 측면은 색 기반을 못 믿는다(녹슨 철근이 봉보다 노랗다 —
#       애초에 seg 를 넣은 이유). 즉 **좌측 레벨봉을 놓칠 수 있다.**
#       'ㄹ'자 주행 중 좌측에 레벨봉이 있으면 사람이 봐줄 것.
#       근본 해결은 `level_rod` 학습데이터 보강 → seg 되살리기.
