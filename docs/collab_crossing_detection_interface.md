# 상부제어 협업 인터페이스 스펙 — 교차점 인식(뎁스) 연동

> **협업 구도**: 우리 = 카메라 데이터 획득 + XYZ/yaw 스테이지 구동 + 결속 시퀀스.
> 학생 = 뎁스 기반 교차점 인식 알고리즘 (Windows / C++).
> 작성일: 2026-07-23

---

## 0. TL;DR (핵심 3줄)

- 학생은 **ROS를 몰라도 됩니다.** 순수 C++ 함수 하나(`detectCrossings`)만 작성 → Windows에서 PNG로 개발, Jetson에서 우리가 ROS2 노드로 래핑.
- 학생 출력은 **`픽셀(u,v) + depth(mm) + confidence`** 까지. 로봇 좌표 변환·모터·스테이지·결속은 **전부 우리 쪽**.
- 스테이지는 **절대 위치 제어**라 학생은 "교차점 좌표"만 주면 됨. 속도/궤적/안전한계는 우리가 관리.

---

## 1. 전체 데이터 흐름 (경계선)

```
[학생 담당 ─────────────────]        [경계 = 서비스]          [우리 담당 ──────────────────────────]
카메라 토픽 (color + depth)              /rebar/detect_crossings   RebarGrid → 카메라→로봇 변환 → 가동범위 필터
   ↓                                    (DetectCrossings.srv)      → mm→모터각(deg) → /joint_control(0x144~0x147)
detectCrossings(color, depth, K)  ───▶  RebarGrid 반환         ───▶ → XYZ/yaw 스테이지 절대위치 이동 + 결속
   → {u, v, depth_mm, confidence}                                   ↑ 호밍 선행조건·안전한계·결속 시퀀스 전부 캡슐화
```

- **좌표변환 소유권 = 옵션 A**: 학생은 픽셀+뎁스까지만, 카메라→로봇 변환은 우리가 소유.
  이유: 카메라↔로봇 외부 캘리브레이션은 지그 교체 때마다 바뀌므로 학생에게 부담을 넘기지 않음.
- 학생 노드는 **모터 각도(deg)를 절대 내보내지 않음.** mm→모터각 변환은 오케스트레이터 전용.

---

## 2. 카메라 데이터 포맷 (이 저장소 실측 기준)

하드웨어: **Orbbec Gemini 2L** (USB3), 드라이버 `src/OrbbecSDK_ROS2`, `gemini2L.launch.py`로 실행 (robot-control 서비스가 자동 기동).

| 스트림 | 토픽 | 포맷 | 해상도 |
|--------|------|------|--------|
| 컬러 | `/camera/color/image_raw` | **bgr8** (원본 MJPG 디코드) | 1280×800 |
| 뎁스 | `/camera/depth/image_raw` | **Y16 = `uint16`, 값 = 밀리미터(mm)** | 1280×800 |
| 내참 | `/camera/color/camera_info`, `/camera/depth/camera_info` | fx, fy, cx, cy, 왜곡계수 | — |

### ⚠️ 반드시 먼저 결정: 뎁스-컬러 정합 (depth registration)

- **현재 설정: `depth_registration: false`** (`gemini2L.yaml`). → **컬러 픽셀 (u,v)와 뎁스 픽셀 (u,v)가 같은 지점이 아님.**
- 학생 알고리즘이 "컬러에서 교차점 찾고 그 픽셀의 depth를 읽는" 방식이면 이 상태로는 어긋남.
- **권장 조치: `depth_registration: true`(D2C) 로 변경** → 픽셀 (u,v) 하나로 컬러·뎁스 동시 인덱싱, 학생 코드 단순화.
  (기존 호모그래피 경로는 뎁스를 안 써서 꺼놨던 것뿐. 뎁스 방식엔 켜야 함.)

> **결정 필요 항목**: `depth_registration`를 `true`로 켤지 여부.

---

## 3. 학생 인터페이스 계약 (실질적 계약 = 이 C++ 함수)

ROS 전혀 없음. Windows에서 PNG로 개발, Jetson(ARM64 Linux)에서 그대로 재컴파일.

```cpp
// detect_crossings.hpp  — 학생이 제공하는 유일한 계약
#include <opencv2/core.hpp>
#include <vector>

struct CameraIntrinsics {
    float fx, fy, cx, cy;        // 컬러 카메라 내참
    float k1, k2, p1, p2, k3;    // 왜곡계수 (미사용 시 0)
};

struct Crossing {
    float u, v;          // 컬러 이미지 픽셀 좌표 (교차점 중심)
    float depth_mm;      // 그 지점의 깊이 (mm)
    float confidence;    // 0.0 ~ 1.0
};

// 입력: 1280x800 bgr8 컬러 + 1280x800 uint16(mm) 뎁스(D2C 정합됨) + 내참
// 출력: 검출된 교차점들. 결속 순서(가까운 행부터, 좌→우)로 정렬해서 반환.
std::vector<Crossing> detectCrossings(const cv::Mat& color_bgr8,
                                      const cv::Mat& depth_mm_u16,
                                      const CameraIntrinsics& K);
```

### 정렬 규약 (RebarGrid 레이아웃, 위에서 본 모습 / 로봇 전방 = X+)

```
[0] [1] [2]    <- row 0 (가까운 쪽)
[3] [4] [5]    <- row 1 (먼 쪽)
```

### 학생 준수 사항 / 금지 사항

- ✅ `u, v, depth_mm, confidence`만 채우면 됨. 로봇 좌표·모터·스테이지는 몰라도 됨.
- ❌ **Win32 전용 API 사용 금지** (이식성 — CMake + OpenCV + C++17만).
- ❌ 로봇 좌표(mm), 모터 각도(deg), 스테이지 제어 토픽 출력 금지.

---

## 4. 협업 흐름 (2단계)

### ① 오프라인 개발 — 지금 당장, 로봇 없이

- 우리가 rosbag 녹화 → **PNG 쌍으로 export**:
  - `color_000123.png` — bgr8, 1280×800, 무손실
  - `depth_000123.png` — **16-bit PNG (uint16, mm)**. OpenCV `imread(path, IMREAD_UNCHANGED)` → `CV_16UC1`
  - `camera_info.json` — 내참 1장
  - (8비트로 저장 금지 — mm 정보 손실)
- 학생은 파일을 읽어 순수 C++/OpenCV로 Windows에서 개발·디버깅. ROS/로봇 불필요.
- 정답 라벨은 우리가 몇 장 찍어주면 검증 가능.

### ② 실기 통합 — 알고리즘 완성 후

- 학생 코드를 **ROS 무관 C++ 라이브러리**로 규약화 → **Jetson에서 우리가 얇은 ROS2 노드로 래핑**.
- 우리 래퍼 노드가 `detectCrossings()` 호출 → `/rebar/detect_crossings` 서비스로 노출 → 오케스트레이터가 소비.

> 참고: 학생이 자기 PC에서 실시간 루프를 돌려야 한다면 rclcpp(Windows 지원)로 DDS 직결(B2)도 가능하나, Win↔Linux DDS가 까다롭고 이 작업엔 과함. **"ROS 없는 C++ 함수 + 우리가 래핑"이 이 상황에 최적.**

---

## 5. ROS2 서비스/메시지 계약 (우리 래퍼 ↔ 오케스트레이터)

학생은 직접 볼 필요 없음. 우리 래퍼 노드가 채우는 계약.

**`src/rebar_base_interfaces/srv/DetectCrossings.srv`**
```
# Request
uint8   camera_selection       # 0=양쪽, 1=좌측만, 2=우측만
float32 confidence_threshold   # 최소 신뢰도 (기본 0.5)
uint8   expected_count         # 예상 교차점 수 (기본 6)
---
# Response
bool     success
RebarGrid grid                 # 정렬된 검출 그리드
string   message
float32  detection_time_ms
```

**`RebarGrid.msg`**: `RebarDetection[] detections`, `grid_rows`, `grid_cols`, `total_detected`, `valid`, `error_message`

**`RebarDetection.msg`** (점 1개):
```
float32 x, y, z        # 로봇 프레임 mm  (옵션 A: 우리 래퍼가 카메라→로봇 변환으로 채움)
float32 confidence
float32 depth_mm       # 원본 깊이 (학생 출력)
uint16  pixel_u, pixel_v   # 픽셀 좌표 (학생 출력)
uint8   camera_id      # 0=left, 1=right
```

---

## 6. 스테이지 구동 (우리 담당, 학생 참고용)

- **절대 위치 제어**: CAN 0xA4 멀티턴, `MODE_ABSOLUTE`. 목표 위치를 주면 이동 → 속도/궤적 계획 불필요.
- **대상이 정지 + 절대위치 제어 → 웨이포인트당 검출 1회면 충분** (실시간 스트리밍/피드백 루프 불필요 → 요청/응답 서비스 모델이 맞음).
- **mm→모터각 변환은 오케스트레이터 전용** (`_send_xy_absolute`). home_ref는 `/homing_status`의 `COMPLETE:{json}`에서 취득.

축 매핑 (결속부 정본 = `rebar_base_control`):

| 축 | CAN ID | deg/mm |
|----|--------|--------|
| X | 0x144 | 4.497 |
| Y | 0x145 | 4.462 |
| Z | 0x146 | 13.45 |
| Yaw | 0x147 | 1:1 (deg 직접) |

> ⚠️ `rmd_robot_control/position_control_node.py`는 **다른 축 매핑**(0x144=Z)을 쓰는 별도/레거시 스택. 참고 금지 — 정본은 `rebar_base_control`.

### 선행조건 / 안전 (통합 시)

- **호밍 선행**: 스테이지는 호밍 완료(`/homing_status`가 `COMPLETE:` 발행) 후에만 이동. (우리가 처리)
- **안전한계 주의**: X 345mm / Y 288mm는 **소프트웨어 숫자 캡이 아니라 물리 리밋 스위치**로만 걸림.
  → 검출점이 가동범위를 벗어난 좌표를 뱉으면 리밋 충돌 위험. **로봇 mm 범위 필터링을 우리 소비 코드에서 반드시 유지** (옵션 A라 자연히 우리 담당).

---

## 7. 결정 대기 항목 (진행 전 확인)

- [ ] **옵션 A 확정** (학생 = 픽셀+뎁스까지 / 우리 = 로봇 변환) — 권장.
- [ ] **`depth_registration: true`** 로 켤지 (뎁스 방식엔 권장).

### 확정되면 우리가 만들 것

1. **rosbag → PNG export 스크립트** (color PNG + depth 16-bit PNG + camera_info.json)
2. **학생용 C++ 인터페이스 헤더** `detect_crossings.hpp` + 샘플 `main.cpp` (PNG 로드 → 함수 호출 → 결과 출력)
3. **본 인터페이스 스펙 문서** (이 파일)
4. 실기 통합용 **ROS2 래퍼 노드** (`detectCrossings()` 호출 → `/rebar/detect_crossings` 서비스)

---

## 참고 파일 경로

- 서비스/메시지: `src/rebar_base_interfaces/srv/DetectCrossings.srv`, `msg/RebarGrid.msg`, `msg/RebarDetection.msg`
- 카메라 설정: `src/OrbbecSDK_ROS2/orbbec_camera/config/gemini2L.yaml`
- 기존 뎁스 검출 참고 구현(레거시): `src/rebar_vision/rebar_vision/rebar_detection_node.py`
- 오케스트레이터: `src/rebar_vision/rebar_vision/tying_orchestrator_node.py`
- 스테이지 mm→deg 변환: `src/rebar_base_control/rebar_base_control/homing_controller.py`, `can_sender.py`
- deg/mm 상수·리밋 채널: `src/rebar_base_control/config/can_devices.yaml`
