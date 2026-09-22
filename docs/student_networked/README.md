# WLAN 프레임 스트리밍 — 학생 교차점 검출 프로그램 원격 연동

학생의 ZED 로컬 카메라 코드를, **로봇(Jetson) 카메라를 WLAN으로 받아** 동일하게
돌아가도록 바꾸는 최소 변경 패키지. **검출 알고리즘 700여 줄은 그대로 두고
"카메라 입력부"만 교체**한다.

```
[로봇 Jetson (Linux)]                         [학생 PC (Windows)]
 Orbbec 카메라 (ROS2 토픽)                       main_networked.cpp
   /camera/color/image_raw  ─┐                    └ FrameClient (frame_client.hpp)
   /camera/depth/image_raw  ─┤                         │ TCP 요청 'G'
   /camera/color/camera_info┘                          ▼
        │                                        image_bgr (BGR)
        ▼   frame_stream_server.py               depth_f32  (mm, 무효=NaN)
   TCP :5001  ◀───────── WLAN ─────────────────  fx,fy,cx,cy
```

## 파일

| 파일 | 위치 | 실행 위치 |
|------|------|-----------|
| `frame_stream_server.py` | `ros2_ws/scripts/bridge/` | **로봇(Jetson)** |
| `frame_client.hpp` | `ros2_ws/docs/student_networked/` | 학생 PC |
| `main_networked.cpp` | `ros2_ws/docs/student_networked/` | 학생 PC |

## 로봇(Jetson) 측 준비

1. Orbbec 카메라는 robot-control 서비스가 자동 기동하므로 이미 떠 있음.
   (확인: `ros2 topic hz /camera/color/image_raw` → ~22Hz)
2. **뎁스-컬러 정합(D2C) 켜기** — 이 파이프라인은 뎁스로 만든 점을 *컬러 내참*으로
   이미지에 재투영해 컬러를 마스킹하므로, 뎁스가 컬러 프레임에 정합돼 있어야 픽셀이 맞다.
   `src/OrbbecSDK_ROS2/orbbec_camera/config/gemini2L.yaml`:
   ```yaml
   depth_registration: true   # (기존 false)
   ```
   변경 후 카메라 재기동.
3. 서버 실행:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash          # cv_bridge, message_filters 사용
   python3 scripts/bridge/frame_stream_server.py --port 5001
   ```
4. 로봇 IP 확인: 같은 공유기면 `192.168.0.x`, 원격이면 **Tailscale IP `100.99.144.107`**
   (rebar-jetson) 도 그대로 됨.

## 학생 PC(Windows) 측 준비

1. 학생 원본 파일(`결속위치코드_전재하`)을 **`rebar_pipeline.hpp`** 로 저장하고 두 곳만 수정:
   - 3행 `#include <sl/Camera.hpp>` **삭제** (ZED SDK 더 이상 안 씀)
   - 맨 아래 `int main(){ ... }` **전체 삭제** (구조체·함수·상수만 남김)
2. `frame_client.hpp`, `main_networked.cpp` 를 같은 폴더에 둔다.
3. `main_networked.cpp` 상단의 접속 정보 수정:
   ```cpp
   static const char* ROBOT_IP   = "192.168.0.10";  // 로봇 IP (또는 100.99.144.107)
   static const int   ROBOT_PORT = 5001;
   ```
4. 빌드 링크: **OpenCV + LibTorch**(기존과 동일) **+ ws2_32**. **ZED SDK 제거.**
   - CMake 예:
     ```cmake
     add_executable(rebar_net main_networked.cpp)
     target_link_libraries(rebar_net ${OpenCV_LIBS} ${TORCH_LIBRARIES} ws2_32)
     # find_package(ZED ...) 및 ZED 링크는 삭제
     ```
   - `frame_client.hpp` 안에 `#pragma comment(lib,"ws2_32.lib")` 가 있어 MSVC면 자동 링크됨.
5. 실행 → 기존과 동일하게 **Depth Map 창** 뜸. **스페이스바** = 파이프라인 실행/CSV·이미지 저장, **ESC** = 종료.

## 동작 방식 (요청/응답)

- 클라이언트 `grab()` 이 1바이트 `'G'` 전송 → 서버가 **최신 프레임 1장**만 회신.
  연속 push가 아니라 요청 기반이라 WLAN 지연/버퍼 적체가 없다.
- 서버는 컬러=**JPEG**(기본 q80), 뎁스=**16bit PNG 무손실**로 압축해 전송 → 대역폭 절감.
- 클라이언트가 복원한 `depth_f32` 는 **mm 단위, 무효 픽셀 = NaN** 으로,
  기존 ZED `retrieveMeasure(DEPTH)` 와 의미가 동일 → 알고리즘 수정 불필요.

## 알아둘 점 / 주의

- **뎁스 소스가 ZED(NEURAL) → Orbbec 으로 바뀜.** 뎁스 특성이 달라 학생 알고리즘의
  `DEPTH_MIN/MAX`, 평면·레이어 군집 파라미터를 **재튜닝**해야 할 수 있다.
  (전송 방식과 무관한 알고리즘 이슈. 우선 프레임 수신부터 확인 후 조정.)
- 해상도: ZED HD720(1280×720) → Orbbec 1280×800. 알고리즘은 `image.cols/rows` 를
  동적으로 쓰므로 대부분 무관하나, 하드코딩된 저장 경로/상수는 확인.
- **이건 "검출 개발용 원격 스트리밍"**이다. 최종 자율결속 통합은 별도 —
  학생 검출 결과(`intersection.csv` 의 픽셀+뎁스)를 로봇의
  `/rebar/detect_crossings` 서비스로 넘기는 계약은
  [collab_crossing_detection_interface.md](../collab_crossing_detection_interface.md) 참고.
- 방화벽: Windows에서 아웃바운드 5001 허용, 로봇에서 인바운드 5001 허용.
- 성능: 1280×800 JPEG+PNG16 한 프레임 ≈ 0.3~1MB. 요청형이라 스페이스바 눌러 처리하는
  현재 워크플로엔 충분. 실시간 고FPS가 필요하면 뎁스 다운스케일/ROI 전송을 추가.

## 왜 이 방식(TCP 브리지)인가

| 방식 | 장점 | 단점 |
|------|------|------|
| **TCP 브리지 (채택)** | Windows에 ROS2 불필요, 의존성 = Winsock+OpenCV(이미 있음), 카메라 종류 무관 | 서버/클라 소켓 코드 필요(이미 제공) |
| ROS2 on Windows | 표준 | Humble+rclcpp+cv_bridge 설치 무겁고, Win↔Linux DDS 까다로움 |
| ZED SDK 스트리밍 | 학생 코드 3줄만 변경 | **ZED 카메라에만** 가능. 우리 결속 카메라는 Orbbec이라 불가 |

우리 카메라가 Orbbec이고 학생은 Windows/C++ 최소 의존성이 필요하므로 TCP 브리지가 최적.
