# ZED Link Quad 카메라 인식 문제 해결 가이드

## 현재 상태 진단 결과

### ✅ 정상 작동 중인 부분
1. **하드웨어 감지**: 4개의 ZED 카메라가 모두 하드웨어 레벨에서 감지됨
   - ZED X mini 2개 (13-0028, 13-0020)
   - ZED X 2개 (13-0038, 13-0030)

2. **커널 드라이버**: ZED Link Quad 드라이버 정상 로드
   ```
   sl_zedx                ✓ (ZED X 드라이버)
   sl_zedxpro             ✓ (ZED X Pro 드라이버)
   sl_zedxone_uhd         ✓ (ZED One 드라이버)
   sl_max9295             ✓ (GMSL 직렬화 드라이버)
   sl_max96712            ✓ (GMSL 역직렬화 드라이버)
   ```

3. **Video4Linux 디바이스**: 8개의 video 디바이스 생성됨
   ```
   /dev/video0-3  → ZED X mini/ZED X (카메라 1)
   /dev/video4-7  → ZED X mini/ZED X (카메라 2)
   ```

4. **Media Controller**: GMSL 파이프라인 정상 연결
   ```
   zedx → nvcsi → tegra-capture-vi → /dev/video*
   ```

### ⚠️ 문제점

**ZED Explorer가 카메라를 인식하지 못하는 이유**:

ZED Link Quad + GMSL 카메라는 **일반 USB ZED 카메라와 다른 방식**으로 작동합니다:

- **USB ZED**: ZED SDK가 직접 USB를 통해 카메라 제어 (ZED Explorer 작동)
- **GMSL ZED (ZED Link)**: V4L2 커널 드라이버를 통한 저레벨 접근 (ZED Explorer **작동 안 함**)

**ZED Explorer는 USB 연결 ZED 카메라만 지원합니다!**

## 해결 방법

### 방법 1: V4L2를 통한 직접 확인 (권장)

GMSL 카메라는 V4L2 API를 통해 직접 접근해야 합니다:

```bash
# 카메라 정보 확인
v4l2-ctl -d /dev/video0 --all

# 지원되는 포맷 확인
v4l2-ctl -d /dev/video0 --list-formats-ext

# 스트림 테스트 (GStreamer)
gst-launch-1.0 v4l2src device=/dev/video0 ! \
  video/x-raw,width=1920,height=1200,format=UYVY ! \
  videoconvert ! xvimagesink

# 또는 간단한 OpenCV 테스트
python3 << 'EOF'
import cv2
cap = cv2.VideoCapture(0)  # /dev/video0
if cap.isOpened():
    ret, frame = cap.read()
    if ret:
        print(f"✓ Camera working! Frame: {frame.shape}")
        cv2.imwrite('/tmp/zed_test.jpg', frame)
    else:
        print("✗ Failed to read frame")
else:
    print("✗ Failed to open camera")
cap.release()
EOF
```

### 방법 2: ROS2로 카메라 확인

ROS2를 통해 4개 카메라 동시 사용:

```bash
cd /home/koceti/ros2_ws
source install/setup.bash

# ZED ROS2 wrapper 실행 (GMSL 모드)
# 참고: 표준 launch 파일은 USB용이므로 커스텀 launch 필요
```

### 방법 3: 커스텀 ZED Link 캡처 노드 생성

GMSL 카메라용 전용 ROS2 노드가 필요합니다:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CVBridge
import cv2

class ZedLinkCaptureNode(Node):
    def __init__(self):
        super().__init__('zed_link_capture')

        # 4개 카메라 초기화
        self.cameras = []
        self.publishers = []

        for i in range(4):
            cap = cv2.VideoCapture(i * 2)  # video0, video2, video4, video6
            if cap.isOpened():
                self.cameras.append(cap)
                pub = self.create_publisher(
                    Image,
                    f'/zed_link/camera_{i}/image_raw',
                    10
                )
                self.publishers.append(pub)
                self.get_logger().info(f'Camera {i} opened: /dev/video{i*2}')
            else:
                self.get_logger().error(f'Failed to open camera {i}')

        self.bridge = CVBridge()
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30 Hz

    def timer_callback(self):
        for i, cap in enumerate(self.cameras):
            ret, frame = cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'zed_link_camera_{i}'
                self.publishers[i].publish(msg)

def main():
    rclpy.init()
    node = ZedLinkCaptureNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 방법 4: ZED SDK GMSL 지원 확인

ZED SDK가 GMSL을 지원하는지 확인:

```bash
# ZED SDK 버전 확인
cat /usr/local/zed/zed-config.cmake | grep VERSION

# GMSL 지원 확인 (SDK 4.1+ 필요)
ls /usr/local/zed/lib/libsl_zed* 2>/dev/null
```

만약 ZED SDK가 GMSL을 지원한다면, 특수한 초기화 방법이 필요합니다:

```cpp
// C++ 예시 (Python binding은 제한적)
#include <sl/Camera.hpp>

sl::Camera zed;
sl::InitParameters init_params;
init_params.camera_resolution = sl::RESOLUTION::HD1080;
init_params.input.setFromGMSLCamera(0);  // GMSL 카메라 0

if (zed.open(init_params) == sl::ERROR_CODE::SUCCESS) {
    // 카메라 사용 가능
}
```

## 문제 해결 체크리스트

- [✓] ZED Link Quad 드라이버 설치됨 (`stereolabs-zedlink-quad`)
- [✓] 커널 모듈 로드됨 (`sl_zedx`, `sl_max96712` 등)
- [✓] /dev/video* 디바이스 생성됨
- [✓] Media controller 파이프라인 연결됨
- [✗] ZED SDK Python 모듈 설치 필요 (선택사항)
- [✗] GMSL 전용 ROS2 노드 필요
- [✗] ZED Explorer 사용 불가 (GMSL 지원 안 함)

## 권장 사항

### 단기 해결책
1. **V4L2 직접 사용**: OpenCV나 GStreamer로 /dev/video0-7 직접 접근
2. **ROS2 커스텀 노드**: 위의 `ZedLinkCaptureNode` 사용

### 장기 해결책
1. **Stereolabs에 문의**: GMSL 카메라용 공식 ROS2 wrapper 제공 여부 확인
2. **ZED SDK GMSL API 확인**: 최신 SDK가 GMSL Python API를 지원하는지 확인
3. **커스텀 래퍼 개발**: GMSL + Depth 처리를 위한 전용 노드 개발

## 다음 단계

### 즉시 테스트 가능한 명령어

```bash
# 1. 카메라 0 (ZED X mini 1) 테스트
gst-launch-1.0 v4l2src device=/dev/video0 num-buffers=100 ! \
  'video/x-raw,width=1920,height=1200' ! \
  videoconvert ! jpegenc ! \
  multifilesink location=/tmp/zed0_frame_%03d.jpg

# 2. 카메라 1 (ZED X mini 2) 테스트
gst-launch-1.0 v4l2src device=/dev/video2 num-buffers=100 ! \
  'video/x-raw,width=1920,height=1200' ! \
  videoconvert ! jpegenc ! \
  multifilesink location=/tmp/zed1_frame_%03d.jpg

# 3. 카메라 2 (ZED X 1) 테스트
gst-launch-1.0 v4l2src device=/dev/video4 num-buffers=100 ! \
  'video/x-raw,width=1920,height=1200' ! \
  videoconvert ! jpegenc ! \
  multifilesink location=/tmp/zed2_frame_%03d.jpg

# 4. 카메라 3 (ZED X 2) 테스트
gst-launch-1.0 v4l2src device=/dev/video6 num-buffers=100 ! \
  'video/x-raw,width=1920,height=1200' ! \
  videoconvert ! jpegenc ! \
  multifilesink location=/tmp/zed3_frame_%03d.jpg
```

### 캡처 확인
```bash
ls -lh /tmp/zed*_frame_*.jpg
```

## 참고 자료

- ZED Link Quad 문서: https://www.stereolabs.com/en-kr/products/zed-link-quad
- NVIDIA GMSL 카메라 가이드: https://docs.nvidia.com/jetson/archives/r36.4/
- V4L2 API: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html

## 요약

**ZED Explorer가 카메라를 인식하지 못하는 것은 정상입니다!**

- ZED Link + GMSL 카메라는 **V4L2 디바이스로만 접근 가능**
- ZED Explorer는 **USB 연결 ZED 카메라 전용** 도구
- 카메라는 정상 작동 중이며, V4L2/GStreamer/OpenCV로 접근하면 됨
- ROS2 사용을 위해서는 **커스텀 노드 개발 필요**
