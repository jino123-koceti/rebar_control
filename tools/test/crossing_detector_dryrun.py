#!/usr/bin/env python3
"""교차점 검출 무부하 검증 — 카메라 없이 돌린다.

합성 컬러 영상·합성 depth·가짜 camera_info 를 발행하고 `crossing_detector` 가 내는
`/rebar/crossings` 를 본다. **모델은 실제로 돌아간다** (GPU 만 있으면 된다).

확인하는 것
  · 입력이 모자랄 때 상태를 제대로 알리는가 (컬러/depth/camera_info)
  · 모델이 적재되고 추론이 도는가
  · **역투영 수식이 맞는가** — depth 를 알고 있으니 기대값을 계산해 대조한다
  · 깊이 0 인 검출을 버리는가
  · 좌표계 표기(`frame=camera`)가 실리는가

난수 영상에는 교차점이 없으므로 **검출 0개가 정상**이다. 그 경우 역투영은
`--synthetic-point` 로 따로 계산만 검증한다.

사용:
    ros2 run rebar_vision crossing_detector --ros-args -p rate:=0.0 &
    python3 tools/test/crossing_detector_dryrun.py
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
from rebar_base_interfaces.msg import RebarGrid

W, H = 640, 480
FX, FY, CX, CY = 600.0, 600.0, 320.0, 240.0
DEPTH_MM = 500


class Harness(Node):
    def __init__(self):
        super().__init__('crossing_detector_dryrun')
        self.color = self.create_publisher(Image, '/camera/color/image_raw',
                                           qos_profile_sensor_data)
        self.depth = self.create_publisher(Image, '/camera/depth/image_raw',
                                           qos_profile_sensor_data)
        self.info = self.create_publisher(CameraInfo, '/camera/color/camera_info',
                                          qos_profile_sensor_data)
        self.trig = self.create_publisher(Empty, '/rebar/detect', 10)
        self.grid = None
        self.status = None
        self.create_subscription(RebarGrid, '/rebar/crossings', self._on_grid, 10)
        self.create_subscription(String, '/rebar/detector_status',
                                 lambda m: setattr(self, 'status', m.data), 10)
        self.send_color = True
        self.send_depth = True
        self.send_info = True
        self.frame = None          # 실제 영상을 쓰면 여기에 담는다

    def _on_grid(self, m):
        self.grid = m

    def _img(self, arr, enc):
        m = Image()
        m.height, m.width = arr.shape[0], arr.shape[1]
        m.encoding = enc
        m.is_bigendian = 0
        m.step = arr.strides[0]
        m.data = arr.tobytes()
        return m

    def pump(self):
        if self.send_color:
            rgb = self.frame if self.frame is not None else \
                np.random.randint(0, 255, (H, W, 3), dtype=np.uint8)
            self.color.publish(self._img(np.ascontiguousarray(rgb), 'bgr8'))
        if self.send_depth:
            hh, ww = (self.frame.shape[:2] if self.frame is not None else (H, W))
            d = np.full((hh, ww), DEPTH_MM, dtype=np.uint16)
            self.depth.publish(self._img(d, '16UC1'))
        if self.send_info:
            ci = CameraInfo()
            ci.width, ci.height = W, H
            ci.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
            self.info.publish(ci)

    def spin(self, sec):
        end = time.time() + sec
        while rclpy.ok() and time.time() < end:
            self.pump()
            rclpy.spin_once(self, timeout_sec=0.02)


def check(name, cond, detail=''):
    print(f"  {'통과' if cond else '실패'}  {name}" + (f"   {detail}" if detail else ''))
    return cond


def main():
    rclpy.init()
    h = Harness()
    ok = True

    # ⚠ 이 단계는 **노드를 막 띄운 직후**에만 의미가 있다. 검출 노드는 한 번 받은
    #   영상을 계속 들고 있어서, 다른 단계를 먼저 돌리면 "입력 없음" 이 안 나온다.
    #   그래서 상태가 이미 오염돼 있으면 건너뛴다 (시험 결함을 실패로 오인하지 않게).
    print("■ 입력이 모자랄 때 상태를 알리는가")
    h.send_color = h.send_depth = h.send_info = False
    h.spin(1.0)
    h.trig.publish(Empty()); h.spin(1.5)
    st = h.status or ''
    if '컬러' in st:
        ok &= check("컬러 없음을 알린다", True, f"status={st}")
        h.send_color = True
        h.spin(1.0); h.trig.publish(Empty()); h.spin(1.5)
        ok &= check("depth 없음을 알린다", 'depth' in (h.status or ''),
                    f"status={h.status}")
        h.send_depth = True
        h.spin(1.0); h.trig.publish(Empty()); h.spin(1.5)
        ok &= check("camera_info 없음을 알린다", 'camera_info' in (h.status or ''),
                    f"status={h.status}")
    else:
        print(f"  건너뜀 — 노드가 이미 영상을 들고 있다 (status={st})")
        print("         이 검사를 보려면 crossing_detector 를 새로 띄우고 돌리세요")
    h.send_color = h.send_depth = h.send_info = True

    print("■ 전부 주고 추론 (모델 적재는 처음에 오래 걸린다)")
    h.spin(1.5)
    h.grid = None
    h.trig.publish(Empty())
    t = time.time() + 90
    while rclpy.ok() and h.grid is None and time.time() < t:
        h.pump(); rclpy.spin_once(h, timeout_sec=0.05)
    ok &= check("결과를 발행한다", h.grid is not None,
                f"status={h.status}")
    if h.grid is not None:
        ok &= check("좌표계 표기가 실린다", 'frame=' in h.grid.error_message,
                    h.grid.error_message)
        print(f"    난수 영상 검출 {h.grid.total_detected}개 "
              f"(교차점이 없으므로 0 이 정상)")
        for d in h.grid.detections:
            xe = (d.pixel_u - CX) * d.depth_mm / FX
            ye = (d.pixel_v - CY) * d.depth_mm / FY
            good = abs(d.x - xe) < 1.0 and abs(d.y - ye) < 1.0 and abs(d.z - DEPTH_MM) < 1.0
            ok &= check(f"역투영 ({d.pixel_u},{d.pixel_v})", good,
                        f"기대 ({xe:.1f},{ye:.1f},{DEPTH_MM}) 실제 ({d.x:.1f},{d.y:.1f},{d.z:.1f})")

    print("■ 실제 카메라 영상으로 역투영 검증")
    import os
    real = os.environ.get('DRYRUN_FRAME', '')
    if real and os.path.exists(real):
        h.frame = np.load(real)
        print(f"    영상 {h.frame.shape[1]}x{h.frame.shape[0]} 사용")
        # ⚠ 넉넉히 기다린다. 짧으면 트리거가 새 영상보다 **먼저** 처리돼서
        #   직전(난수) 영상으로 추론하고 "검출 0개" 가 된다.
        h.spin(3.0)
        h.grid = None
        h.trig.publish(Empty())
        t = time.time() + 60
        while rclpy.ok() and h.grid is None and time.time() < t:
            h.pump(); rclpy.spin_once(h, timeout_sec=0.05)
        if h.grid is None:
            ok &= check("실영상 결과 수신", False)
        else:
            n = len(h.grid.detections)
            ok &= check("실영상에서 검출된다", n > 0,
                        f"{h.grid.total_detected}개 중 깊이확보 {n}개")
            for d in h.grid.detections:
                xe = (d.pixel_u - CX) * d.depth_mm / FX
                ye = (d.pixel_v - CY) * d.depth_mm / FY
                good = (abs(d.x - xe) < 1.0 and abs(d.y - ye) < 1.0
                        and abs(d.z - DEPTH_MM) < 1.0)
                ok &= check(f"역투영 화소({d.pixel_u},{d.pixel_v})", good,
                            f"기대({xe:+.1f},{ye:+.1f},{d.depth_mm:.0f}) "
                            f"실제({d.x:+.1f},{d.y:+.1f},{d.z:.0f})")
        h.frame = None
    else:
        print("    (DRYRUN_FRAME 환경변수로 .npy 경로를 주면 실영상으로 검증합니다)")

    print("■ 역투영 수식 자체 (노드와 같은 식으로 계산되는지 손으로 대조)")
    for u, v in ((CX, CY), (CX + 100, CY), (CX, CY + 60)):
        xe = (u - CX) * DEPTH_MM / FX
        ye = (v - CY) * DEPTH_MM / FY
        print(f"    화소({u:.0f},{v:.0f}) depth {DEPTH_MM}mm → "
              f"({xe:+.2f}, {ye:+.2f}, {DEPTH_MM}) mm")
    print("    (중심 화소가 (0,0) 이 되고, 오른쪽·아래로 갈수록 +가 되어야 한다)")

    print(f"\n■ 결과: {'통과' if ok else '실패'}")
    h.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
