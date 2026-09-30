#!/usr/bin/env python3
"""L5 — Orbbec 교차점 검출. 컬러+depth 에서 결속 지점을 찾아 3D 좌표로 낸다.

## 무엇을 내는가

    /rebar/crossings (RebarGrid)  검출 지점들. 각 점은 RebarDetection:
        pixel_u, pixel_v   박스 **중심** (2차년도 좌표변환이 쓰는 기준)
        depth_mm           그 점의 깊이 (작은 창의 중앙값 — 한 픽셀은 튄다)
        x, y, z            3D 좌표 (mm)
        confidence         신뢰도

⚠ **`error_message` 에 좌표계가 적혀 있다.** 캘리브레이션 전에는 `frame=camera`
  (카메라 광학 좌표계) 이고, `stage_transform.yaml` 이 있으면 `frame=stage` 다.
  `RebarDetection.msg` 주석은 "로봇 프레임" 이라고 적혀 있지만, 변환을 못 뜬 상태에서
  로봇 좌표인 척하면 그게 사고로 이어진다. 그래서 명시한다.

## 모델

RF-DETR Medium (`cameras.yaml` 의 `model:`). **YOLO 가 아니다.**
클래스는 `['_', 'crossing', 'tie', 'untie']` — 인덱스가 1부터다.

**tie/untie 를 가리지 않는다.** 운용 방침이 "이중결속이더라도 결속하는 게 맞다" 이므로,
검출된 지점은 클래스와 무관하게 전부 결속 대상으로 본다. 같은 점에 tie·untie 가
동시에 떠도 `merge_px` 안이면 한 점으로 합치고 신뢰도가 높은 쪽 라벨을 남긴다.

## 왜 여러 프레임을 모으나

검출이 프레임마다 흔들린다. 2차년도도 3프레임을 합쳤다. 같은 지점이 여러 프레임에서
나오면 좌표를 평균 내어 안정시킨다.

## 깊이가 없으면 버린다

depth 가 0 인 픽셀이 40% 가량 된다(실측 유효율 57%). 깊이 없는 검출은 3D 로 못 바꾸니
결과에서 뺀다. 몇 개가 빠졌는지는 `total_detected` 와 개수 차이로 알 수 있다.
"""

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
from rebar_base_interfaces.msg import RebarDetection, RebarGrid


def load_model_cfg():
    """cameras.yaml 의 model 항목. 없으면 기본값."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    default = dict(path='src/rebar_control/model/new_weights_260930.pt',
                   class_names=['_', 'crossing', 'tie', 'untie'],
                   threshold=0.3, fp16=True, merge_px=25)
    try:
        p = os.path.join(get_package_share_directory('rebar_base_control'),
                         'config', 'cameras.yaml')
        m = (yaml.safe_load(open(p, encoding='utf-8')) or {}).get('model') or {}
        default.update({k: v for k, v in m.items() if v is not None})
    except Exception:
        pass
    return default


class CrossingDetector(Node):
    def __init__(self):
        super().__init__('crossing_detector')
        cfg = load_model_cfg()

        self.declare_parameter('weights', str(cfg['path']))
        self.declare_parameter('threshold', float(cfg['threshold']))
        self.declare_parameter('fp16', bool(cfg['fp16']))
        self.declare_parameter('merge_px', float(cfg['merge_px']))
        self.declare_parameter('frames', 3)            # 합칠 프레임 수
        self.declare_parameter('rate', 1.0)            # 0 이면 트리거로만 동작
        self.declare_parameter('depth_window', 5)      # 깊이 중앙값 창 (홀수)
        self.declare_parameter('transform_yaml', '')   # 있으면 stage 좌표로 변환

        g = self.get_parameter
        self.weights = str(g('weights').value)
        self.threshold = float(g('threshold').value)
        self.merge_px = float(g('merge_px').value)
        self.n_frames = int(g('frames').value)
        self.win = max(1, int(g('depth_window').value) | 1)
        self.classes = list(cfg['class_names'])

        if not os.path.isabs(self.weights):
            self.weights = os.path.join(os.path.expanduser('~/ros2_ws'), self.weights)

        self.bridge = None
        self.model = None
        self.color = None
        self.depth = None
        self.K = None                                  # fx, fy, cx, cy

        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Empty, '/rebar/detect', lambda m: self.detect(), 10)

        self.pub = self.create_publisher(RebarGrid, '/rebar/crossings', 10)
        self.status_pub = self.create_publisher(String, '/rebar/detector_status', 10)

        self.R, self.t = None, None
        self._load_transform(str(g('transform_yaml').value))

        rate = float(g('rate').value)
        if rate > 0:
            self.create_timer(1.0 / rate, self.detect)
        self.get_logger().info(
            f"교차점 검출 시작 — 모델 {os.path.basename(self.weights)}, "
            f"임계 {self.threshold}, {self.n_frames}프레임 합침, "
            f"좌표계 {'stage' if self.R is not None else 'camera(캘리브레이션 전)'}"
            + ("" if rate > 0 else ", 트리거 전용(/rebar/detect)"))

    # ---- 준비 --------------------------------------------------------------
    def _load_transform(self, path):
        if not path or not os.path.exists(path):
            return
        try:
            import yaml
            d = yaml.safe_load(open(path, encoding='utf-8')) or {}
            self.R = np.array(d['R'], dtype=float).reshape(3, 3)
            self.t = np.array(d['t'], dtype=float).reshape(3)
            self.get_logger().info(f"좌표변환 적용: {path}")
        except Exception as e:
            self.get_logger().error(f"좌표변환 읽기 실패 ({path}): {e}")

    def _ensure_model(self):
        if self.model is not None:
            return True
        try:
            import contextlib
            import io
            import rfdetr
            import torch
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                self.model = rfdetr.RFDETRMedium(
                    pretrain_weights=self.weights, num_classes=len(self.classes))
                if bool(self.get_parameter('fp16').value):
                    self.model.inference(dtype=torch.float16)
            self.get_logger().info("모델 적재 완료")
            return True
        except Exception as e:
            self.get_logger().error(f"모델 적재 실패: {e}")
            return False

    # ---- 입력 --------------------------------------------------------------
    def _on_color(self, msg):
        self.color = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    # ---- 검출 --------------------------------------------------------------
    def _depth_at(self, dimg, u, v, scale=(1.0, 1.0)):
        """작은 창의 중앙값. 한 픽셀만 읽으면 0 이나 튄 값을 잡는다.

        `scale` 은 컬러→depth 화소 비율이다. `depth_registration` 을 끄면 depth 가
        컬러보다 작게 나오는데(예: 컬러 1280x800, depth 640x400), 그대로 읽으면
        **큰 좌표가 범위를 벗어나 조용히 버려진다** — 화면 오른쪽·아래 검출이
        통째로 사라진다 (2026-09-30 무부하 시험에서 6개 중 4개가 이렇게 없어졌다).
        """
        h, w = dimg.shape
        u = int(round(u * scale[0]))
        v = int(round(v * scale[1]))
        r = self.win // 2
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = dimg[v0:v1, u0:u1]
        good = patch[patch > 0]
        return float(np.median(good)) if good.size else 0.0

    def detect(self):
        why = self._not_ready()
        if why:
            self._status(why)
            return
        if not self._ensure_model():
            self._status('모델 적재 실패')
            return

        import cv2
        bgr = self.bridge.imgmsg_to_cv2(self.color, desired_encoding='bgr8')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        dimg = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding='passthrough')

        # 여러 프레임 합치기 — 같은 영상에 대해 반복 추론해 흔들림을 줄인다
        raw = []
        for _ in range(max(1, self.n_frames)):
            det = self.model.predict(rgb, threshold=self.threshold,
                                     include_source_image=False)
            for (x1, y1, x2, y2), cid, conf in zip(det.xyxy, det.class_id,
                                                   det.confidence):
                cid = int(cid)
                name = self.classes[cid] if 0 <= cid < len(self.classes) else f'slot{cid}'
                raw.append([(x1 + x2) / 2.0, (y1 + y2) / 2.0, name, float(conf)])

        merged = self._merge(raw)
        total = len(merged)

        # 컬러와 depth 해상도가 다르면 화소 좌표를 환산해야 한다
        ch, cw = rgb.shape[:2]
        dh, dw = dimg.shape[:2]
        scale = (dw / float(cw), dh / float(ch))
        if (dw, dh) != (cw, ch) and not getattr(self, '_warned_size', False):
            self._warned_size = True
            self.get_logger().warning(
                f"컬러 {cw}x{ch} 와 depth {dw}x{dh} 해상도가 다릅니다 — 화소를 "
                f"{scale[0]:.3f}x{scale[1]:.3f} 로 환산합니다. "
                f"정확도를 위해 depth_registration:=true 를 권합니다")

        out = RebarGrid()
        out.grid_rows = 0            # 격자 정렬 안 함 (2차년도 2x3 개념을 쓰지 않는다)
        out.grid_cols = 0
        out.total_detected = min(255, total)
        fx, fy, cx, cy = self.K
        dets = []
        for u, v, name, conf in merged:
            ui, vi = int(round(u)), int(round(v))
            z = self._depth_at(dimg, ui, vi, scale)
            if z <= 0:
                continue                       # 깊이 없는 검출은 3D 로 못 바꾼다
            p = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=float)
            if self.R is not None:
                p = self.R @ p + self.t
            d = RebarDetection()
            d.pixel_u, d.pixel_v = ui, vi
            d.depth_mm = float(z)
            d.x, d.y, d.z = float(p[0]), float(p[1]), float(p[2])
            d.confidence = float(conf)
            d.camera_id = 0
            dets.append(d)

        out.detections = dets
        out.valid = bool(dets)
        frame = 'stage' if self.R is not None else 'camera'
        out.error_message = (f"frame={frame}" if dets else
                             f"frame={frame} / 유효 검출 없음")
        self.pub.publish(out)
        self._status(f"검출 {total}개 중 깊이 확보 {len(dets)}개, 좌표계 {frame}")

    def _not_ready(self):
        if self.color is None:
            return '컬러 영상 없음 (드라이버 확인)'
        if self.depth is None:
            return 'depth 영상 없음 (depth_registration 확인)'
        if self.K is None:
            return 'camera_info 없음'
        return None

    def _merge(self, raw):
        """merge_px 안의 검출을 한 점으로 합친다. 좌표는 평균, 라벨은 최고 신뢰도."""
        groups = []
        for u, v, name, conf in raw:
            for gp in groups:
                if abs(gp['u'] - u) <= self.merge_px and abs(gp['v'] - v) <= self.merge_px:
                    gp['pts'].append((u, v))
                    if conf > gp['conf']:
                        gp['conf'], gp['name'] = conf, name
                    gp['u'] = sum(p[0] for p in gp['pts']) / len(gp['pts'])
                    gp['v'] = sum(p[1] for p in gp['pts']) / len(gp['pts'])
                    break
            else:
                groups.append(dict(u=u, v=v, pts=[(u, v)], conf=conf, name=name))
        return [(g['u'], g['v'], g['name'], g['conf']) for g in groups]

    def _status(self, msg):
        self.status_pub.publish(String(data=msg))
        if msg != getattr(self, '_last_status', None):
            self._last_status = msg
            self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CrossingDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
