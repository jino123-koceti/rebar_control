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
        # ⚠⚠ [사용자 지시 2026-10-15] **배근 간격 기반 중복 제거.**
        #   `merge_px`(화소)·`dedup_mm`(25mm)로는 **51~59mm 떨어진 중복 오검출**이
        #   그대로 통과했다 (2026-10-15 실측, 사용자가 오검출로 확인).
        #   배근 간격이 양방향 200mm 로 일정하므로, **그 절반보다 가까운 두 검출은
        #   같은 교차점**일 수밖에 없다 — 진짜 교차점 둘을 합칠 위험이 원리적으로 없다.
        #   화소가 아니라 **mm 로 판정**한다: 화소 기준은 깊이에 따라 거리가 달라져
        #   먼 줄에서 진짜 점을 합칠 수 있다.
        #   0 이면 끈다.
        self.declare_parameter('grid_spacing_mm', 200.0)
        self.declare_parameter('frames', 3)            # 합칠 프레임 수
        self.declare_parameter('rate', 1.0)            # 0 이면 트리거로만 동작
        self.declare_parameter('depth_window', 5)      # 깊이 중앙값 창 (홀수)
        # ⚠⚠ [2026-10-16 사용자 지적] **depth 를 여러 장 모아 화소별 중앙값을 쓴다.**
        #   예전에는 `_on_depth` 가 마지막 1장만 들고 있어서, 그 한 장이 튀면
        #   그대로 깊이 오차가 됐다. 더 나쁜 건 **그 프레임에서 0 이면 그 교차점이
        #   통째로 버려진다**는 것이다 — depth 유효율이 57% 뿐이라 자주 일어난다.
        #   여러 장 중 하나만 값이 있으면 살아나므로 유효율도 같이 올라간다.
        #   ⚠ 지연은 없다 — 콜백에서 미리 쌓아 둔 것을 쓴다. 1 이면 옛 동작과 같다.
        self.declare_parameter('depth_frames', 5)      # 합칠 depth 장수 (1 = 끔)
        # 모델 검출(녹색) vs 실제 출력(빨강) 겹쳐 그린 그림. 빈 문자열이면 안 그린다.
        # ⚠ **절대경로여야 한다** — 상대경로는 노드의 작업 디렉터리(서비스 기준)에
        #   떨어져서 찾을 수 없다 (2026-10-16 에 실제로 그랬다).
        self.declare_parameter(
            'debug_png',
            '/home/koceti/ros2_ws/src/rebar_control/data/vision/detect_debug.png')
        self.declare_parameter('transform_yaml', '')   # 있으면 stage 좌표로 변환

        g = self.get_parameter
        self.weights = str(g('weights').value)
        self.threshold = float(g('threshold').value)
        self.merge_px = float(g('merge_px').value)
        self.grid_mm = float(g('grid_spacing_mm').value)
        self.n_frames = int(g('frames').value)
        self.win = max(1, int(g('depth_window').value) | 1)
        self.n_depth = max(1, int(g('depth_frames').value))
        from collections import deque
        self.depth_buf = deque(maxlen=self.n_depth)
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
            f"depth {self.n_depth}장 중앙값, "
            f"좌표계 {'stage' if self.R is not None else 'camera(캘리브레이션 전)'}"
            + ("" if rate > 0 else ", 트리거 전용(/rebar/detect)"))

        # ⚠⚠ **모델을 기동 직후 미리 적재한다.** 예전에는 첫 검출 요청 때 올렸고
        #   그게 약 20초 걸렸다 — `tying_planner` 의 검출 대기가 15초라 **그 날의
        #   첫 시도가 항상 실패**했다 (2026-10-07 실측: 19:56:15 타임아웃 →
        #   19:56:20 적재 완료 → 19:56:24 검출 9개). 전원 켜고 기다려도 안 풀린다 —
        #   적재가 **요청 전에는 시작되지 않기** 때문이다.
        # ⚠ 생성자에서 바로 부르지 않는다 — 20초 동안 spin 이 안 돌면 이 노드가
        #   죽은 것처럼 보인다. one-shot 타이머로 첫 spin 뒤에 올린다.
        self._warm = self.create_timer(0.1, self._warmup)

    def _warmup(self):
        '''기동 직후 한 번 — 모델을 미리 올린다.

        적재가 실패해도 노드는 살려 둔다 — `detect` 가 다시 시도한다.
        적재 실패를 기동 실패로 만들면 카메라가 늦게 붙는 경우에 노드가 사라진다.
        '''
        self.destroy_timer(self._warm)
        t0 = time.time()
        ok = self._ensure_model()
        self.get_logger().info(
            f"모델 예비 적재 {'완료' if ok else '실패'} — {time.time() - t0:.1f}초"
            + ('' if ok else ' (첫 검출에서 다시 시도한다)'))

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
        # ⚠ 원본 메시지가 아니라 **배열로** 쌓는다 — 뒤에서 한 번에 쌓으려면
        #   변환을 N 번 해야 하고, 해상도가 섞이면 stack 이 터진다.
        try:
            a = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return
        if self.depth_buf and self.depth_buf[0].shape != a.shape:
            self.depth_buf.clear()          # 해상도가 바뀌었다 — 섞지 않는다
        self.depth_buf.append(a.copy())

    def _on_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    # ---- 검출 --------------------------------------------------------------
    def _depth_stack(self):
        """최근 depth 들을 **화소별 중앙값**으로 합친 한 장.

        0 은 "못 쟀다" 는 뜻이라 평균에 넣으면 거리를 끌어내린다. 그래서 화소마다
        **0 이 아닌 값들만** 보고 중앙값을 낸다. 전부 0 인 화소만 0 으로 남는다.
        그래서 장수를 늘리면 깊이가 안정될 뿐 아니라 **유효 화소가 늘어난다.**
        """
        buf = list(self.depth_buf)
        if not buf:
            return self.bridge.imgmsg_to_cv2(self.depth,
                                             desired_encoding='passthrough')
        if len(buf) == 1:
            return buf[0]
        a = np.stack(buf).astype(np.float32)
        a[a <= 0] = np.nan
        with np.errstate(all='ignore'):
            med = np.nanmedian(a, axis=0)
        return np.nan_to_num(med, nan=0.0).astype(buf[0].dtype)

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
        dimg = self._depth_stack()

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
            # ⚠ [2026-10-15] 깊이를 빼고 화소 비례로 가는 안을 시험했다가 **되돌렸다.**
            #   이 장비는 카메라가 비스듬해 **스테이지 X 와 깊이의 상관이 -0.89** 다 —
            #   X 를 맞추는 정보가 사실상 깊이에 들어 있다. 빼면 LOO 가
            #   12.0 → 20.2mm 로 나빠졌다 (짝 47개, 다섯 모델 비교).
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

        # 배근 간격의 **절반**보다 가까운 검출끼리는 한 점으로 본다 (위 주석).
        #   신뢰도가 높은 쪽을 남긴다 — 낮은 쪽이 오검출일 확률이 높다.
        #   ⚠ **조용히 버리지 않는다.** 왜 점이 줄었는지 로그에 남아야 사람이 안다.
        if self.grid_mm > 0 and len(dets) > 1:
            lim = self.grid_mm / 2.0
            keep, dropped = [], []
            for d in sorted(dets, key=lambda e: -e.confidence):
                near = next((k for k in keep
                             if float(np.hypot(k.x - d.x, k.y - d.y)) < lim), None)
                if near is None:
                    keep.append(d)
                else:
                    dropped.append((d, near, float(np.hypot(near.x - d.x, near.y - d.y))))
            if dropped:
                self.get_logger().warning(
                    f"간격 중복 {len(dropped)}개 제외 (기준 {lim:.0f}mm = 배근 "
                    f"{self.grid_mm:.0f}mm 의 절반): "
                    + ", ".join(f"conf {d.confidence:.2f} ↔ {n.confidence:.2f} "
                                f"{g:.0f}mm" for d, n, g in dropped))
            dets = keep

        out.detections = dets
        out.valid = bool(dets)
        frame = 'stage' if self.R is not None else 'camera'
        out.error_message = (f"frame={frame}" if dets else
                             f"frame={frame} / 유효 검출 없음")
        self.pub.publish(out)
        self._status(f"검출 {total}개 중 깊이 확보 {len(dets)}개, 좌표계 {frame}")
        self._debug_png(bgr, merged, dets, scale, dimg)

    def _debug_png(self, bgr, merged, dets, scale, dimg):
        """모델이 찾은 것과 실제로 나간 것을 한 장에 겹쳐 그린다.

        ⚠⚠ [2026-10-16] 시연 직전에 **모델은 8~9개를 찾는데 출력이 4~6개**인
          상황이 나왔다. 어디서 버려지는지 토픽만 봐서는 알 수 없다 —
          `total_detected` 와 개수 차이로 "몇 개" 만 알 뿐 "어느 것" 을 모른다.
          그래서 두 집합을 겹쳐 그린다.
            **녹색** = 모델이 찾은 것 (merged)
            **빨강** = 실제로 나간 것 (dets)
          녹색만 있고 빨강이 없는 점이 **버려진 점**이고, 그 자리의 깊이를 같이
          적어 둬서 "깊이 0 이라 버려졌는지" 가 그림에서 바로 보이게 한다.
        """
        # 매번 다시 읽는다 — `ros2 param set` 으로 껐다 켤 수 있게 (재시작 불필요)
        png = str(self.get_parameter('debug_png').value or '')
        if not png:
            return
        try:
            import cv2
            im = bgr.copy()
            got = {(int(round(d.pixel_u)), int(round(d.pixel_v))) for d in dets}
            for u, v, name, conf in merged:
                ui, vi = int(round(u)), int(round(v))
                z = self._depth_at(dimg, ui, vi, scale)
                hit = any(abs(ui - a) <= 2 and abs(vi - b) <= 2 for a, b in got)
                cv2.circle(im, (ui, vi), 22, (0, 255, 0), 2)
                if hit:
                    cv2.circle(im, (ui, vi), 11, (0, 0, 255), -1)
                cv2.putText(im, f"{conf:.2f}", (ui + 26, vi - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(im, ("z%d" % z) if z > 0 else "z0!",
                            (ui + 26, vi + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0) if z > 0 else (0, 0, 255), 1)
            cv2.rectangle(im, (0, 0), (im.shape[1], 30), (0, 0, 0), -1)
            cv2.putText(im, f"green=model {len(merged)}   red=output {len(dets)}"
                            f"   dropped {len(merged) - len(dets)}",
                        (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1)
            cv2.imwrite(png, im)
        except Exception as e:                      # 디버그 그림이 검출을 막으면 안 된다
            self.get_logger().warning(f"디버그 그림 실패: {e}")

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
