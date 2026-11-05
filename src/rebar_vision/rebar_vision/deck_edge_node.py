#!/usr/bin/env python3
"""L2 — 전·후방 카메라로 **주행해도 되는지**를 판정한다.

세그멘테이션 추론 **한 번**으로 세 가지를 낸다:

    deck_edge   발밑 배근이 이어지는가      → GO / SLOW / STOP
    corridor    장비 폭 띠로 몇 mm 갈 수 있나 → 직진 가능 거리
    rebar_grid  가로철근 기울기              → heading 오차

## 왜 이 노드가 필요한가

`/deck_edge_block` 은 `safety_node` 가 구독만 하고 **발행자가 없었다**. 그
빈자리를 `coverage_map` 의 `explore_ring` 이 "본 적 없는 600mm 를 지어내서"
메우고 있었다. 주행 가능 여부가 센서에서 나와야 그 지어내기를 없앨 수 있다.

## 토픽

    구독  <front>/rgb/..., <front>/depth/...  (+ back)
    발행  /deck_edge_block   String(JSON) {"block": ["forward", ...], "reason": ...}
          /deck_edge/status  String(JSON) 방향별 상세 — 거리·heading·평면

## 운용 규칙 — 2차년도에서 가져온 것

**전·후방을 교대로 추론한다.** 진행 방향만 보면 반대 방향 판정이 묵어서
**탈출 방향까지 막힌다.** (2차년도 deck_edge_node 가 같은 결론)

**차단은 방향별이다.** 전 방향을 막으면 빠져나올 수단이 사라져 그 자리에
갇힌다. `SafetyState` 도 같은 이유로 방향별 필드를 갖는다.

## 함정

⚠ **ZED 는 `bgra8`(4채널)로 발행한다.** 무조건 `COLOR_RGB2BGR` 를 걸면 R·B 가
  뒤바뀌고, 파란 방수포가 갈색으로 찍힌다. 예외도 경고도 없이 세그멘테이션이
  망가진다 — 원거리 철근 검출이 반토막 나고 장애물 클래스가 바뀐다.
  (2026-11-05 에 이걸로 반나절 날렸다.)

⚠ **클래스 인덱스를 하드코딩하지 않는다.** 7→9클래스에서 rebar_h 가 4→6 으로
  밀렸다. `set_class_map()` 을 로드 직후 부르지 않으면 판정이 통째로 뒤집혀
  데크 밖으로 나간다.

⚠ **`plate` 는 차단물이 아니다.** 바닥 철판이고, 실제로 맨 합판을 그렇게
  오검출한다. 납작한 것은 corridor 의 높이 필터가 거른다.
"""

import json
import os
import time

import numpy as np
import cv2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

from rebar_vision.rebar_seg import RebarSegmenter
from rebar_vision import deck_edge as de
from rebar_vision import rebar_grid as rg
from rebar_vision import corridor as co

DEFAULT_WEIGHTS = 'src/rebar_control/model/rebar_seg_260914.pt'
# 바닥 평면을 맞출 클래스. 테스트베드는 방수포(background)가 가장 깨끗하다
# (잔차 14.7mm, 철근으로 맞추면 28.4mm). ⚠ 현장 데크는 아연도금 강판이라
# 이 전제가 안 맞는다 — 현장에서 파라미터로 바꿀 수 있게 열어 둔다.
DEFAULT_PLANE_CLASSES = 'background'
DEFAULT_BLOCK_CLASSES = 'obstacle,human,wall'


def to_bgr(msg):
    """ROS Image → BGR. **인코딩을 반드시 본다** (머리말 함정 참고)."""
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    e = msg.encoding
    if e == 'bgra8':
        return a[:, :, :3].copy()
    if e == 'bgr8':
        return a.copy()
    if e == 'rgba8':
        return cv2.cvtColor(a, cv2.COLOR_RGBA2BGR)
    if e == 'rgb8':
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    raise ValueError(f'모르는 인코딩: {e}')


class Side:
    """한 방향(전 또는 후)의 최신 입력과 판정 상태."""

    def __init__(self, name, median_n):
        self.name = name
        self.rgb = None
        self.depth = None
        self.K = None
        self.fsm = de.VerdictFSM()
        self.plane = None
        self.plane_t = 0.0
        self.hist = []              # 최근 go_mm (중앙값용)
        self.median_n = median_n
        self.last = {}
        self.stamp = 0.0

    def ready(self):
        return self.rgb is not None and self.depth is not None and self.K is not None

    def push(self, go_mm, limit):
        """최근 N프레임의 중앙값과 **그 프레임의 사유**를 함께 돌려준다.

        ⚠ 거리만 중앙값을 내고 사유는 최신 프레임에서 가져오면 둘이 다른
          프레임을 가리킨다 (첫 가동에서 `go 1675 / limit unknown` 이 나왔다).
          보고가 서로 안 맞으면 로그를 읽고 원인을 못 찾는다.
        """
        self.hist.append((go_mm, limit))
        if len(self.hist) > self.median_n:
            self.hist.pop(0)
        got = [x for x in self.hist if x[0] is not None]
        if len(got) < max(1, len(self.hist) // 2 + 1):
            # 과반이 '거리 없음' = 통로 안에 장애물이 없다는 뜻이다.
            # 'unknown' 으로 적으면 "판정 실패" 로 읽혀 로그가 오해를 부른다.
            return None, 'clear'
        got.sort(key=lambda x: x[0])
        return got[len(got) // 2]


class DeckEdgeNode(Node):

    def __init__(self):
        super().__init__('deck_edge_node')
        g = self.get_parameter
        self.declare_parameter('weights', DEFAULT_WEIGHTS)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('front_ns', '/zed_front/zed_node')
        self.declare_parameter('back_ns', '/zed_back/zed_node')
        self.declare_parameter('rgb_sub', 'rgb/color/rect/image')
        self.declare_parameter('depth_sub', 'depth/depth_registered')
        self.declare_parameter('info_sub', 'depth/camera_info')
        self.declare_parameter('rate_hz', 2.0)          # 추론 주기 (양방향 합)
        # ⚠ 발행은 추론과 **분리**한다. safety_node 의 input_timeout 이 0.5s 인데
        #   추론 1틱이 평면 재적합 때 0.6s 를 넘는다. 같은 타이머로 묶으면 안전
        #   입력이 묵어 **전 방향이 정지**한다 (2026-11-05 첫 통합에서 발생).
        #   안전 신호는 판정 속도와 무관한 하트비트여야 한다.
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('median_frames', 5)
        self.declare_parameter('plane_period_s', 2.0)   # 평면 재적합 주기
        self.declare_parameter('plane_classes', DEFAULT_PLANE_CLASSES)
        self.declare_parameter('block_classes', DEFAULT_BLOCK_CLASSES)
        self.declare_parameter('center_offset_mm', co.CENTER_OFFSET_MM)
        self.declare_parameter('body_w_mm', co.BODY_W_MM)
        self.declare_parameter('margin_mm', co.MARGIN_MM)
        self.declare_parameter('front_offset_mm', co.FRONT_OFFSET_MM)
        self.declare_parameter('stop_mm', 300.0)        # 이하이면 그 방향 차단
        self.declare_parameter('slow_mm', 700.0)        # 이하이면 감속 권고
        self.declare_parameter('use_rebar_frac', True)  # 2차년도 GO/SLOW/STOP 병행

        w = str(g('weights').value)
        if not os.path.isabs(w):
            w = os.path.join(os.path.expanduser('~/ros2_ws'), w)
        self.seg = RebarSegmenter(w, device=str(g('device').value))
        de.set_class_map(self.seg.class_map)      # ⚠ 둘 다 불러야 한다
        rg.set_class_map(self.seg.class_map)
        self.idx = {v: k for k, v in self.seg.class_map.items()}
        self.plane_cls = [self.idx[x] for x in
                          str(g('plane_classes').value).split(',') if x in self.idx]
        self.block_cls = [self.idx[x] for x in
                          str(g('block_classes').value).split(',') if x in self.idx]
        self.drive_cls = [self.idx['rebar_h'], self.idx['rebar_v']]
        self.center = float(g('center_offset_mm').value)
        self.body_w = float(g('body_w_mm').value)
        self.margin = float(g('margin_mm').value)
        self.front_off = float(g('front_offset_mm').value)
        self.stop_mm = float(g('stop_mm').value)
        self.slow_mm = float(g('slow_mm').value)
        self.use_frac = bool(g('use_rebar_frac').value)
        self.plane_period = float(g('plane_period_s').value)

        self.cg_sub = MutuallyExclusiveCallbackGroup()
        n_med = int(g('median_frames').value)
        self.sides = {'forward': Side('forward', n_med),
                      'backward': Side('backward', n_med)}
        for key, ns in (('forward', str(g('front_ns').value)),
                        ('backward', str(g('back_ns').value))):
            self._wire(self.sides[key], ns, g)

        self.block_pub = self.create_publisher(String, '/deck_edge_block', 10)
        self.status_pub = self.create_publisher(String, '/deck_edge/status', 10)
        self.turn = 0
        # ⚠ 추론과 발행을 **다른 콜백 그룹**에 둔다. 한 그룹(=단일 스레드)이면
        #   0.6초짜리 추론 틱이 발행 타이머를 막아 간격이 1초까지 벌어지고,
        #   safety_node 의 0.5초 한도를 넘겨 전 방향이 정지한다 (실측 2.4Hz).
        self.cg_infer = MutuallyExclusiveCallbackGroup()
        self.cg_pub = MutuallyExclusiveCallbackGroup()
        hz = max(0.2, float(g('rate_hz').value))
        self.create_timer(1.0 / hz, self.tick, callback_group=self.cg_infer)
        phz = max(2.0, float(g('publish_hz').value))
        self.create_timer(1.0 / phz, self._publish, callback_group=self.cg_pub)
        self.get_logger().info(
            f'데크끝 판정 시작 — 전후 교대 {hz:.1f}Hz · 중앙값 {n_med}장 · '
            f'중심선 {self.center:+.0f}mm · 폭 {self.body_w:.0f}+여유 {self.margin:.0f} · '
            f'정지 {self.stop_mm:.0f}mm / 감속 {self.slow_mm:.0f}mm · '
            f'발행 {phz:.0f}Hz')

    def _wire(self, side, ns, g):
        s = str(g('rgb_sub').value)
        self.create_subscription(Image, f'{ns}/{s}',
                                 lambda m, k=side: self._on_rgb(k, m), qos_profile_sensor_data,
                                 callback_group=self.cg_sub)
        self.create_subscription(Image, f'{ns}/{str(g("depth_sub").value)}',
                                 lambda m, k=side: self._on_depth(k, m), qos_profile_sensor_data,
                                 callback_group=self.cg_sub)
        self.create_subscription(CameraInfo, f'{ns}/{str(g("info_sub").value)}',
                                 lambda m, k=side: self._on_info(k, m), qos_profile_sensor_data,
                                 callback_group=self.cg_sub)

    # ---- 입력 --------------------------------------------------------------
    def _on_rgb(self, side, msg):
        try:
            side.rgb = to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)

    def _on_depth(self, side, msg):
        a = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        side.depth = np.nan_to_num(a * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)

    def _on_info(self, side, msg):
        side.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    # ---- 판정 --------------------------------------------------------------
    def tick(self):
        # ⚠ 교대로 본다. 진행 방향만 보면 반대쪽이 묵어 탈출까지 막힌다.
        side = self.sides['forward' if self.turn % 2 == 0 else 'backward']
        self.turn += 1
        if not side.ready():
            return
        img, depth, K = side.rgb, side.depth, side.K
        mask = self.seg.predict(img)

        now = time.time()
        if side.plane is None or now - side.plane_t > self.plane_period:
            # ⚠ 정차마다 다시 맞춘다 — 배근 위에서 기울어 높이가 ±25mm 흔들린다
            p = co.fit_plane(np.isin(mask, self.plane_cls), depth, K)
            if p is not None:
                side.plane, side.plane_t = p, now
        if side.plane is None:
            side.last = {'error': '평면 적합 실패 (바닥 화소 부족)'}
            return          # 발행은 별도 타이머가 한다 — 여기서 멈추면 안 된다

        r = co.judge(np.isin(mask, self.block_cls), np.isin(mask, self.drive_cls),
                     depth, side.plane, K, center_mm=self.center,
                     body_w_mm=self.body_w, margin_mm=self.margin,
                     front_offset_mm=self.front_off)
        go, lim = side.push(r['go_mm'], r['limit'])

        frac = None
        if self.use_frac:
            j = de.judge(mask, side.fsm)
            frac = {'verdict': j['verdict'], 'rebar_frac': round(j['rebar_frac'], 3),
                    'on_rebar': bool(j['on_rebar']), 'reason': j['reason']}
        head, n_bar = rg.heading(mask)

        side.stamp = now
        side.last = {
            'go_mm': None if go is None else round(go),
            'limit': lim,
            'obs_mm': None if r['obs_mm'] is None else round(r['obs_mm']),
            'obs_lat_mm': None if r['obs_lat_mm'] is None else round(r['obs_lat_mm']),
            'deck_mm': None if r['deck_mm'] is None else round(r['deck_mm']),
            'heading_deg': None if head is None else round(float(head), 2),
            'heading_bars': int(n_bar),
            'cam_h_mm': round(side.plane.camera_height_mm),
            'plane_res_mm': round(side.plane.residual_mm, 1),
            'blind_mm': (lambda x: None if not np.isfinite(x) or x <= 0 else round(x))(
                side.plane.blind_mm(K[3], K[1], img.shape[0])),
            'frac': frac,
        }

    def _decide(self, side):
        """그 방향을 막아야 하나 / 늦춰야 하나. (block, slow, 이유)"""
        d = side.last
        if not d or 'go_mm' not in d:
            return False, False, ''
        # **두 신호가 맡는 일이 다르다** (2026-11-05 900mm 주행으로 확인):
        #   장애물  → corridor 의 go_mm. 줄자 대비 1.6~2.4%, 산포 6~13mm
        #   데크끝  → rebar_frac 의 GO/SLOW/STOP. 단조감소하고 현장 튜닝됨
        # corridor 의 deck_mm 은 요동쳐서 못 쓴다 — 섞으면 멀쩡한 판정까지 망친다.
        go = d['go_mm']
        if go is not None and go <= self.stop_mm:
            return True, False, f"장애물 {go:.0f}mm"
        f = d.get('frac')
        if f and f['verdict'] == 'STOP':
            return True, False, f"배근 {f['reason'] or f['rebar_frac']}"
        slow = (go is not None and go <= self.slow_mm) or bool(f and f['verdict'] == 'SLOW')
        why = []
        if go is not None and go <= self.slow_mm:
            why.append(f'장애물 {go:.0f}mm')
        if f and f['verdict'] == 'SLOW':
            why.append(f"배근 {f['rebar_frac']:.3f}")
        return False, slow, ' · '.join(why)

    def _publish(self):
        block, slow, reasons = [], [], []
        detail = {}
        for key, side in self.sides.items():
            b, s, why = self._decide(side)
            if b:
                block.append(key)
                reasons.append(f'{key}: {why}')
            if s:
                slow.append(key)
            detail[key] = dict(side.last, age_s=round(time.time() - side.stamp, 1))
        self.block_pub.publish(String(data=json.dumps(
            {'block': block, 'slow': slow, 'reason': ' · '.join(reasons)},
            ensure_ascii=False)))
        self.status_pub.publish(String(data=json.dumps(
            {'t': round(time.time(), 1), 'block': block, 'slow': slow,
             'detail': detail}, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DeckEdgeNode()
        ex = MultiThreadedExecutor(num_threads=3)
        ex.add_node(node)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
