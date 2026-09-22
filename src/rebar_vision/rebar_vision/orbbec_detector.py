#!/usr/bin/env python3
"""Orbbec 교차점 검출 + CAD 변환 로컬라이즈 (orchestrator용 헬퍼).

YOLO(orbbec_crossing.pt)로 컬러에서 교차점 검출 → **CAD 강체변환 + 자세별 오프셋**으로
로봇(스테이지) XY 산출 → 도달가능 자세로 분류(겹침은 current_pose 우선)해 반환.

⚠️ 2026-08-05 호모그래피 → CAD변환 대체. 이제 **depth 필요**(정합 depth로 P_cam 역투영).
   변환 코어는 orbbec_cad_transform.py (tools/calibration/test_cad_transform_plot.py 와 동일 상수).
   전제: gemini2L depth_registration:=true (depth가 컬러에 정합돼 동일 픽셀).

검출 흐름(비주얼서보잉 대체): WP 도달 시 detect()로 전체 교차점을 한 번에 얻고,
자세별로 결속. Z(하강)는 결속 시퀀스가 별도 처리 → 여기선 XY만 산출.
"""
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

from rebar_vision.orbbec_cad_transform import (
    backproject, to_stage_xy, rebar_depth_mm, bar_depths_at, split_layers, crossing_bar_layers,
    load_calibration, CALIB_PATH)


class OrbbecLocalizer:
    def __init__(self, node, model_path, color_topic, depth_topic, info_topic,
                 yrange, x_min, x_max, left_x_min=None, n_depth=5,
                 calib_path=CALIB_PATH, merge_policy='prefer_tie',
                 layer_mode='off', layer_gap_mm=100.0,
                 layer_on_unknown='accept'):
        """node: orchestrator(구독/로깅용). yrange={'r':(ymin,ymax),'l':(...)}.
        left_x_min: 좌측자세 전용 x 하한(mm). None이면 공통 x_min 사용.
        calib_path: CAD변환+오프셋 yaml(호모그래피 대체). 없으면 모듈 상수 폴백.
        depth/intrinsic으로 CAD 변환 → 자세별 스테이지 XY."""
        self.node = node
        self.merge_policy = merge_policy    # 'prefer_tie' | 'majority'
        # ★ [2026-09-08] 2층 배근에서 **한 층만** 결속하기.
        #   하단근 세로 × 상단근 가로는 탑뷰에서 교차점처럼 보이지만 실제가 아니다.
        #   'lower'/'upper' 로 목표 층을 고르면 반대 층 점은 뺀다.
        #   ⚠ 층 간격이 layer_min_gap_mm 미만이면 **단층으로 보고 아무것도 안 뺀다**
        #     (테스트베드 목업은 30mm라 자동으로 무동작 → 안전).
        #   ⚠ 앞서 시도한 '교차점 두 철근 depth 비교'는 폐기했다 — 카메라가 28°
        #     기울고 철근이 화면에서 대각선이라 팔이 철근을 벗어난다. 사용자 라벨
        #     대조에서 진짜가 259mm, 가짜가 0mm를 내 상관이 없었다(2026-09-08).
        self.layer_mode = layer_mode          # 'off' | 'on'
        self.layer_gap_mm = float(layer_gap_mm)
        self.layer_on_unknown = layer_on_unknown   # 'accept' | 'reject'
        self.yrange = yrange
        self.x_min = x_min
        self.x_max = x_max
        self.x_min_pose = {'r': x_min, 'l': x_min if left_x_min is None else left_x_min}
        self.buf = []
        self.depthbuf = []
        self.n_depth = n_depth
        self.K = None                      # (fx, fy, cx, cy)
        from cv_bridge import CvBridge
        from ultralytics import YOLO
        self.br = CvBridge()
        self.model = YOLO(model_path)
        # ⚠️ [2026-08-18] 모델이 결속 상태를 구분하는지 여기서 판별한다.
        #   구모델 orbbec_crossing.pt = {0:'crossing'} 단일 클래스 → 상태 구분 불가
        #   신모델 tie_untie_weights.pt = {0:'crossing', 1:'tie', 2:'untie'}
        #   단일 클래스 모델에서는 필터를 걸면 결속 대상이 0개가 되므로,
        #   **상태 구분이 가능한 모델일 때만** 필터가 동작하게 한다(하위호환).
        self.class_names = dict(getattr(self.model, 'names', {}) or {})
        self.state_aware = 'untie' in self.class_names.values()
        calib = load_calibration(calib_path)   # CAD 변환 R,t + 자세별 오프셋
        self.R = calib['R']
        self.t = calib['t']
        self.pose_offset = calib['pose_offset']
        node.create_subscription(Image, color_topic, self._cb,
                                 qos_profile_sensor_data)
        node.create_subscription(Image, depth_topic, self._depth_cb,
                                 qos_profile_sensor_data)
        node.create_subscription(CameraInfo, info_topic, self._info_cb,
                                 qos_profile_sensor_data)
        node.get_logger().info(
            f'  [orbbec] YOLO+CAD변환 로드: 오프셋 {self.pose_offset} '
            f'(calib={calib["source"]}, color={color_topic}, depth={depth_topic})')
        node.get_logger().info(
            f'  [orbbec] 모델 클래스 {self.class_names} → '
            f'{"결속상태 구분 가능" if self.state_aware else "단일클래스(상태 구분 불가)"}')

    def _cb(self, m):
        self.buf.append(self.br.imgmsg_to_cv2(m, 'bgr8'))
        if len(self.buf) > 5:
            self.buf.pop(0)

    def _depth_cb(self, m):
        self.depthbuf.append(
            self.br.imgmsg_to_cv2(m, 'passthrough').astype(np.float32))
        if len(self.depthbuf) > self.n_depth:
            self.depthbuf.pop(0)

    def _info_cb(self, m):
        if self.K is None and m.k[0] > 0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])   # fx, fy, cx, cy

    def ready(self):
        return bool(self.buf) and bool(self.depthbuf) and self.K is not None

    def _pcam(self, u, v):
        """(u,v) → (P_cam, z). 링/평면 계산은 **카메라 프레임**에서 해야 한다
        — `_peef`는 결속건 프레임이라 좌표계가 다르다(2026-09-08 버그)."""
        if self.K is None or not self.depthbuf:
            return None, None
        z = rebar_depth_mm(self.depthbuf, u, v)
        if z is None:
            return None, None
        fx, fy, cx, cy = self.K
        return backproject(u, v, z, fx, fy, cx, cy), z

    def _peef(self, u, v):
        """(u,v) → P_eef(결속건 프레임). depth/intrinsic 없으면 None."""
        if self.K is None or not self.depthbuf:
            return None
        z = rebar_depth_mm(self.depthbuf, u, v)
        if z is None:
            return None
        fx, fy, cx, cy = self.K
        pcam = backproject(u, v, z, fx, fy, cx, cy)
        return self.R @ pcam + self.t

    def _to_robot(self, pose, u, v, peef=None):
        """자세별 스테이지 XY. depth 소실 시 None."""
        if peef is None:
            peef = self._peef(u, v)
        if peef is None:
            return None
        return to_stage_xy(peef, pose, self.pose_offset)

    def _reachable(self, pose, u, v, peef=None):
        """해당 자세로 도달 가능? → (bool, (x,y))."""
        xy = self._to_robot(pose, u, v, peef)
        if xy is None:
            return False, None
        x, y = xy
        ymin, ymax = self.yrange[pose]
        xmin = self.x_min_pose.get(pose, self.x_min)
        ok = (ymin <= y <= ymax) and (xmin <= x <= self.x_max)
        return ok, (x, y)

    def detect_crossings(self, conf=0.3, n_frames=3):
        """최근 n_frames YOLO 검출 → 교차점 리스트 (근접 병합).

        반환: [(u, v, cls_name, conf), ...]
        ⚠️ [2026-08-18] 클래스를 보존한다. 신모델이 crossing/tie/untie를 구분하므로
           **이미 결속된 점(tie)을 다시 결속하지 않도록** 소비자가 걸러 쓸 수 있어야 한다.

           병합 클러스터의 클래스 결정 (`merge_policy`):
             'prefer_tie' (기본) — 클러스터에 `tie`가 **하나라도** 있으면 tie로 본다.
             'majority'          — 다수결, 동수면 신뢰도 합이 큰 쪽.

           ⚠ 기본이 prefer_tie인 이유 (2026-08-18 실측):
             같은 장면 12회 반복 검출에서 한 점이 **untie 12회(0.49) / tie 9회(0.37)**로
             흔들렸다. 모델이 같은 자리에 두 클래스 박스를 동시에 뱉는다.
             다수결이면 untie가 이겨 **이미 결속된 점을 다시 치게 된다**(실물 확인).
             신뢰도로도 못 잡는다 — untie 쪽이 더 높다.
             나머지 13점은 전부 단일 클래스라 이 규칙으로 잃는 것이 없었다.
             누락은 다시 결속하면 되지만 이중결속은 되돌릴 수 없다 → 보수적으로 간다.
        """
        import time
        from collections import defaultdict
        allc = []
        for _ in range(n_frames):
            if not self.buf:
                time.sleep(0.05); continue
            r = self.model(self.buf[-1], conf=conf, verbose=False)[0]
            for b in r.boxes:
                ci = int(b.cls[0]) if b.cls is not None and len(b.cls) else 0
                allc.append(((float(b.xyxy[0][0]+b.xyxy[0][2])/2),
                             (float(b.xyxy[0][1]+b.xyxy[0][3])/2),
                             self.class_names.get(ci, str(ci)),
                             float(b.conf[0]) if b.conf is not None and len(b.conf) else 0.0))
            time.sleep(0.05)
        if not allc:
            return []
        # 근접(<25px) 병합
        arr = np.array([[c[0], c[1]] for c in allc])
        used = np.zeros(len(arr), bool)
        merged = []
        for i in range(len(arr)):
            if used[i]:
                continue
            near = np.linalg.norm(arr - arr[i], axis=1) < 25
            near &= ~used
            idx = np.where(near)[0]
            uv = arr[near].mean(0)
            votes = defaultdict(lambda: [0, 0.0])      # cls -> [표수, 신뢰도합]
            for j in idx:
                v = votes[allc[j][2]]
                v[0] += 1
                v[1] += allc[j][3]
            if self.merge_policy == 'prefer_tie' and 'tie' in votes:
                cls = 'tie'                            # 결속 흔적이 한 번이라도 보이면 제외
            else:
                cls = max(votes.items(),
                          key=lambda kv: (kv[1][0], kv[1][1]))[0]
            _n, csum = votes[cls]
            merged.append((int(uv[0]), int(uv[1]), cls, csum / max(1, _n)))
            used |= near
        return merged

    def localize(self, current_pose, conf=0.3, n_frames=3):
        """검출 → 자세별 로봇XY 분류. 겹침은 current_pose 우선(자세변경 최소화).

        반환 {'r': [(x, y, u, v, cls, conf)...], 'l': [...]} (X 오름차순 정렬)
        ⚠️ [2026-08-18] 튜플에 `cls`(crossing/tie/untie)와 `conf`가 추가됐다.
           소비자는 **이동거리 계산에는 전부**, **결속 대상은 untie만** 쓰는 식으로
           나눠 써야 한다 — untie만 남기면 격자 피치 계산에서 행이 빠진다.
        """
        other = 'l' if current_pose == 'r' else 'r'
        pts = self.detect_crossings(conf, n_frames)
        sets = {'r': [], 'l': []}
        n_skip = 0
        n_nodepth = 0
        n_layer = 0
        keep = []
        all3d = []          # 검출 전부의 3D (평면 적합용)
        for (u, v, cls, cf) in pts:
            peef = self._peef(u, v)                     # depth 역투영 1회
            if peef is None:
                n_nodepth += 1; continue                # depth 소실 → 무시
            pcam, zc = self._pcam(u, v)                 # ★ 카메라 프레임 (평면·링용)
            if pcam is None:
                n_nodepth += 1; continue
            i3 = len(all3d)
            all3d.append(pcam)                          # 평면 적합엔 전부 쓴다
            ok_c, xy_c = self._reachable(current_pose, u, v, peef)
            ok_o, xy_o = (False, None)
            if not ok_c:
                ok_o, xy_o = self._reachable(other, u, v, peef)
            if not (ok_c or ok_o):
                n_skip += 1                             # 범위밖 → 무시
                continue
            keep.append((current_pose if ok_c else other,
                         (xy_c if ok_c else xy_o), u, v, cls, cf, zc, i3))
        # ★ 두 단계로 거른다 (2026-09-08).
        #   ① 평면: 검출 **전부**로 맞춘다(점이 많아야 안정. 도달범위 안은 2~4개뿐).
        #   ② 판정: 도달범위 안 점만, **교차점 둘레 링**으로 두 철근 높이를 따로 재
        #      섞였으면 뺀다. 중심 depth 하나로는 `상단×상단`과 `상단×하단`을
        #      구분할 수 없다(위에 얹힌 철근만 보이므로) — 사용자 지적.
        drop = set()
        if self.layer_mode != 'off' and len(all3d) >= 4:
            A = np.asarray(all3d, float)
            c0 = A.mean(0)
            _, _, vt = np.linalg.svd(A - c0, full_matrices=False)
            nv = vt[2] / np.linalg.norm(vt[2])
            if nv[2] > 0:
                nv = -nv
            n_unk = 0
            for i, k in enumerate(keep):
                ok, spread, nsec = crossing_bar_layers(
                    self.depthbuf, k[2], k[3], float(k[6]), nv, c0,
                    self.K[0], self.K[1], self.K[2], self.K[3],
                    layer_gap_mm=self.layer_gap_mm)
                if ok is None:
                    n_unk += 1
                    if self.layer_on_unknown == 'reject':
                        drop.add(i)
                elif not ok:
                    drop.add(i)
                    self.node.get_logger().info(
                        f'  [orbbec] 층혼합 제외 ({k[2]},{k[3]}) '
                        f'철근 높이차 {spread:.0f}mm > {self.layer_gap_mm:.0f} '
                        f'(갈래 {nsec})')
            n_layer = len(drop)
            if n_unk:
                self.node.get_logger().info(f'  [orbbec] 층 판정불가 {n_unk}개')
        for i, (pose, xy, u, v, cls, cf, _z, _i3) in enumerate(keep):
            if i in drop:
                continue
            sets[pose].append((xy[0], xy[1], u, v, cls, cf))
        for p in sets:
            sets[p].sort(key=lambda c: c[0])            # X 오름차순
        cls_n = {}
        for c in pts:
            cls_n[c[2]] = cls_n.get(c[2], 0) + 1
        detail = ' '.join(f'{k}{v}' for k, v in sorted(cls_n.items())) or '-'
        self.node.get_logger().info(
            f'  [orbbec] 검출 {len(pts)}개 [{detail}] → 우{len(sets["r"])} 좌{len(sets["l"])} '
            f'범위밖{n_skip} depth소실{n_nodepth}'
            + (f' 층제외{n_layer}' if self.layer_mode != 'off' else '')
            + f' (현재자세={current_pose} 우선)')
        return sets
