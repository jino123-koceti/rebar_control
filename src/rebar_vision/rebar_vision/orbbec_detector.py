#!/usr/bin/env python3
"""Orbbec 교차점 검출 + 호모그래피 로컬라이즈 (orchestrator용 헬퍼).

YOLO(orbbec_crossing.pt)로 컬러에서 교차점 검출 → 자세별 호모그래피로 로봇 XY 산출
→ 도달가능 자세로 분류(겹침은 current_pose 우선)해 자세별 리스트 반환.

검출 흐름(비주얼서보잉 대체): WP 도달 시 detect()로 전체 교차점을 한 번에 얻고,
자세별로 결속. depth 불필요 (평면 호모그래피, X,Y만).
"""
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class OrbbecLocalizer:
    def __init__(self, node, model_path, homo_path, color_topic,
                 yrange, x_min, x_max, left_x_min=None):
        """node: orchestrator(구독/로깅용). yrange={'r':(ymin,ymax),'l':(...)}.
        left_x_min: 좌측자세 전용 x 하한(mm). 저-X 결속점이 검출 XY 오차로 배근에
        부딪히는 걸 막기 위해 좌측만 x_min을 높여 필터링. None이면 공통 x_min 사용."""
        self.node = node
        self.yrange = yrange
        self.x_min = x_min
        self.x_max = x_max
        # 자세별 x 하한 (좌측은 left_x_min으로 저-X 결속점 스킵)
        self.x_min_pose = {'r': x_min, 'l': x_min if left_x_min is None else left_x_min}
        self.buf = []
        from cv_bridge import CvBridge
        from ultralytics import YOLO
        import yaml
        self.br = CvBridge()
        self.model = YOLO(model_path)
        hom = yaml.safe_load(open(homo_path))
        self.H = {}
        for pose, key in (('r', 'right'), ('l', 'left')):
            if key in hom and 'H' in hom[key]:
                self.H[pose] = np.array(hom[key]['H'], float)
        node.create_subscription(Image, color_topic, self._cb,
                                 qos_profile_sensor_data)
        node.get_logger().info(
            f'  [orbbec] YOLO+호모그래피 로드: 자세 {list(self.H.keys())} '
            f'(topic={color_topic})')

    def _cb(self, m):
        self.buf.append(self.br.imgmsg_to_cv2(m, 'bgr8'))
        if len(self.buf) > 5:
            self.buf.pop(0)

    def ready(self):
        return bool(self.buf) and bool(self.H)

    def _to_robot(self, pose, u, v):
        p = self.H[pose] @ np.array([u, v, 1.0])
        return p[0]/p[2], p[1]/p[2]

    def _reachable(self, pose, u, v):
        """해당 자세로 도달 가능? → (bool, (x,y))."""
        if pose not in self.H:
            return False, None
        x, y = self._to_robot(pose, u, v)
        ymin, ymax = self.yrange[pose]
        xmin = self.x_min_pose.get(pose, self.x_min)
        ok = (ymin <= y <= ymax) and (xmin <= x <= self.x_max)
        return ok, (x, y)

    def detect_crossings(self, conf=0.3, n_frames=3):
        """최근 n_frames YOLO 검출 → 교차점 픽셀 리스트 (근접 병합)."""
        import time
        allc = []
        for _ in range(n_frames):
            if not self.buf:
                time.sleep(0.05); continue
            r = self.model(self.buf[-1], conf=conf, verbose=False)[0]
            for b in r.boxes:
                allc.append(((float(b.xyxy[0][0]+b.xyxy[0][2])/2),
                             (float(b.xyxy[0][1]+b.xyxy[0][3])/2)))
            time.sleep(0.05)
        if not allc:
            return []
        # 근접(<25px) 병합
        arr = np.array(allc)
        used = np.zeros(len(arr), bool)
        merged = []
        for i in range(len(arr)):
            if used[i]:
                continue
            near = np.linalg.norm(arr - arr[i], axis=1) < 25
            near &= ~used
            merged.append(arr[near].mean(0))
            used |= near
        return [(int(u), int(v)) for u, v in merged]

    def localize(self, current_pose, conf=0.3, n_frames=3):
        """검출 → 자세별 로봇XY 분류. 겹침은 current_pose 우선(자세변경 최소화).
        반환 {'r': [(x,y,u,v)...], 'l': [...]} (X 오름차순 정렬)."""
        other = 'l' if current_pose == 'r' else 'r'
        pts = self.detect_crossings(conf, n_frames)
        sets = {'r': [], 'l': []}
        n_skip = 0
        for (u, v) in pts:
            ok_c, xy_c = self._reachable(current_pose, u, v)
            if ok_c:                                   # 겹침 포함 현재자세 우선
                sets[current_pose].append((xy_c[0], xy_c[1], u, v))
                continue
            ok_o, xy_o = self._reachable(other, u, v)
            if ok_o:
                sets[other].append((xy_o[0], xy_o[1], u, v))
            else:
                n_skip += 1                            # 범위밖 → 무시
        for p in sets:
            sets[p].sort(key=lambda c: c[0])           # X 오름차순
        self.node.get_logger().info(
            f'  [orbbec] 검출 {len(pts)}개 → 우{len(sets["r"])} 좌{len(sets["l"])} '
            f'범위밖{n_skip} (현재자세={current_pose} 우선)')
        return sets
