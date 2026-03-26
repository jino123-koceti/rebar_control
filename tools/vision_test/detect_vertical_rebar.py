#!/usr/bin/env python3
"""
세로 철근 검출 테스트 (OpenCV 기반)

원근 왜곡된 세로 배근 철근을 검출합니다.
카메라가 하방 경사로 배근면을 보기 때문에 세로 철근은
이미지 상단(먼 곳)에서 소실점을 향해 수렴하는 사선으로 보입니다.

접근:
1. ROI로 배경 제외 (상단 배경, 하단 데크플레이트 가장자리)
2. 원근 효과 고려: 이미지 위치별로 세로 철근 각도가 다름
   - 이미지 좌측: 소실점 방향으로 우측 경사 (양수 각도)
   - 이미지 중앙: 거의 수직
   - 이미지 우측: 소실점 방향으로 좌측 경사 (음수 각도)
3. 소실점 기반 각도 필터링으로 실제 세로 철근만 추출

사용법:
    python3 detect_vertical_rebar.py --image path/to/image.png
    python3 detect_vertical_rebar.py  # ROS2 토픽 (zed_back)

키 조작:
    q: 종료
    s: 현재 프레임 저장
    1: 검출 결과 (기본)
    2: Canny edge
    3: 각도 필터 시각화 (소실점 방향 표시)
    +/-: Hough threshold 조정
    v/V: 소실점 Y위치 조정 (up/down)
"""

import cv2
import numpy as np
import argparse
import json
import os
from datetime import datetime


class VerticalRebarDetector:
    """원근 왜곡을 고려한 세로 철근 검출기"""

    def __init__(self, canny_low=80, canny_high=180, hough_threshold=50,
                 min_line_length=100, max_line_gap=50,
                 vanish_y_ratio=0.0, angle_margin=15.0,
                 roi_top_ratio=0.15, roi_bottom_ratio=0.95):
        # Canny
        self.canny_low = canny_low
        self.canny_high = canny_high

        # HoughLinesP
        self.hough_threshold = hough_threshold
        self.min_line_length = min_line_length
        self.max_line_gap = max_line_gap

        # 소실점: 이미지 상단 밖 (vanish_y_ratio < 0이면 이미지 위쪽 밖)
        # 소실점 x = 이미지 중앙으로 가정 (카메라가 정면을 볼 때)
        self.vanish_y_ratio = vanish_y_ratio  # 소실점 y = h * ratio (0.0 = 상단, 음수 = 상단 밖)
        self.vanish_x_ratio = 0.5             # 소실점 x = 이미지 중앙

        # 소실점 방향 대비 각도 허용 범위 (도)
        self.angle_margin = angle_margin

        # ROI 영역 (이미지 높이 대비 비율)
        self.roi_top_ratio = roi_top_ratio
        self.roi_bottom_ratio = roi_bottom_ratio

        # 전처리
        self.blur_ksize = 7
        self.clahe_clip = 2.5
        self.clahe_grid = (8, 8)

        # 병합 거리 (세로 라인 x좌표 기준)
        self.merge_dist = 50

        # 두께 필터: 철근은 이미지에서 일정 폭 이상의 어두운 밴드
        self.min_width = 10     # 최소 두께 (px) - 이보다 얇으면 노이즈
        self.width_samples = 7  # 라인을 따라 두께 측정할 샘플 수
        self.width_scan = 30    # 수직 방향 스캔 범위 (±px)

    def _color_mask(self, roi_bgr):
        """철근 색상 마스크 생성 (HSV 기반)

        철근: 갈색/어두운 회색 → 낮은 채도, 낮은~중간 밝기
        데크플레이트: 밝은 은색 → 높은 밝기
        배경/하늘: 높은 밝기 또는 높은 채도(파란색)

        어두운 영역(철근 후보)만 남기고 밝은 배경을 제거
        """
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)

        # 조건1: 밝기가 너무 높지 않은 영역 (데크플레이트/하늘 제외)
        _, bright_mask = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY_INV)

        # 조건2: 너무 어둡지도 않은 영역 (완전 검정 제외)
        _, dark_mask = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)

        # 조건3: 채도가 과도하게 높지 않은 영역 (색이 강한 배경 제외)
        sat = hsv[:, :, 1]
        _, sat_mask = cv2.threshold(sat, 120, 255, cv2.THRESH_BINARY_INV)

        # 최종 마스크: 모든 조건 AND
        mask = cv2.bitwise_and(bright_mask, dark_mask)
        mask = cv2.bitwise_and(mask, sat_mask)

        # 모폴로지로 작은 노이즈 제거 + 철근 영역 연결
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        return mask

    def preprocess(self, frame):
        """전처리: ROI → 색상필터 → 그레이 → CLAHE → 블러 → Canny"""
        h, w = frame.shape[:2]
        roi_top = int(h * self.roi_top_ratio)
        roi_bottom = int(h * self.roi_bottom_ratio)

        roi = frame[roi_top:roi_bottom, :]

        # 색상 마스크로 철근 후보 영역만 추출
        color_mask = self._color_mask(roi)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=self.clahe_grid)
        enhanced = clahe.apply(gray)

        blurred = cv2.GaussianBlur(enhanced, (self.blur_ksize, self.blur_ksize), 0)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)

        # 색상 마스크 적용: 철근 영역 밖의 엣지 제거
        edges = cv2.bitwise_and(edges, color_mask)

        # 세로 방향 강조: 수직 커널로 모폴로지
        kernel_v = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 5))
        edges = cv2.dilate(edges, kernel_v, iterations=1)
        edges = cv2.erode(edges, kernel_v, iterations=1)

        # 전체 이미지 크기 edge (ROI 밖은 0)
        edges_full = np.zeros((h, w), dtype=np.uint8)
        edges_full[roi_top:roi_bottom, :] = edges

        return enhanced, edges_full, roi_top, roi_bottom

    def _expected_angle_at(self, x, y, w, h):
        """이미지 위치 (x, y)에서 소실점 방향의 각도를 계산

        소실점으로부터 (x, y)를 잇는 직선의 각도가
        해당 위치에서의 '세로 철근 예상 각도'
        """
        vx = w * self.vanish_x_ratio
        vy = h * self.vanish_y_ratio

        dx = x - vx
        dy = y - vy
        if abs(dy) < 1:
            return 90.0 if dx == 0 else 0.0
        # atan2 결과를 -90~+90 범위로 변환
        angle = np.degrees(np.arctan2(dx, dy))  # dx/dy: x변화/y변화
        return angle

    def _is_vertical_rebar(self, line_info, w, h):
        """소실점 기반으로 세로 철근인지 판별

        라인이 소실점 방향과 평행한지 확인.
        세로 철근은 소실점에서 방사형으로 뻗어나가므로,
        라인 각도가 소실점→라인중심 방향과 일치해야 함.
        """
        cx, cy = line_info['center']
        line_angle = line_info['angle']  # atan2(dy,dx), -180 ~ +180

        # 소실점 → 라인 중심 방향 각도
        vx = w * self.vanish_x_ratio
        vy = h * self.vanish_y_ratio
        dx = cx - vx
        dy = cy - vy
        if abs(dx) < 1 and abs(dy) < 1:
            vanish_line_angle = 90.0
        else:
            vanish_line_angle = np.degrees(np.arctan2(dy, dx))

        # 각도 차이 (순환 보정: atan2는 -180~+180이므로 차이는 최대 360)
        diff = abs(line_angle - vanish_line_angle)
        if diff > 180:
            diff = 360 - diff
        # 방향 무관 (라인은 양방향): 0°차이 또는 180°차이 모두 일치
        if diff > 90:
            diff = 180 - diff

        return diff <= self.angle_margin

    def _measure_width(self, gray, line_info):
        """라인의 수직 방향 두께(어두운 밴드 폭)를 측정

        라인을 따라 여러 지점에서 수직 방향 밝기 프로파일을 샘플링하고,
        어두운 밴드(=철근 단면)의 평균 폭을 반환.
        """
        h, w = gray.shape[:2]
        x1, y1, x2, y2 = line_info['pts']
        angle_rad = np.radians(line_info['angle'])

        # 라인에 수직인 방향 (단위벡터)
        nx = -np.sin(angle_rad)
        ny = np.cos(angle_rad)

        widths = []
        for i in range(self.width_samples):
            # 라인 위의 샘플 포인트 (균등 분할)
            t = (i + 1) / (self.width_samples + 1)
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)

            # 수직 방향으로 밝기 프로파일 샘플링
            profile = []
            for d in range(-self.width_scan, self.width_scan + 1):
                sx = int(round(px + d * nx))
                sy = int(round(py + d * ny))
                if 0 <= sx < w and 0 <= sy < h:
                    profile.append(gray[sy, sx])
                else:
                    profile.append(255)  # 범위 밖은 밝게 (철근 아님)

            profile = np.array(profile, dtype=np.float32)

            # 어두운 밴드 폭 측정: 프로파일 중앙값 기준으로 임계값 적용
            center_val = profile[len(profile) // 2]
            # 주변 밝기 대비 어두운 구간 찾기
            bg_level = np.percentile(profile, 75)  # 배경 밝기 (밝은 쪽 기준)
            threshold = (center_val + bg_level) / 2  # 중간값

            dark_mask = profile < threshold
            if not np.any(dark_mask):
                widths.append(0)
                continue

            # 연속된 어두운 구간 중 중심을 포함하는 것의 폭
            mid = len(profile) // 2
            # 중심에서 좌우로 확장하며 어두운 영역 찾기
            left = mid
            while left > 0 and dark_mask[left - 1]:
                left -= 1
            right = mid
            while right < len(dark_mask) - 1 and dark_mask[right + 1]:
                right += 1

            widths.append(right - left + 1)

        if not widths:
            return 0.0
        # 중앙값 사용 (이상치 제거)
        return float(np.median(widths))

    def detect(self, frame):
        """세로 철근 검출 파이프라인"""
        h, w = frame.shape[:2]
        enhanced, edges, roi_top, roi_bottom = self.preprocess(frame)

        # HoughLinesP
        lines = cv2.HoughLinesP(
            edges, rho=1, theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )

        vertical = []
        rejected = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx = x2 - x1
                dy = y2 - y1
                length = np.sqrt(dx**2 + dy**2)

                if dx == 0:
                    angle = 90.0
                else:
                    angle = np.degrees(np.arctan2(dy, dx))

                line_info = {
                    'pts': (x1, y1, x2, y2),
                    'angle': angle,
                    'length': length,
                    'center': ((x1 + x2) // 2, (y1 + y2) // 2),
                }

                # 1차 필터: 최소 각도 (너무 수평인 건 제외)
                if abs(angle) < 30:
                    rejected.append(line_info)
                    continue

                # 2차 필터: 소실점 방향 일치
                if self._is_vertical_rebar(line_info, w, h):
                    vertical.append(line_info)
                else:
                    rejected.append(line_info)

        # 3차 필터: 두께 측정 → 얇은 라인 제거
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        thick_vertical = []
        thin_rejected = []
        for line in vertical:
            width = self._measure_width(gray, line)
            line['width'] = width
            if width >= self.min_width:
                thick_vertical.append(line)
            else:
                thin_rejected.append(line)

        rejected.extend(thin_rejected)

        # 중복 병합 (x좌표 기준)
        thick_vertical = self._merge_lines(thick_vertical)

        return {
            'vertical': thick_vertical,
            'rejected': rejected,
            'thin_rejected': thin_rejected,  # 두께 필터로 제거된 것 별도 보관
            'edges': edges,
            'enhanced': enhanced,
            'roi': (roi_top, roi_bottom),
            'vanish_pt': (int(w * self.vanish_x_ratio), int(h * self.vanish_y_ratio)),
        }

    def _merge_lines(self, lines):
        """동일선상(collinear) 라인 병합

        같은 철근의 상/하 분절을 하나로 합침:
        1. 각도 유사 (angle_thresh 이내)
        2. 한 라인의 연장선과 다른 라인 중심의 수직 거리가 가까움 (perp_thresh 이내)
        → 두 라인의 4개 끝점 중 가장 먼 2개를 연결하여 하나의 라인으로 병합
        """
        if len(lines) <= 1:
            return lines

        angle_thresh = 10.0   # 각도 차이 허용 (도)
        perp_thresh = 40.0    # 수직 거리 허용 (px)

        used = [False] * len(lines)
        merged = []

        for i in range(len(lines)):
            if used[i]:
                continue
            current = lines[i]
            # 현재 라인과 병합 가능한 후보 찾기
            for j in range(i + 1, len(lines)):
                if used[j]:
                    continue
                candidate = lines[j]

                # 1) 각도 유사성
                adiff = abs(current['angle'] - candidate['angle'])
                if adiff > 180:
                    adiff = 360 - adiff
                if adiff > angle_thresh:
                    continue

                # 2) 수직 거리: current 라인 위에 candidate 중심을 투영
                perp = self._perpendicular_distance(current, candidate['center'])
                if perp > perp_thresh:
                    continue

                # 병합: 4개 끝점 중 가장 먼 2점으로 새 라인 생성
                current = self._join_lines(current, candidate)
                used[j] = True

            merged.append(current)

        # 최종 x좌표 기준 정렬
        merged.sort(key=lambda l: l['center'][0])
        return merged

    def _perpendicular_distance(self, line, point):
        """라인의 연장선과 point 사이의 수직 거리"""
        x1, y1, x2, y2 = line['pts']
        px, py = point
        # 직선 ax + by + c = 0
        dx = x2 - x1
        dy = y2 - y1
        length = np.sqrt(dx**2 + dy**2)
        if length < 1:
            return np.sqrt((px - x1)**2 + (py - y1)**2)
        # 수직 거리 = |cross product| / |line length|
        dist = abs(dy * px - dx * py + x2 * y1 - y2 * x1) / length
        return dist

    def _join_lines(self, line_a, line_b):
        """두 라인의 4개 끝점 중 가장 먼 2점으로 병합"""
        pts = [
            (line_a['pts'][0], line_a['pts'][1]),
            (line_a['pts'][2], line_a['pts'][3]),
            (line_b['pts'][0], line_b['pts'][1]),
            (line_b['pts'][2], line_b['pts'][3]),
        ]
        # 모든 점 쌍 중 가장 먼 거리
        max_dist = 0
        best = (pts[0], pts[1])
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                d = np.sqrt((pts[i][0] - pts[j][0])**2 + (pts[i][1] - pts[j][1])**2)
                if d > max_dist:
                    max_dist = d
                    best = (pts[i], pts[j])

        x1, y1 = best[0]
        x2, y2 = best[1]
        dx = x2 - x1
        dy = y2 - y1
        angle = 90.0 if dx == 0 else np.degrees(np.arctan2(dy, dx))

        # 두께는 큰 값 유지
        width = max(line_a.get('width', 0), line_b.get('width', 0))

        return {
            'pts': (x1, y1, x2, y2),
            'angle': angle,
            'length': max_dist,
            'center': ((x1 + x2) // 2, (y1 + y2) // 2),
            'width': width,
        }

    def draw_result(self, frame, result, show_mode='detect'):
        """검출 결과 시각화"""
        vis = frame.copy()
        h, w = vis.shape[:2]
        roi_top, roi_bottom = result['roi']
        vx, vy = result['vanish_pt']
        vertical = result['vertical']

        # ROI 영역 표시
        cv2.rectangle(vis, (0, roi_top), (w, roi_bottom), (100, 100, 100), 1)

        if show_mode == 'vanish':
            # 소실점 방향 가이드 라인 표시
            cv2.circle(vis, (vx, max(vy, 0)), 8, (0, 0, 255), 2)
            for x_sample in range(0, w, 80):
                y_start = roi_bottom
                # 소실점 → (x_sample, y_start) 방향 라인
                cv2.line(vis, (vx, max(vy, 0)), (x_sample, y_start), (50, 50, 200), 1)

        # 두께 부족으로 제거된 라인 (빨간색, 얇게) - detect 모드에서만
        if show_mode == 'detect':
            for line in result.get('thin_rejected', []):
                x1, y1, x2, y2 = line['pts']
                cv2.line(vis, (x1, y1), (x2, y2), (0, 0, 200), 1)
                cx, cy = line['center']
                w_val = line.get('width', 0)
                cv2.putText(vis, f"w={w_val:.0f}", (cx + 5, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 200), 1)

        # 세로 철근 (녹색, 굵게)
        if show_mode in ('detect', 'vanish'):
            for i, line in enumerate(vertical):
                x1, y1, x2, y2 = line['pts']
                cv2.line(vis, (x1, y1), (x2, y2), (0, 255, 100), 3)
                cx, cy = line['center']
                cv2.circle(vis, (cx, cy), 5, (0, 255, 100), -1)
                w_val = line.get('width', 0)
                cv2.putText(vis, f"{i} w={w_val:.0f}", (cx + 8, cy - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 100), 2)

        # 정보 패널
        panel_h = 140
        overlay = vis[:panel_h, :].copy()
        cv2.rectangle(vis, (0, 0), (w, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, vis[:panel_h, :], 0.7, 0, vis[:panel_h, :])

        y_text = 25
        cv2.putText(vis, f"Vertical rebar: {len(vertical)}",
                    (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 100), 2)
        y_text += 30
        # 각 라인의 x좌표 나열
        x_positions = [l['center'][0] for l in vertical]
        if x_positions:
            spacing = [x_positions[i+1] - x_positions[i] for i in range(len(x_positions)-1)]
            spacing_str = ", ".join([f"{s}px" for s in spacing])
            cv2.putText(vis, f"X: {x_positions}  Spacing: [{spacing_str}]",
                        (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        y_text += 25
        cv2.putText(vis, f"Canny: {self.canny_low}/{self.canny_high}  "
                    f"Hough: {self.hough_threshold}  "
                    f"VanishY: {self.vanish_y_ratio:.2f}  "
                    f"Margin: {self.angle_margin:.0f}deg",
                    (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        y_text += 25
        thin_count = len(result.get('thin_rejected', []))
        widths_str = ", ".join([f"{l.get('width', 0):.0f}" for l in vertical])
        cv2.putText(vis, f"ROI: {roi_top}-{roi_bottom}  "
                    f"Rejected: {len(result['rejected'])}  "
                    f"ThinFiltered: {thin_count}  "
                    f"MinW: {self.min_width}  Widths: [{widths_str}]",
                    (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        y_text += 25
        cv2.putText(vis, "Keys: q=quit s=save 1=detect 2=edge 3=vanish +/-=hough v/V=vanishY",
                    (10, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        return vis


def get_save_dir():
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', 'data', 'vision_test', 'rebar_lines'
    )


def save_frame(frame, result, detector, tag='manual'):
    """프레임 + 검출결과 + JSON 저장"""
    save_dir = get_save_dir()
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = f'{timestamp}_{tag}'

    cv2.imwrite(os.path.join(save_dir, f'{prefix}_original.png'), frame)
    vis = detector.draw_result(frame, result, 'detect')
    cv2.imwrite(os.path.join(save_dir, f'{prefix}_vertical.png'), vis)
    vis_vanish = detector.draw_result(frame, result, 'vanish')
    cv2.imwrite(os.path.join(save_dir, f'{prefix}_vanish.png'), vis_vanish)
    cv2.imwrite(os.path.join(save_dir, f'{prefix}_edges.png'), result['edges'])

    def line_to_dict(line):
        return {
            'pts': [int(v) for v in line['pts']],
            'angle': round(float(line['angle']), 2),
            'length': round(float(line['length']), 1),
            'center': [int(v) for v in line['center']],
            'width': round(float(line.get('width', 0)), 1),
        }

    vertical = result['vertical']
    x_positions = [l['center'][0] for l in vertical]
    spacing = [x_positions[i+1] - x_positions[i] for i in range(len(x_positions)-1)]

    log = {
        'timestamp': timestamp,
        'tag': tag,
        'image_size': [frame.shape[1], frame.shape[0]],
        'params': {
            'canny_low': detector.canny_low,
            'canny_high': detector.canny_high,
            'hough_threshold': detector.hough_threshold,
            'min_line_length': detector.min_line_length,
            'max_line_gap': detector.max_line_gap,
            'vanish_y_ratio': round(detector.vanish_y_ratio, 3),
            'vanish_x_ratio': round(detector.vanish_x_ratio, 3),
            'angle_margin': detector.angle_margin,
            'roi_top_ratio': detector.roi_top_ratio,
            'roi_bottom_ratio': detector.roi_bottom_ratio,
        },
        'result': {
            'vertical_count': len(vertical),
            'rejected_count': len(result['rejected']),
            'thin_filtered_count': len(result.get('thin_rejected', [])),
            'min_width': detector.min_width,
            'vertical': [line_to_dict(l) for l in vertical],
            'x_positions': [int(x) for x in x_positions],
            'spacing_px': [int(s) for s in spacing],
        },
    }
    log_path = os.path.join(save_dir, f'{prefix}_vertical_result.json')
    with open(log_path, 'w') as f:
        json.dump(log, f, indent=2, ensure_ascii=False)

    print(f"Saved: {save_dir}/{prefix}_*")


def handle_key(key, detector, frame, result, show_mode):
    """키 입력 처리, (새 show_mode, quit 여부) 반환"""
    if key == ord('q'):
        return show_mode, True
    elif key == ord('s'):
        save_frame(frame, result, detector, tag='manual')
    elif key == ord('1'):
        return 'detect', False
    elif key == ord('2'):
        return 'edge', False
    elif key == ord('3'):
        return 'vanish', False
    elif key == ord('+') or key == ord('='):
        detector.hough_threshold = min(200, detector.hough_threshold + 5)
        print(f"Hough threshold: {detector.hough_threshold}")
    elif key == ord('-'):
        detector.hough_threshold = max(10, detector.hough_threshold - 5)
        print(f"Hough threshold: {detector.hough_threshold}")
    elif key == ord('v'):
        detector.vanish_y_ratio -= 0.05
        print(f"Vanish Y ratio: {detector.vanish_y_ratio:.2f}")
    elif key == ord('V'):
        detector.vanish_y_ratio += 0.05
        print(f"Vanish Y ratio: {detector.vanish_y_ratio:.2f}")
    elif key == ord('w'):
        detector.min_width = max(1, detector.min_width - 1)
        print(f"Min width: {detector.min_width}px")
    elif key == ord('W'):
        detector.min_width += 1
        print(f"Min width: {detector.min_width}px")
    return show_mode, False


def run_with_image(detector, image_path):
    """이미지 파일로 테스트"""
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Cannot read image: {image_path}")
        return

    print(f"Image loaded: {frame.shape[1]}x{frame.shape[0]}")

    # 즉시 자동 저장
    result = detector.detect(frame)
    save_frame(frame, result, detector, tag='vert_test')
    print(f"Auto-saved: V={len(result['vertical'])}, rejected={len(result['rejected'])}")

    show_mode = 'detect'
    while True:
        result = detector.detect(frame)
        if show_mode == 'edge':
            vis = cv2.cvtColor(result['edges'], cv2.COLOR_GRAY2BGR)
        else:
            vis = detector.draw_result(frame, result, show_mode)

        cv2.imshow('Vertical Rebar Detection', vis)
        key = cv2.waitKey(0) & 0xFF
        show_mode, quit_req = handle_key(key, detector, frame, result, show_mode)
        if quit_req:
            break

    cv2.destroyAllWindows()

    # 최종 결과
    print(f"\n=== Vertical Rebar Detection ===")
    print(f"Detected: {len(result['vertical'])} lines")
    for i, line in enumerate(result['vertical']):
        print(f"  [{i}] x={line['center'][0]:4d}  angle={line['angle']:+6.1f}deg  len={line['length']:.0f}px")


def run_with_ros(detector, args):
    """ROS2 토픽에서 실시간 검출"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    class VertRebarNode(Node):
        def __init__(self):
            super().__init__('vertical_rebar_detect_test')
            self.bridge = CvBridge()
            self.detector = detector
            self.latest_frame = None
            self.show_mode = 'detect'
            self.auto_saved = False
            self.frame_count = 0

            topic = args.topic
            self.get_logger().info(f"Subscribing to: {topic}")
            self.sub = self.create_subscription(Image, topic, self.image_cb, 1)
            self.timer = self.create_timer(1.0 / 30, self.display_loop)
            self.get_logger().info(f"Auto-save dir: {get_save_dir()}")

        def image_cb(self, msg):
            try:
                self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f"cv_bridge: {e}")

        def display_loop(self):
            if self.latest_frame is None:
                return

            frame = self.latest_frame
            self.frame_count += 1
            result = self.detector.detect(frame)

            if not self.auto_saved and self.frame_count >= 3:
                self.auto_saved = True
                save_frame(frame, result, self.detector, tag='vert_auto')
                self.get_logger().info(
                    f"Auto-saved: V={len(result['vertical'])}, "
                    f"rejected={len(result['rejected'])}"
                )

            if self.show_mode == 'edge':
                vis = cv2.cvtColor(result['edges'], cv2.COLOR_GRAY2BGR)
            else:
                vis = self.detector.draw_result(frame, result, self.show_mode)

            cv2.imshow('Vertical Rebar Detection', vis)
            key = cv2.waitKey(1) & 0xFF
            self.show_mode, quit_req = handle_key(
                key, self.detector, frame, result, self.show_mode
            )
            if quit_req:
                rclpy.shutdown()

    rclpy.init()
    node = VertRebarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='세로 철근 검출 테스트 (원근 보정)')
    parser.add_argument('--image', '-i', type=str, default=None,
                        help='테스트 이미지 (없으면 ROS2 토픽)')
    parser.add_argument('--topic', '-t', type=str,
                        default='/zed_back/zed_node/left/image_rect_color',
                        help='ROS2 이미지 토픽 (default: zed_back)')
    parser.add_argument('--canny-low', type=int, default=80)
    parser.add_argument('--canny-high', type=int, default=180)
    parser.add_argument('--hough-thresh', type=int, default=50)
    parser.add_argument('--min-line-length', type=int, default=100)
    parser.add_argument('--max-line-gap', type=int, default=50)
    parser.add_argument('--vanish-y', type=float, default=0.0,
                        help='소실점 Y 비율 (0.0=상단, 음수=상단 밖, default: 0.0)')
    parser.add_argument('--angle-margin', type=float, default=15.0,
                        help='소실점 방향 대비 허용 각도 (default: 15도)')
    parser.add_argument('--roi-top', type=float, default=0.15,
                        help='ROI 상단 비율 (default: 0.15)')
    parser.add_argument('--roi-bottom', type=float, default=0.95,
                        help='ROI 하단 비율 (default: 0.95)')

    args = parser.parse_args()

    detector = VerticalRebarDetector(
        canny_low=args.canny_low,
        canny_high=args.canny_high,
        hough_threshold=args.hough_thresh,
        min_line_length=args.min_line_length,
        max_line_gap=args.max_line_gap,
        vanish_y_ratio=args.vanish_y,
        angle_margin=args.angle_margin,
        roi_top_ratio=args.roi_top,
        roi_bottom_ratio=args.roi_bottom,
    )

    if args.image:
        run_with_image(detector, args.image)
    else:
        run_with_ros(detector, args)


if __name__ == '__main__':
    main()
