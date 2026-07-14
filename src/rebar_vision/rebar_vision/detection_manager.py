#!/usr/bin/env python3
"""
검출 관리 모듈

멀티검출 서비스 호출, 포인트 누적, 클러스터링, 보간, 추세선 보정, 범위 필터링.
tying_orchestrator_node에서 분리.
"""

import time
import numpy as np
from rebar_base_interfaces.srv import DetectCrossings


class DetectionManager:
    """비전 검출 관리.

    사용법:
        dm = DetectionManager(node, detect_client, params)
        dm.start(camera_side='right')  # or 'left' or 'both'
        # 상태머신 루프에서:
        result = dm.update()
        if result is not None:  # 검출 완료
            points = result  # list of (label, x_mm, y_mm, side)
        elif result == 'error':
            ...
    """

    def __init__(self, node, detect_client, params):
        """
        Args:
            node: ROS2 노드 (get_logger, flog)
            detect_client: DetectCrossings 서비스 클라이언트
            params: dict with keys:
                multi_detect_tries, multi_detect_delay, detection_confidence,
                expected_points_per_cam, cluster_distance_mm, rebar_spacing_mm,
                max_stage_x_mm, max_stage_y_mm,
                right_cam_y_min, right_cam_y_max, left_cam_y_min, left_cam_y_max
        """
        self._node = node
        self._client = detect_client
        self._p = params

        # 상태
        self._camera_side = 'both'  # 'right', 'left', 'both'
        self._detect_phase = 'right'
        self._future = None
        self._detect_count = 0
        self._accumulated = []
        self._wait_until = 0.0
        self._start_time = 0.0
        self._error = None

    def start(self, camera_side='both'):
        """검출 시작.

        Args:
            camera_side: 'right' (cam=2만), 'left' (cam=1만), 'both' (교대)
        """
        self._camera_side = camera_side
        if camera_side == 'left':
            self._detect_phase = 'left'
        else:
            self._detect_phase = 'right'
        self._future = None
        self._detect_count = 0
        self._accumulated = []
        self._wait_until = 0.0
        self._start_time = time.monotonic()
        self._error = None

    def update(self):
        """매 tick 호출. 검출 진행.

        Returns:
            None: 진행 중
            list: 검출 완료 시 포인트 리스트 [(label, x, y, side), ...]
            'error': 검출 실패 시 (에러 메시지는 self.error로 접근)
        """
        logger = self._node.get_logger()
        max_tries = self._p['multi_detect_tries']
        delay = self._p['multi_detect_delay']

        # 대기 중
        if time.monotonic() < self._wait_until:
            return None

        # 서비스 호출
        if self._future is None:
            if not self._client.service_is_ready():
                if time.monotonic() - self._start_time >= 5.0:
                    self._error = '검출 서비스 미가용'
                    return 'error'
                return None

            cam_sel, cam_label = self._get_camera_selection()

            request = DetectCrossings.Request()
            request.camera_selection = cam_sel
            request.confidence_threshold = self._p['detection_confidence']
            request.expected_count = 3
            self._future = self._client.call_async(request)

            if self._should_count():
                self._detect_count += 1
            logger.info(
                f'멀티검출 [{self._detect_count}/{max_tries}] '
                f'({cam_label}) 서비스 호출...')
            return None

        # 응답 대기
        if not self._future.done():
            return None

        # 응답 처리
        try:
            result = self._future.result()
            self._future = None
            cam_label = 'R' if self._detect_phase == 'right' else 'L'

            if result.success and len(result.grid.detections) > 0:
                dets = list(result.grid.detections)
                logger.info(
                    f'  [{self._detect_count}/{max_tries}] '
                    f'({cam_label}) {len(dets)}개 검출 '
                    f'({result.detection_time_ms:.0f}ms)')
                for det in dets:
                    cam_tag = 'R' if det.camera_id == 1 else 'L'
                    logger.info(
                        f'    누적: ({det.x:.1f}, {det.y:.1f}) [{cam_tag}]')
                    self._accumulated.append({
                        'x': det.x,
                        'y': det.y,
                        'camera_id': det.camera_id,
                    })
            else:
                msg_text = result.message if result else '응답 없음'
                logger.warn(
                    f'  [{self._detect_count}/{max_tries}] '
                    f'({cam_label}) 검출 실패: {msg_text}')

            # 카메라 교대 (both 모드)
            if self._camera_side == 'both':
                if self._detect_phase == 'right':
                    self._detect_phase = 'left'
                    return None
                else:
                    self._detect_phase = 'right'

            # 조기 종료 체크
            if self._detect_count >= 2:
                if self._check_early_exit():
                    return self._finalize()

            # 최대 시도 도달
            if self._detect_count >= max_tries:
                if not self._accumulated:
                    self._error = f'멀티검출 {max_tries}회 모두 실패'
                    return 'error'
                return self._finalize()

            # 다음 검출 대기
            self._wait_until = time.monotonic() + delay
            return None

        except Exception as e:
            self._future = None
            self._error = f'검출 서비스 예외: {e}'
            return 'error'

    @property
    def error(self):
        return self._error

    @property
    def accumulated_points(self):
        return self._accumulated

    def _get_camera_selection(self):
        """현재 phase에 따른 카메라 선택값 반환."""
        if self._detect_phase == 'right':
            return 2, 'R'
        else:
            return 1, 'L'

    def _should_count(self):
        """검출 카운트 증가 여부."""
        if self._camera_side == 'both':
            return self._detect_phase == 'right'
        return True

    def _check_early_exit(self):
        """양쪽 카메라 충분한 클러스터 확보 시 True."""
        expected = self._p['expected_points_per_cam']

        if self._camera_side == 'both':
            right_pts = [p for p in self._accumulated if p['camera_id'] == 1]
            left_pts = [p for p in self._accumulated if p['camera_id'] == 0]
            right_clusters = self._cluster_points(right_pts)
            left_clusters = self._cluster_points(left_pts)
            if (len(right_clusters) >= expected
                    and len(left_clusters) >= expected):
                self._node.get_logger().info(
                    f'  양쪽 카메라 각 {expected}개 클러스터 확보 → 조기 종료')
                return True
        else:
            clusters = self._cluster_points(self._accumulated)
            if len(clusters) >= expected:
                self._node.get_logger().info(
                    f'  {expected}개 클러스터 확보 → 조기 종료')
                return True
        return False

    def _finalize(self):
        """누적 데이터 → 클러스터링 → 보간 → 추세선 → 최종 포인트."""
        logger = self._node.get_logger()
        flog = self._node.flog

        right_pts = [p for p in self._accumulated if p['camera_id'] == 1]
        left_pts = [p for p in self._accumulated if p['camera_id'] == 0]

        logger.info(
            f'멀티검출 완료: {len(self._accumulated)}개 누적 '
            f'(R:{len(right_pts)}, L:{len(left_pts)})')
        flog(f"DETECT_RAW: total={len(self._accumulated)} "
             f"right={len(right_pts)} left={len(left_pts)}")
        for p in self._accumulated:
            flog(f"  RAW: cam={p['camera_id']} "
                 f"x={p['x']:.1f} y={p['y']:.1f} "
                 f"conf={p.get('confidence', '?')}")

        right_final = self._process_camera_points(right_pts, 'right')
        left_final = self._process_camera_points(left_pts, 'left')

        if not right_final and not left_final:
            self._error = '유효한 교차점 없음 (클러스터링 후)'
            return 'error'

        points = self._build_final_points(right_final, left_final)

        if len(points) == 0:
            self._error = '유효한 교차점 없음 (범위 필터링 후)'
            return 'error'

        return points

    def _process_camera_points(self, pts, camera_side):
        """카메라 한쪽의 누적 포인트를 클러스터링 → 보간 → 추세선 보정."""
        logger = self._node.get_logger()
        flog = self._node.flog
        p = self._p

        if not pts:
            logger.warn(f'  [{camera_side}] 검출 포인트 없음')
            return []

        clusters = self._cluster_points(pts)
        logger.info(
            f'  [{camera_side}] 클러스터링: {len(pts)}개 → {len(clusters)}개')
        flog(f"DETECT_CLUSTER: [{camera_side}] raw={len(pts)} → clusters={len(clusters)}")
        for i, c in enumerate(clusters):
            flog(f"  CLUSTER[{i}]: x={c['x']:.1f} y={c['y']:.1f} hits={c['hit_count']}")

        # 범위 외 클러스터 사전 제거
        margin = 30.0
        if camera_side == 'right':
            y_min_limit = p['right_cam_y_min'] - margin
            y_max_limit = p['right_cam_y_max'] + margin
        else:
            y_min_limit = p['left_cam_y_min'] - margin
            y_max_limit = p['left_cam_y_max'] + margin

        x_min_limit = p.get('detect_x_min_mm', -margin)  # 검출 X 최소값 (기본 -margin)
        valid_clusters = []
        for c in clusters:
            if (c['x'] < x_min_limit or c['x'] > p['max_stage_x_mm'] + margin
                    or c['y'] < y_min_limit or c['y'] > y_max_limit):
                logger.info(
                    f'  [{camera_side}] 범위 외 클러스터 제거: '
                    f'X={c["x"]:.1f}, Y={c["y"]:.1f}')
                flog(f"  FILTERED_OUT: [{camera_side}] x={c['x']:.1f} y={c['y']:.1f} "
                     f"(range: x=[{x_min_limit:.0f}~{p['max_stage_x_mm']+margin:.0f}] "
                     f"y=[{y_min_limit:.0f}~{y_max_limit:.0f}])")
                continue
            valid_clusters.append(c)
        clusters = valid_clusters

        # 보간
        before = len(clusters)
        clusters = self._interpolate_missing(clusters)
        interp_count = len(clusters) - before
        if interp_count > 0:
            logger.info(
                f'  [{camera_side}] 보간: {interp_count}개 추론 추가')
        flog(f"DETECT_INTERP: [{camera_side}] before={before} after={len(clusters)} "
             f"interpolated={interp_count}")

        # X 내림차순 정렬
        clusters.sort(key=lambda c: c['x'], reverse=True)

        # 추세선 피팅 + 보정
        trendline = self._fit_trendline(clusters)
        if trendline is not None:
            corrected = trendline['corrected']
            avg_err = np.mean(trendline['errors'])
            slope, intercept, axis = trendline['line_eq']
            eq = f'Y={slope:.4f}*X+{intercept:.1f}' if axis == 'x' else f'X={slope:.4f}*Y+{intercept:.1f}'
            logger.info(
                f'  [{camera_side}] 추세선: {eq}, 평균오차: {avg_err:.1f}mm')
            for i, (cx, cy) in enumerate(corrected):
                orig = clusters[i]
                src = 'interp' if orig.get('source') == 'interpolated' else f'hits:{orig["hit_count"]}'
                logger.info(
                    f'    P{i+1}: ({orig["x"]:.1f},{orig["y"]:.1f}) → '
                    f'({cx:.1f},{cy:.1f}) err:{trendline["errors"][i]:.1f}mm [{src}]')

            # 이상치 제거 + 재피팅
            outlier_threshold = 100.0
            outlier_indices = [i for i, e in enumerate(trendline['errors']) if e > outlier_threshold]
            if outlier_indices and len(clusters) - len(outlier_indices) >= 2:
                for idx in outlier_indices:
                    logger.warn(
                        f'  [{camera_side}] 이상치 제거: P{idx+1} '
                        f'({clusters[idx]["x"]:.1f}, {clusters[idx]["y"]:.1f}) '
                        f'err:{trendline["errors"][idx]:.1f}mm > {outlier_threshold}mm')
                clusters = [c for i, c in enumerate(clusters) if i not in outlier_indices]
                trendline2 = self._fit_trendline(clusters)
                if trendline2 is not None:
                    corrected = trendline2['corrected']
                    avg_err2 = np.mean(trendline2['errors'])
                    s2, i2, ax2 = trendline2['line_eq']
                    eq2 = f'Y={s2:.4f}*X+{i2:.1f}' if ax2 == 'x' else f'X={s2:.4f}*Y+{i2:.1f}'
                    logger.info(
                        f'  [{camera_side}] 재피팅: {eq2}, '
                        f'평균오차: {avg_err:.1f}→{avg_err2:.1f}mm')
            return corrected
        else:
            logger.info(f'  [{camera_side}] 추세선 불가 → 원본 좌표 사용')
            return [(c['x'], c['y']) for c in clusters]

    def _build_final_points(self, right_coords, left_coords):
        """범위 필터링 + 라벨 부여."""
        logger = self._node.get_logger()
        flog = self._node.flog
        p = self._p
        points = []

        left_coords_asc = list(reversed(left_coords))
        for side, coords in [('right', right_coords), ('left', left_coords_asc)]:
            if side == 'right':
                y_min_safe = p['right_cam_y_min']
                y_max_safe = p['right_cam_y_max']
            else:
                y_min_safe = p['left_cam_y_min']
                y_max_safe = p['left_cam_y_max']
            for x, y in coords:
                if x < 0 or y < 0:
                    logger.info(f'  음수 스킵: X={x:.1f}mm, Y={y:.1f}mm')
                    continue
                if x > p['max_stage_x_mm'] or y > p['max_stage_y_mm']:
                    logger.warn(f'  범위 외 제거: X={x:.1f}, Y={y:.1f} (초과)')
                    continue
                if y < y_min_safe or y > y_max_safe:
                    logger.info(
                        f'  [{side}] Y 범위 외 스킵: Y={y:.1f}mm '
                        f'(허용: {y_min_safe}~{y_max_safe}mm)')
                    flog(f"DETECT_Y_FILTER: [{side}] x={x:.1f} y={y:.1f} "
                         f"(range: {y_min_safe}~{y_max_safe})")
                    continue
                points.append((None, x, y, side))

        labeled = []
        for idx, (_, x, y, side) in enumerate(points, 1):
            labeled.append((f'P{idx}', x, y, side))

        return labeled

    def _cluster_points(self, pts):
        """거리 기반 클러스터링."""
        dist_mm = self._p['cluster_distance_mm']
        used = set()
        clusters = []
        for i in range(len(pts)):
            if i in used:
                continue
            group = [i]
            used.add(i)
            for j in range(i + 1, len(pts)):
                if j in used:
                    continue
                dx = pts[i]['x'] - pts[j]['x']
                dy = pts[i]['y'] - pts[j]['y']
                if (dx * dx + dy * dy) ** 0.5 < dist_mm:
                    group.append(j)
                    used.add(j)
            clusters.append(group)

        results = []
        for group in clusters:
            gp = [pts[i] for i in group]
            results.append({
                'x': np.mean([p['x'] for p in gp]),
                'y': np.mean([p['y'] for p in gp]),
                'hit_count': len(group),
                'source': 'detected',
            })
        return results

    def _interpolate_missing(self, clusters):
        """클러스터가 기대 수보다 적으면 보간."""
        expected = self._p['expected_points_per_cam']
        max_x = self._p['max_stage_x_mm']
        x_min = self._p.get('detect_x_min_mm', 0.0)  # 보간 X 하한 (검출 필터와 동일)
        spacing = self._p['rebar_spacing_mm']
        logger = self._node.get_logger()

        if len(clusters) >= expected or len(clusters) < 1:
            return clusters

        clusters.sort(key=lambda c: c['x'], reverse=True)
        x_vals = [c['x'] for c in clusters]
        y_avg = np.mean([c['y'] for c in clusters])

        if len(clusters) == 1:
            base_x = clusters[0]['x']
            for cx in [base_x - spacing, base_x + spacing]:
                if x_min <= cx <= max_x and len(clusters) < expected:
                    logger.info(
                        f'  보간(1pt): X={cx:.1f}mm 추가 '
                        f'(기준 {base_x:.1f}mm ± {spacing}mm)')
                    clusters.append({
                        'x': cx, 'y': y_avg,
                        'hit_count': 0, 'source': 'interpolated',
                    })

        elif len(clusters) == 2:
            x_max, x_min = max(x_vals), min(x_vals)
            gap = x_max - x_min

            if gap > spacing * 1.5:
                mid_x = (x_max + x_min) / 2.0
                logger.info(
                    f'  보간: 간격 {gap:.0f}mm ≈ 2*{spacing}mm → '
                    f'중간 X={mid_x:.1f}mm 추가')
                clusters.append({
                    'x': mid_x, 'y': y_avg,
                    'hit_count': 0, 'source': 'interpolated',
                })
            else:
                candidate_high = x_max + spacing
                candidate_low = x_min - spacing
                high_valid = x_min <= candidate_high <= max_x
                low_valid = x_min <= candidate_low <= max_x
                if high_valid and not low_valid:
                    new_x = candidate_high
                elif low_valid and not high_valid:
                    new_x = candidate_low
                elif high_valid and low_valid:
                    center = max_x / 2.0
                    new_x = (candidate_high
                             if abs(candidate_high - center)
                             < abs(candidate_low - center)
                             else candidate_low)
                else:
                    new_x = x_max + spacing
                logger.info(
                    f'  보간: 간격 {gap:.0f}mm ≈ 1*{spacing}mm → '
                    f'X={new_x:.1f}mm 추가')
                clusters.append({
                    'x': new_x, 'y': y_avg,
                    'hit_count': 0, 'source': 'interpolated',
                })

        clusters.sort(key=lambda c: c['x'], reverse=True)
        return clusters

    def _fit_trendline(self, clusters):
        """직선 추세선 피팅."""
        pts = [(c['x'], c['y']) for c in clusters]
        if len(pts) < 2:
            return None

        xs = np.array([p[0] for p in pts])
        ys = np.array([p[1] for p in pts])

        x_range = xs.max() - xs.min()
        y_range = ys.max() - ys.min()

        if x_range >= y_range:
            coeffs = np.polyfit(xs, ys, 1)
            slope, intercept = coeffs
            axis = 'x'
            a, b, c = slope, -1.0, intercept
        else:
            coeffs = np.polyfit(ys, xs, 1)
            slope, intercept = coeffs
            axis = 'y'
            a, b, c = -1.0, slope, intercept

        corrected = []
        errors = []
        for x0, y0 in pts:
            x_proj = (b * (b * x0 - a * y0) - a * c) / (a * a + b * b)
            y_proj = (a * (-b * x0 + a * y0) - b * c) / (a * a + b * b)
            corrected.append((x_proj, y_proj))
            dist = abs(a * x0 + b * y0 + c) / np.sqrt(a * a + b * b)
            errors.append(dist)

        return {
            'corrected': corrected,
            'line_eq': (slope, intercept, axis),
            'errors': errors,
        }
