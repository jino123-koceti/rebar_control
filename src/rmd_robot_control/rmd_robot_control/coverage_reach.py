#!/usr/bin/env python3
"""L6 — 도달 판정. **ROS 무관 순수 모듈.** `coverage_map` 에서 떼어낸 것이다.

정준 좌표(1번 자세 스테이지 mm) ↔ 카메라 좌표 변환과, "그 점을 어느 자세로
결속할 수 있나" 를 캘리 모델 + 가동범위로 판정한다. 좌표 규약은 `coverage_map`
머리말에 있다.

쪼갠 이유: `coverage_map` 이 R4 한도(850줄)를 넘었고, 의존이 한 방향
(`coverage_map` → `coverage_reach`)이라 여기가 자연스러운 경계였다.
"""
import os

# ──────────────────────────────────────────────────────────────────────
# 도달 판정
# ──────────────────────────────────────────────────────────────────────

class Reach:
    """정준 좌표 (du, dv) 에 **어느 자세로 닿는지**.

    캘리 모델(자세별 A·b)과 가동범위(자세별 X·Y 한계)로 판정한다.
    `tying_planner` 가 실행 시점에 하는 것과 같은 판정을, 계획 시점에
    미리 해 보는 것이다.

    ⚠ 깊이는 **평면식**으로 대체한다. 교차점은 한 평면 위에 있고
      (뷰 안 평면이탈 0~4.3mm), 깊이는 cam_y 와 상관 -0.994 다. 그래서
      depth = p0 + px·cam_x + py·cam_y 로 충분하다. 기본값은 짝 83개
      (PLANE_SINCE 이후) 적합으로 잔차 RMS 6.7mm.
      ⚠ **배근 높이가 바뀌면 다시 적합해야 한다** — 캘리 모델과 같은 전제다.
    """

    # 깊이 평면 (실측 2026-10-17, 짝 83개)
    PLANE = (448.6, 0.0036, -0.5557)

    def __init__(self, poses_ab, envelope, canon=1, plane=None):
        """poses_ab : {자세: (A 3x3 리스트, b 3 리스트)}
        envelope   : {자세: {'x': [lo, hi], 'y': [lo, hi]}} + 여유 적용 완료
        canon      : 정준 좌표로 쓸 자세
        """
        self.ab = poses_ab
        self.env = envelope
        self.canon = canon
        self.plane = plane or self.PLANE
        p0, px, py = self.plane
        A, b = self.ab[canon]
        # 정준 (U, V) 는 (cam_x, cam_y) 의 아핀 — 깊이를 평면식으로 소거한 것
        #   U = A[0]·(cx, cy, p0+px·cx+py·cy) + b[0]
        self._M = (
            (A[0][0] + A[0][2] * px, A[0][1] + A[0][2] * py, A[0][2] * p0 + b[0]),
            (A[1][0] + A[1][2] * px, A[1][1] + A[1][2] * py, A[1][2] * p0 + b[1]),
        )
        det = self._M[0][0] * self._M[1][1] - self._M[0][1] * self._M[1][0]
        if abs(det) < 1e-9:
            raise ValueError('정준 변환이 특이행렬이다 — 캘리 모델을 확인할 것')
        self._det = det

    # ---- 변환 ----
    def canon_of_cam(self, cx, cy):
        """카메라 좌표(mm) → 정준 (U, V)."""
        m = self._M
        return (m[0][0] * cx + m[0][1] * cy + m[0][2],
                m[1][0] * cx + m[1][1] * cy + m[1][2])

    def cam_of_canon(self, u, v):
        """정준 (U, V) → 카메라 좌표 (cx, cy). 위 2x2 의 역."""
        m = self._M
        au, av = u - m[0][2], v - m[1][2]
        cx = (m[1][1] * au - m[0][1] * av) / self._det
        cy = (m[0][0] * av - m[1][0] * au) / self._det
        return cx, cy

    def stage_of_canon(self, pose, u, v):
        """정준 (U, V) → 그 자세로 보낼 스테이지 (X, Y)."""
        cx, cy = self.cam_of_canon(u, v)
        p0, px, py = self.plane
        dz = p0 + px * cx + py * cy
        A, b = self.ab[pose]
        return (A[0][0] * cx + A[0][1] * cy + A[0][2] * dz + b[0],
                A[1][0] * cx + A[1][1] * cy + A[1][2] * dz + b[1])

    # ---- 판정 ----
    def pose_for(self, du, dv):
        """그 점을 결속할 수 있는 자세. 못 가면 None.

        ⚠ 자세 순서가 결과를 바꾼다 — 여러 자세로 닿는 점이 많다(계산상
          결속영역의 절반 이상). 번호 순으로 고르는 것은 **임의 선택**이고,
          실제 순회에서 자세 전환을 줄이는 선택은 `tying_planner` 가 한다
          (지금 자세로 갈 수 있으면 계속 그 자세로). 여기서는 "닿느냐" 만
          본다.
        """
        for p in sorted(self.ab):
            X, Y = self.stage_of_canon(p, du, dv)
            e = self.env.get(p)
            if e and e['x'][0] <= X <= e['x'][1] and e['y'][0] <= Y <= e['y'][1]:
                return p
        return None

    def image_fov(self, width=1280, height=800, K=None, pad_px=24):
        """**검출 시야**를 정준 좌표 외접 사각으로. (du0, du1, dv0, dv1)

        결속영역보다 훨씬 넓다 — 카메라는 스테이지가 닿는 곳보다 멀리 본다.
        이 값이 있어야 "아직 못 본 곳을 보러 간다"(탐색)를 계획할 수 있다.

        화소를 평면에 역투영해서 구한다. 평면식 depth = p0+px·cx+py·cy 와
        cx = (u-Cx)z/fx, cy = (v-Cy)z/fy 를 연립하면 z 가 닫힌 형태로 풀린다.

        ⚠ `pad_px` 로 테두리를 잘라낸다 — 화면 끝에 걸친 교차점은 검출이
          불안정하고, 깊이도 가장자리가 나쁘다.
        """
        fx, fy, Cx, Cy = K or (605.3041, 605.5226, 642.5536, 399.0131)
        p0, px, py = self.plane
        us, vs = [], []
        xs = list(range(pad_px, width - pad_px, 16)) + [width - 1 - pad_px]
        ys = list(range(pad_px, height - pad_px, 16)) + [height - 1 - pad_px]
        for ix in xs:
            for iy in (ys[0], ys[-1]):
                _acc(us, vs, self, ix, iy, fx, fy, Cx, Cy, p0, px, py)
        for iy in ys:
            for ix in (xs[0], xs[-1]):
                _acc(us, vs, self, ix, iy, fx, fy, Cx, Cy, p0, px, py)
        if not us:
            raise ValueError('시야 역투영이 전부 실패했다 — 평면식을 확인할 것')
        return min(us), max(us), min(vs), max(vs)

    def bbox(self, step=10.0):
        """도달 영역의 외접 사각 (du0, du1, dv0, dv1). 훑어서 구한다."""
        us, vs = [], []
        for u in _frange(-600, 601, step):
            for v in _frange(-600, 601, step):
                if self.pose_for(u, v) is not None:
                    us.append(u); vs.append(v)
        if not us:
            raise ValueError('도달 가능한 점이 없다 — 모델/가동범위를 확인할 것')
        return min(us), max(us), min(vs), max(vs)

    @classmethod
    def from_files(cls, model_path=None, axes_path=None, canon=1):
        """`stage_camera.yaml` + `axes.yaml` 에서 만든다."""
        import yaml
        root = os.path.expanduser('~/ros2_ws/src/rebar_control')
        model_path = model_path or f'{root}/data/calibration/stage_camera.yaml'
        axes_path = axes_path or f'{root}/src/rebar_base_control/config/axes.yaml'
        M = yaml.safe_load(open(model_path))
        ax = yaml.safe_load(open(axes_path))
        env_raw = _find_key(ax, 'envelope')
        if env_raw is None:
            raise ValueError(f'{axes_path} 에 envelope 가 없다')
        margin = float(env_raw.get('margin_mm', 10.0))
        poses_ab, env = {}, {}
        for k, v in M['poses'].items():
            p = int(k)
            e = env_raw['poses'].get(p) or env_raw['poses'].get(str(p))
            if e is None:
                continue
            poses_ab[p] = (v['A'], v['b'])
            # 하한 X 는 기계 끝단이라 여유를 빼지 않는다 (axes.yaml 주석 참조)
            env[p] = {'x': [e['x'][0], e['x'][1] - margin],
                      'y': [e['y'][0] + margin, e['y'][1] - margin]}
        return cls(poses_ab, env, canon=canon)


def _acc(us, vs, reach, ix, iy, fx, fy, Cx, Cy, p0, px, py):
    """화소 하나를 평면에 역투영해 정준 좌표로. 평면 뒤로 가면 버린다."""
    k = 1.0 - px * (ix - Cx) / fx - py * (iy - Cy) / fy
    if k <= 1e-6:
        return                       # 시선이 평면과 평행하거나 뒤를 본다
    z = p0 / k
    if not (50.0 < z < 5000.0):
        return
    u, v = reach.canon_of_cam((ix - Cx) * z / fx, (iy - Cy) * z / fy)
    us.append(u); vs.append(v)


def _frange(a, b, s):
    x = float(a)
    while x < b:
        yield x
        x += s


def _find_key(d, key):
    if isinstance(d, dict):
        if key in d:
            return d[key]
        for v in d.values():
            r = _find_key(v, key)
            if r is not None:
                return r
    return None
