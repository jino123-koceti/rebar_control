#!/usr/bin/env python3
"""Orbbec 컬러에서 철근 교차점 고전 CV 검출 프로토타입.
어두운 철근 라인 검출 → 가로/세로 그룹 → 교점 → 클러스터.

  python3 crossing_detect_cv.py /tmp/orbbec_color.png
저장: /tmp/crossing_cv.png (오버레이)
"""
import sys
import numpy as np
import cv2


def _cluster_1d(vals, gap):
    """1D 값 군집 (정렬 후 gap 기준). 반환 [(대표값, [원소인덱스]), ...]."""
    order = np.argsort(vals)
    groups = []
    cur = [order[0]]
    for k in order[1:]:
        if vals[k] - vals[cur[-1]] <= gap:
            cur.append(k)
        else:
            groups.append(cur); cur = [k]
    groups.append(cur)
    return [(float(np.mean(vals[g])), g) for g in groups]


def _fit_line(pts_xy):
    """점들에 직선 피팅 → (점, 방향단위벡터)."""
    vx, vy, x0, y0 = cv2.fitLine(pts_xy.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    return np.array([x0, y0]), np.array([vx, vy])


def _intersect(p1, d1, p2, d2):
    den = d1[0]*d2[1] - d1[1]*d2[0]
    if abs(den) < 1e-6:
        return None
    t = ((p2[0]-p1[0])*d2[1] - (p2[1]-p1[1])*d2[0]) / den
    return p1 + t*d1


def detect(img, roi=(0.12, 0.03, 0.88, 0.97), min_support=2):
    H, W = img.shape[:2]
    x0, y0, x1, y1 = int(roi[0]*W), int(roi[1]*H), int(roi[2]*W), int(roi[3]*H)
    cx_img, cy_img = W/2, H/2
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    dark = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                 cv2.THRESH_BINARY_INV, 51, 10)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    mask = np.zeros_like(dark); mask[y0:y1, x0:x1] = 255
    dark = cv2.bitwise_and(dark, mask)

    lines = cv2.HoughLinesP(dark, 1, np.pi/180, threshold=80,
                            minLineLength=int(W*0.18), maxLineGap=40)
    hs, vs = [], []   # (line, key) — key = 중심축에서의 위치
    if lines is not None:
        for l in lines[:, 0]:
            xa, ya, xb, yb = map(float, l)
            ang = np.degrees(np.arctan2(yb-ya, xb-xa)) % 180
            dx, dy = xb-xa, yb-ya
            if ang < 25 or ang > 155:       # 가로 → y@center-x
                if abs(dx) < 1e-3:
                    continue
                y_at = ya + (cx_img-xa)*dy/dx
                hs.append((l, y_at))
            elif 55 < ang < 125:            # 세로 → x@center-y
                if abs(dy) < 1e-3:
                    continue
                x_at = xa + (cy_img-ya)*dx/dy
                vs.append((l, x_at))

    def merge(items, gap):
        """같은 철근 라인 조각들을 군집→직선피팅. 반환 [(점,방향), ...]."""
        if not items:
            return []
        keys = np.array([k for _, k in items])
        out = []
        for _, idxs in _cluster_1d(keys, gap):
            if len(idxs) < min_support:
                continue
            pts = []
            for i in idxs:
                l = items[i][0]
                pts += [[l[0], l[1]], [l[2], l[3]]]
            out.append(_fit_line(np.array(pts)))
        return out

    Hlines = merge(hs, gap=25)    # 가로철근 (y간격 대비 작게)
    Vlines = merge(vs, gap=30)    # 세로철근

    # 밝기(데크) 판단용
    brightT = np.percentile(gray[y0:y1, x0:x1], 60)

    crossings = []
    for p1, d1 in Hlines:
        for p2, d2 in Vlines:
            pt = _intersect(p1, d1, p2, d2)
            if pt is None:
                continue
            x, y = pt
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            # 교차점 주변 데크가 밝은지 (프레임/어두운영역 오검출 제거)
            xi, yi = int(x), int(y)
            patch = gray[max(0, yi-30):yi+30, max(0, xi-30):xi+30]
            if patch.size and patch.max() > brightT:
                crossings.append((x, y))
    return crossings, [l for l, _ in hs], [l for l, _ in vs], dark, Hlines, Vlines


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/orbbec_color.png'
    img = cv2.imread(path)
    if img is None:
        print(f'이미지 없음: {path}'); return
    crossings, hori, vert, dark = detect(img)
    disp = img.copy()
    for l in hori:
        cv2.line(disp, (l[0], l[1]), (l[2], l[3]), (0, 180, 0), 1)
    for l in vert:
        cv2.line(disp, (l[0], l[1]), (l[2], l[3]), (255, 140, 0), 1)
    for (x, y) in crossings:
        cv2.circle(disp, (int(x), int(y)), 12, (0, 0, 255), 3)
    print(f'가로선 {len(hori)}, 세로선 {len(vert)}, 교차점 {len(crossings)}개')
    cv2.imwrite('/tmp/crossing_cv.png', disp)
    cv2.imwrite('/tmp/crossing_cv_mask.png', dark)
    print('저장: /tmp/crossing_cv.png (오버레이), /tmp/crossing_cv_mask.png (라인마스크)')


if __name__ == '__main__':
    main()
