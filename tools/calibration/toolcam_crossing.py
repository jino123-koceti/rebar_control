#!/usr/bin/env python3
"""툴캠(zedxone) 교차점 검출기 — classical CV.
목표픽셀 주변 ROI에서 두 철근 직선(우세 2방향) fit → 교차점.
클로즈드루프 미세보정용 (학습 불필요).

단독 테스트: python3 toolcam_crossing.py <이미지>  (저장이미지로 검출 검증)
라이브:     python3 toolcam_crossing.py --live      (zedxone 스트림 + 안정성 확인)
"""
import sys
import numpy as np
import cv2

TARGET = (477, 886)   # 결속 성공 목표픽셀 (전체뷰 1920x1200 기준)
TOOTH = (402, 731)


def _fit_line(group):
    pts = []
    for (x1, y1, x2, y2, a, ln) in group:
        pts += [[x1, y1], [x2, y2]]
    pts = np.array(pts, np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    return float(x0), float(y0), float(vx), float(vy)


def _intersect(L1, L2):
    x1, y1, vx1, vy1 = L1
    x2, y2, vx2, vy2 = L2
    A = np.array([[vx1, -vx2], [vy1, -vy2]])
    if abs(np.linalg.det(A)) < 1e-6:
        return None
    t, _ = np.linalg.solve(A, np.array([x2 - x1, y2 - y1]))
    return (x1 + t * vx1, y1 + t * vy1)


def detect_crossing(img, target=TARGET, roi=320, debug=False):
    """목표픽셀 주변 ROI에서 두 철근방향 직선의 교차점 반환. (cx,cy) or None."""
    H, W = img.shape[:2]
    tx, ty = target
    x0 = max(0, tx - roi); y0 = max(0, ty - roi)
    x1 = min(W, tx + roi); y1 = min(H, ty + roi)
    sub = img[y0:y1, x0:x1]
    g = cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY)
    g = cv2.GaussianBlur(g, (5, 5), 0)
    # 철근=어두움 → Otsu로 분리 후 morphology 정리 (홀/브래킷 잡음 억제)
    _, dark = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    edges = cv2.Canny(dark, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 60,
                            minLineLength=120, maxLineGap=25)
    if lines is None:
        return None
    segs = []
    for l in lines:
        a, bb, c, d = l[0]
        ang = np.degrees(np.arctan2(d - bb, c - a)) % 180
        ln = np.hypot(c - a, d - bb)
        segs.append((a, bb, c, d, ang, ln))
    angles = np.array([s[4] for s in segs])
    lens = np.array([s[5] for s in segs])
    hist, _ = np.histogram(angles, bins=36, range=(0, 180), weights=lens)
    p1 = int(np.argmax(hist)); a1 = (p1 + 0.5) * 5
    h2 = hist.copy()
    for i in range(36):
        d = min(abs(i - p1), 36 - abs(i - p1))
        if d * 5 < 30:
            h2[i] = 0
    p2 = int(np.argmax(h2)); a2 = (p2 + 0.5) * 5
    if h2[p2] == 0:
        return None

    def adist(a, peak):
        d = abs(a - peak); return min(d, 180 - d)
    g1 = [s for s in segs if adist(s[4], a1) <= adist(s[4], a2)]
    g2 = [s for s in segs if adist(s[4], a1) > adist(s[4], a2)]
    if len(g1) < 1 or len(g2) < 1:
        return None
    # 목표(ROI 로컬좌표) 근처를 지나는 철근만 골라 fit → 그 교차점 = 타깃 교차점
    tloc = (tx - x0, ty - y0)

    def line_dist(seg, pt):
        x1s, y1s, x2s, y2s = seg[:4]
        dx, dy = x2s - x1s, y2s - y1s
        L = np.hypot(dx, dy) + 1e-9
        return abs((pt[0]-x1s)*dy - (pt[1]-y1s)*dx) / L

    def near(group, band):
        sel = [s for s in group if line_dist(s, tloc) < band]
        return sel if len(sel) >= 1 else None

    for band in (45, 70, 110):
        n1, n2 = near(g1, band), near(g2, band)
        if n1 and n2:
            g1, g2 = n1, n2
            break
    L1 = _fit_line(g1); L2 = _fit_line(g2)
    inter = _intersect(L1, L2)
    if inter is None:
        return None
    cx, cy = inter[0] + x0, inter[1] + y0
    # ROI 밖이면 무효
    if not (x0 <= cx <= x1 and y0 <= cy <= y1):
        return None
    if debug:
        return (cx, cy), (a1, a2, len(g1), len(g2), (x0, y0, x1, y1))
    return (cx, cy)


def _draw(img, cross, target=TARGET):
    out = img.copy()
    cv2.drawMarker(out, (int(target[0]), int(target[1])), (255, 0, 255),
                   cv2.MARKER_TILTED_CROSS, 40, 3)
    cv2.putText(out, "TARGET", (int(target[0]) + 22, int(target[1])),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
    if cross is not None:
        cv2.circle(out, (int(cross[0]), int(cross[1])), 16, (0, 255, 0), 3)
        err = np.hypot(cross[0] - target[0], cross[1] - target[1])
        cv2.line(out, (int(cross[0]), int(cross[1])),
                 (int(target[0]), int(target[1])), (0, 255, 255), 2)
        cv2.putText(out, f"cross ({cross[0]:.0f},{cross[1]:.0f}) err={err:.0f}px",
                    (int(cross[0]) + 20, int(cross[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return out


def main():
    if '--live' in sys.argv:
        import rclpy, time
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        rclpy.init(); node = Node('toolcam'); br = CvBridge(); buf = []
        node.create_subscription(
            Image, '/zedxone/zed_node/rgb/rect/image',
            lambda m: buf.append(br.imgmsg_to_cv2(m, 'bgr8')),
            qos_profile_sensor_data)
        print("라이브 — 10라운드 검출 안정성:")
        xs, ys = [], []
        for rd in range(10):
            buf.clear(); t = time.time()
            while not buf and time.time() - t < 3:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not buf:
                continue
            c = detect_crossing(buf[-1])
            if c:
                xs.append(c[0]); ys.append(c[1])
                err = np.hypot(c[0]-TARGET[0], c[1]-TARGET[1])
                print(f"  R{rd+1}: cross=({c[0]:.0f},{c[1]:.0f}) 목표오차={err:.0f}px")
            else:
                print(f"  R{rd+1}: 검출 실패")
            cv2.imwrite('/home/koceti/ros2_ws/data/calibration/toolcam_live.png',
                        _draw(buf[-1], c))
        if xs:
            print(f"\n안정성: x std={np.std(xs):.1f}px y std={np.std(ys):.1f}px "
                  f"(검출 {len(xs)}/10)")
        node.destroy_node(); rclpy.shutdown()
    else:
        path = sys.argv[1] if len(sys.argv) > 1 else \
            '/home/koceti/ros2_ws/data/calibration/zedxone_tyingpoint_UP.png'
        img = cv2.imread(path)
        res = detect_crossing(img, debug=True)
        if res and isinstance(res, tuple) and len(res) == 2 and isinstance(res[1], tuple):
            (cx, cy), info = res
            print(f"{path}: 교차점 ({cx:.0f},{cy:.0f})  "
                  f"각도={info[0]:.0f}/{info[1]:.0f}°  세그={info[2]}/{info[3]}  "
                  f"오차={np.hypot(cx-TARGET[0],cy-TARGET[1]):.0f}px")
            out = '/home/koceti/ros2_ws/data/calibration/toolcam_detect.png'
            cv2.imwrite(out, _draw(img, (cx, cy)))
            print(f"저장: {out}")
        else:
            print(f"{path}: 검출 실패")
            cv2.imwrite('/home/koceti/ros2_ws/data/calibration/toolcam_detect.png',
                        _draw(img, None))


if __name__ == '__main__':
    main()
