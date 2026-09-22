#!/usr/bin/env python3
"""deck_edge 판정을 시각화한다 — 왜 그 판정이 나왔는지 눈으로 확인용.

## 왜
`rebar_frac=0.00 배근이탈`처럼 판정이 이상할 때, 숫자만으로는 원인을 못 찾는다.
세그 마스크를 원본에 겹쳐 보면 **모델이 철근을 어디까지 봤는지** 즉시 드러난다.

## 판정 로직 (deck_edge.rebar_edge)
1. 중앙 컬럼(좌우 20% 제외)에서 **행별 철근 비율** r(y) 계산
2. r(y) ≥ ROW_THR(0.015) 인 행 = "그 행에 배근 있음"
3. **발밑 게이트**: 배근이 있는 최하단 행이 화면 하단 45% 안에 없으면
   → `on_rebar=False`, `rebar_frac=0.0` ("배근이탈")   ← 대부분의 0.00은 여기
4. 통과하면 발밑에서 위로 스캔, 연속 무철근이 H의 9%를 넘으면 거기가 데크끝

## 저장물
  *_raw.jpg      원본
  *_overlay.jpg  세그 마스크 오버레이 + 판정선(발밑 게이트/데크끝) + 중앙컬럼
  *_profile.jpg  행별 철근비율 그래프 (ROW_THR 기준선 포함)

사용:
    python3 tools/vision_test/deck_edge_debug.py                 # front/back 둘 다
    python3 tools/vision_test/deck_edge_debug.py --cam front
"""
import argparse
import os
import time

OUT = '/home/koceti/ros2_ws/data/deck_edge_debug'
TOPIC = {
    'front': '/zedxmini2/zed_node/rgb/color/rect/image/compressed',
    'back': '/zedxmini1/zed_node/rgb/color/rect/image/compressed',
}
# 클래스별 색 (BGR) — 철근 계열만 강조하고 나머지는 흐리게
CLS_COLOR = {
    0: (60, 60, 60),      # background(방수포)
    1: (90, 90, 90),      # floor
    2: (0, 0, 220),       # human
    3: (0, 120, 220),     # obstacle
    4: (60, 200, 60),     # rebar_h  ← 철근
    5: (220, 160, 40),    # rebar_v  ← 철근
    6: (120, 60, 120),    # wall
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', default='both', choices=['front', 'back', 'both'])
    ap.add_argument('--wait', type=float, default=15.0)
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()

    import cv2
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    import rebar_vision.deck_edge as de
    from rebar_vision.deck_edge_node import DEFAULT_WEIGHTS
    from rebar_vision.rebar_seg import RebarSegmenter

    cams = ['front', 'back'] if a.cam == 'both' else [a.cam]
    rclpy.init()
    node = rclpy.create_node('deck_edge_debug')
    br = CvBridge()
    got = {}

    def mk(k):
        def cb(m):
            got[k] = br.compressed_imgmsg_to_cv2(m, 'bgr8')
        return cb

    for k in cams:
        node.create_subscription(CompressedImage, TOPIC[k], mk(k),
                                 qos_profile_sensor_data)
    print('영상 대기…')
    t0 = time.time()
    while time.time() - t0 < a.wait and len(got) < len(cams):
        rclpy.spin_once(node, timeout_sec=0.2)
    missing = [k for k in cams if k not in got]
    if missing:
        print(f'❌ 영상 없음: {missing}')
    if not got:
        node.destroy_node(); rclpy.shutdown(); return

    seg = RebarSegmenter(DEFAULT_WEIGHTS, 'cuda', 640)
    os.makedirs(a.out, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')

    for k in cams:
        if k not in got:
            continue
        img = got[k]
        mask = seg.predict(img)
        if mask.shape != img.shape[:2]:
            img = cv2.resize(img, (mask.shape[1], mask.shape[0]))
        H, W = mask.shape
        edge_row, frac, on_rebar = de.rebar_edge(mask)
        prof = de.rebar_profile(mask)

        # ── 오버레이 ────────────────────────────────────
        color = np.zeros_like(img)
        for ci, c in CLS_COLOR.items():
            color[mask == ci] = c
        ov = cv2.addWeighted(img, 0.55, color, 0.45, 0)

        c0, c1 = int(W * de.CENTER[0]), int(W * de.CENTER[1])
        cv2.rectangle(ov, (c0, 0), (c1, H - 1), (255, 255, 255), 1)   # 중앙 컬럼

        anchor = int(H * (1.0 - de.ANCHOR_FRAC))
        cv2.line(ov, (0, anchor), (W, anchor), (0, 200, 255), 2)
        cv2.putText(ov, 'anchor (below = foot)', (6, anchor - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)

        if on_rebar:
            cv2.line(ov, (0, edge_row), (W, edge_row), (0, 0, 255), 2)
            cv2.putText(ov, f'edge_row {edge_row}', (6, edge_row - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 판정 요약
        verdict = ('STOP' if frac < de.STOP_THR else
                   'SLOW' if frac < de.SLOW_THR else 'GO')
        lines = [f'{k}  {verdict}',
                 f'rebar_frac {frac:.3f}  (STOP<{de.STOP_THR} SLOW<{de.SLOW_THR})',
                 f'on_rebar {on_rebar}' + ('' if on_rebar else '  <-- 배근이탈')]
        y = 24
        for s in lines:
            cv2.putText(ov, s, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(ov, s, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y += 24

        # ── 행별 프로파일 ───────────────────────────────
        pw = 320
        pl = np.full((H, pw, 3), 255, np.uint8)
        thr_x = int(de.ROW_THR / max(prof.max(), de.ROW_THR * 4) * (pw - 40))
        for yy in range(H):
            v = prof[yy] / max(prof.max(), de.ROW_THR * 4)
            x = int(v * (pw - 40))
            c = (60, 200, 60) if prof[yy] >= de.ROW_THR else (200, 200, 200)
            cv2.line(pl, (20, yy), (20 + x, yy), c, 1)
        cv2.line(pl, (20 + thr_x, 0), (20 + thr_x, H), (0, 0, 255), 1)
        cv2.putText(pl, 'ROW_THR', (20 + thr_x + 3, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        cv2.line(pl, (0, anchor), (pw, anchor), (0, 200, 255), 2)
        if on_rebar:
            cv2.line(pl, (0, edge_row), (pw, edge_row), (0, 0, 255), 1)

        pr = os.path.join(a.out, f'{stamp}_{k}_raw.jpg')
        po = os.path.join(a.out, f'{stamp}_{k}_overlay.jpg')
        pp = os.path.join(a.out, f'{stamp}_{k}_profile.jpg')
        cv2.imwrite(pr, img)
        cv2.imwrite(po, np.hstack([ov, pl]))
        cv2.imwrite(pp, pl)

        # ── 콘솔 요약 ───────────────────────────────────
        u, cnt = np.unique(mask, return_counts=True)
        dist = '  '.join(f'{de.__dict__.get("CLS_NAME", {}).get(int(i), int(i))}:'
                         f'{100.0 * n / mask.size:.1f}%'
                         for i, n in zip(u, cnt) if 100.0 * n / mask.size >= 1.0)
        sup = prof >= de.ROW_THR
        idx = np.flatnonzero(sup)
        print(f'\n■ {k}   {verdict}   rebar_frac={frac:.3f}  on_rebar={on_rebar}')
        print(f'  클래스 분포: {dist}')
        print(f'  배근 있는 행: {idx.size}/{H}'
              + (f'  (최하단 {int(idx[-1])}행, 발밑기준 {anchor}행)' if idx.size else ''))
        if idx.size and int(idx[-1]) < anchor:
            print(f'  ⚠ 최하단 배근행({int(idx[-1])})이 발밑기준({anchor})보다 위 '
                  f'→ **배근이탈 판정**')
        print(f'  저장: {po}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
