#!/usr/bin/env python3
"""Orbbec Gemini 2L 컬러 실시간 뷰어 (ROS 불필요, v4l2 직접).
카메라 위치/각도 조절용 — 중앙 십자 + 3분할 격자 오버레이.

  python3 orbbec_view.py                 # /dev/video13, 1280x800
  python3 orbbec_view.py --dev 13 --w 1280 --h 800

  s  스냅샷 저장 (/tmp/orbbec_snap.png)   g  격자 on/off   q/ESC 종료
"""
import argparse, time
import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dev', type=int, default=13, help='color v4l2 노드 (기본 13)')
    ap.add_argument('--w', type=int, default=1280)
    ap.add_argument('--h', type=int, default=800)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.h)
    if not cap.isOpened():
        print(f'❌ /dev/video{args.dev} 열기 실패'); return
    print(f'Orbbec color 뷰어 — /dev/video{args.dev}. s=저장 g=격자 q=종료')

    win = 'Orbbec Gemini2L color (s=save g=grid q=quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 800)
    grid = True
    t0, n, fps = time.time(), 0, 0.0
    try:
        while True:
            ok, fr = cap.read()
            if not ok or fr is None:
                time.sleep(0.02); continue
            H, W = fr.shape[:2]
            n += 1
            if n % 10 == 0:
                fps = 10.0 / (time.time() - t0); t0 = time.time()
            disp = fr.copy()
            if grid:
                # 3분할 격자
                for i in (1, 2):
                    cv2.line(disp, (W*i//3, 0), (W*i//3, H), (0, 200, 200), 1)
                    cv2.line(disp, (0, H*i//3), (W, H*i//3), (0, 200, 200), 1)
                # 중앙 십자
                cv2.drawMarker(disp, (W//2, H//2), (0, 255, 255),
                               cv2.MARKER_CROSS, 40, 2)
            cv2.putText(disp, f'{W}x{H}  {fps:4.1f}fps  mean{fr.mean():.0f}',
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(win, disp)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('g'):
                grid = not grid
            elif k == ord('s'):
                cv2.imwrite('/tmp/orbbec_snap.png', fr)
                print('  저장 /tmp/orbbec_snap.png')
    finally:
        cap.release(); cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
