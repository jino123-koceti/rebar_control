#!/usr/bin/env python3
"""툴캠 교차점 클릭 라벨러 → YOLO 포맷 저장.
이미지에서 교차점 중심을 좌클릭하면 고정크기 박스 생성. 우클릭=가장가까운 점 삭제.

키:
  좌클릭   교차점 추가      우클릭  가까운 점 삭제
  n/SPACE  저장+다음        p       이전
  u        마지막 점 취소   c       전부 지우기
  q        저장+종료

저장: data/toolcam_dataset/labels/tcam_NNNN.txt  (YOLO: class cx cy w h, 정규화)
"""
import os, glob, argparse
import cv2
import numpy as np

ROOT = '/home/koceti/ros2_ws/data/toolcam_dataset'
IMG_DIR = os.path.join(ROOT, 'images')
LBL_DIR = os.path.join(ROOT, 'labels')
TARGET = (477, 886)


def load_label(path, W, H):
    pts = []
    if os.path.exists(path):
        for line in open(path):
            p = line.split()
            if len(p) >= 5:
                cx, cy = float(p[1])*W, float(p[2])*H
                pts.append((cx, cy))
    return pts


def save_label(path, pts, W, H, box):
    with open(path, 'w') as f:
        for (cx, cy) in pts:
            f.write(f"0 {cx/W:.6f} {cy/H:.6f} {box/W:.6f} {box/H:.6f}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--box', type=int, default=130, help='박스 크기(px)')
    ap.add_argument('--scale', type=float, default=0.5)
    args = ap.parse_args()
    os.makedirs(LBL_DIR, exist_ok=True)
    images = sorted(glob.glob(os.path.join(IMG_DIR, 'tcam_*.png')))
    if not images:
        print('이미지 없음'); return
    print(f'{len(images)}장 라벨링. 좌클릭=교차점, 우클릭=삭제, n=다음, q=종료')

    state = {'pts': [], 'sc': args.scale}
    win = 'label (L=add R=del  n=next p=prev u=undo c=clear q=quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def on_mouse(ev, x, y, flags, param):
        sc = state['sc']
        if ev == cv2.EVENT_LBUTTONDOWN:
            state['pts'].append((x/sc, y/sc))
        elif ev == cv2.EVENT_RBUTTONDOWN and state['pts']:
            fx, fy = x/sc, y/sc
            d = [np.hypot(p[0]-fx, p[1]-fy) for p in state['pts']]
            if min(d) < 80:
                state['pts'].pop(int(np.argmin(d)))
    cv2.setMouseCallback(win, on_mouse)

    idx = 0
    while 0 <= idx < len(images):
        img = cv2.imread(images[idx]); H, W = img.shape[:2]
        lbl = os.path.join(LBL_DIR, os.path.basename(images[idx])[:-4]+'.txt')
        state['pts'] = load_label(lbl, W, H)
        sc = args.scale
        while True:
            disp = cv2.resize(img, None, fx=sc, fy=sc)
            b = int(args.box*sc)
            for (cx, cy) in state['pts']:
                dx, dy = int(cx*sc), int(cy*sc)
                cv2.rectangle(disp, (dx-b//2, dy-b//2), (dx+b//2, dy+b//2),
                              (0, 255, 0), 2)
                cv2.circle(disp, (dx, dy), 3, (0, 255, 0), -1)
            tx, ty = int(TARGET[0]*sc), int(TARGET[1]*sc)
            cv2.drawMarker(disp, (tx, ty), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 24, 2)
            cv2.putText(disp, f'[{idx+1}/{len(images)}] {os.path.basename(images[idx])}'
                        f'  pts:{len(state["pts"])}', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(win, disp)
            k = cv2.waitKey(20) & 0xFF
            if k in (ord('n'), ord(' ')):
                save_label(lbl, state['pts'], W, H, args.box); idx += 1; break
            elif k == ord('p'):
                save_label(lbl, state['pts'], W, H, args.box); idx -= 1; break
            elif k == ord('u') and state['pts']:
                state['pts'].pop()
            elif k == ord('c'):
                state['pts'] = []
            elif k == ord('q'):
                save_label(lbl, state['pts'], W, H, args.box)
                cv2.destroyAllWindows()
                n = len(glob.glob(os.path.join(LBL_DIR, '*.txt')))
                print(f'\n라벨 {n}개 저장 → {LBL_DIR}')
                return
    cv2.destroyAllWindows()
    n = len(glob.glob(os.path.join(LBL_DIR, '*.txt')))
    print(f'\n완료. 라벨 {n}개 → {LBL_DIR}')


if __name__ == '__main__':
    main()
