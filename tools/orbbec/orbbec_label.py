#!/usr/bin/env python3
"""Orbbec 교차점 클릭 라벨러 → YOLO 포맷 저장.
좌클릭=교차점 추가(고정박스), 우클릭=최근접 삭제.

키:
  좌클릭   교차점 추가       우클릭  가까운 점 삭제
  n/SPACE  저장+다음         p       이전
  u        마지막 취소       c       전부 지우기
  +/-      박스 크기 조정    [ / ]   표시 배율
  q        저장+종료

저장: data/orbbec_crossing_dataset/labels/ocross_NNNN.txt (YOLO: class cx cy w h 정규화)
"""
import os, glob, argparse
import cv2
import numpy as np

ROOT = '/home/koceti/ros2_ws/data/orbbec_crossing_dataset'
IMG_DIR = os.path.join(ROOT, 'images')
LBL_DIR = os.path.join(ROOT, 'labels')


def load_label(path, W, H):
    pts = []
    if os.path.exists(path):
        for line in open(path):
            p = line.split()
            if len(p) >= 5:
                pts.append((float(p[1])*W, float(p[2])*H))
    return pts


def save_label(path, pts, W, H, box):
    with open(path, 'w') as f:
        for (cx, cy) in pts:
            f.write(f"0 {cx/W:.6f} {cy/H:.6f} {box/W:.6f} {box/H:.6f}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--box', type=int, default=60, help='박스 크기(px)')
    ap.add_argument('--scale', type=float, default=0.8)
    args = ap.parse_args()
    os.makedirs(LBL_DIR, exist_ok=True)
    images = sorted(glob.glob(os.path.join(IMG_DIR, 'ocross_*.png')))
    if not images:
        print(f'이미지 없음: {IMG_DIR}'); return
    print(f'{len(images)}장 라벨링. 좌=추가 우=삭제 n=다음 p=이전 +/-=박스 q=종료')

    st = {'pts': [], 'sc': args.scale, 'box': args.box}
    win = 'orbbec label (L=add R=del  n/p  u undo  c clear  +/- box  q quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def on_mouse(ev, x, y, flags, param):
        sc = st['sc']
        if ev == cv2.EVENT_LBUTTONDOWN:
            st['pts'].append((x/sc, y/sc))
        elif ev == cv2.EVENT_RBUTTONDOWN and st['pts']:
            fx, fy = x/sc, y/sc
            d = [np.hypot(p[0]-fx, p[1]-fy) for p in st['pts']]
            if min(d) < 60:
                st['pts'].pop(int(np.argmin(d)))
    cv2.setMouseCallback(win, on_mouse)

    idx = 0
    while 0 <= idx < len(images):
        img = cv2.imread(images[idx]); H, W = img.shape[:2]
        lbl = os.path.join(LBL_DIR, os.path.basename(images[idx])[:-4]+'.txt')
        st['pts'] = load_label(lbl, W, H)
        while True:
            sc = st['sc']; b = int(st['box']*sc)
            disp = cv2.resize(img, None, fx=sc, fy=sc)
            for (cx, cy) in st['pts']:
                dx, dy = int(cx*sc), int(cy*sc)
                cv2.rectangle(disp, (dx-b//2, dy-b//2), (dx+b//2, dy+b//2), (0, 255, 0), 2)
                cv2.circle(disp, (dx, dy), 2, (0, 0, 255), -1)
            cv2.putText(disp, f'[{idx+1}/{len(images)}] {os.path.basename(images[idx])}'
                        f'  pts:{len(st["pts"])}  box:{st["box"]}', (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(win, disp)
            k = cv2.waitKey(20) & 0xFF
            if k in (ord('n'), ord(' ')):
                save_label(lbl, st['pts'], W, H, st['box']); idx += 1; break
            elif k == ord('p'):
                save_label(lbl, st['pts'], W, H, st['box']); idx = max(0, idx-1); break
            elif k == ord('u') and st['pts']:
                st['pts'].pop()
            elif k == ord('c'):
                st['pts'] = []
            elif k in (ord('+'), ord('=')):
                st['box'] += 6
            elif k == ord('-'):
                st['box'] = max(12, st['box']-6)
            elif k == ord(']'):
                st['sc'] = min(1.5, st['sc']+0.1)
            elif k == ord('['):
                st['sc'] = max(0.3, st['sc']-0.1)
            elif k == ord('q'):
                save_label(lbl, st['pts'], W, H, st['box'])
                cv2.destroyAllWindows()
                n = len(glob.glob(os.path.join(LBL_DIR, '*.txt')))
                print(f'\n라벨 {n}개 저장 → {LBL_DIR}')
                return
    cv2.destroyAllWindows()
    n = len(glob.glob(os.path.join(LBL_DIR, '*.txt')))
    print(f'\n완료. 라벨 {n}개 → {LBL_DIR}')


if __name__ == '__main__':
    main()
