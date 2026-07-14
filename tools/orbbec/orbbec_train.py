#!/usr/bin/env python3
"""Orbbec 교차점 YOLO 학습.
train/val 재분할 → 학습 → src/rebar_vision/model/orbbec_crossing.pt 저장.

  python3 orbbec_train.py                 # yolov8n 전이, imgsz1280, → orbbec_crossing.pt
  python3 orbbec_train.py --epochs 200 --imgsz 1280
"""
import os, glob, shutil, argparse

ROOT = '/home/koceti/ros2_ws/data/orbbec_crossing_dataset'
IMG = os.path.join(ROOT, 'images')
LBL = os.path.join(ROOT, 'labels')
MODEL_DIR = '/home/koceti/ros2_ws/src/rebar_vision/model'


def rebuild_split(val_every):
    imgs = sorted(glob.glob(os.path.join(IMG, 'ocross_*.png')))
    imgs = [p for p in imgs
            if os.path.exists(os.path.join(LBL, os.path.basename(p)[:-4]+'.txt'))]
    for sub in ('train', 'val'):
        for base in (IMG, LBL):
            d = os.path.join(base, sub)
            if os.path.isdir(d):
                shutil.rmtree(d)
            os.makedirs(d)
    ntr = nval = 0
    for i, ip in enumerate(imgs):
        sub = 'val' if (i % val_every == 0) else 'train'
        name = os.path.basename(ip)
        shutil.copy(ip, os.path.join(IMG, sub, name))
        shutil.copy(os.path.join(LBL, name[:-4]+'.txt'),
                    os.path.join(LBL, sub, name[:-4]+'.txt'))
        nval += sub == 'val'; ntr += sub == 'train'
    print(f'  분할: train {ntr}, val {nval}')
    return ntr, nval


def write_yaml():
    p = os.path.join(ROOT, 'data.yaml')
    with open(p, 'w') as f:
        f.write(f"path: {ROOT}\ntrain: images/train\nval: images/val\n"
                "nc: 1\nnames: ['crossing']\n")
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--val-every', type=int, default=5)
    ap.add_argument('--base', default='yolov8n.pt', help='전이 베이스 (없으면 자동 다운로드)')
    ap.add_argument('--epochs', type=int, default=150)
    ap.add_argument('--imgsz', type=int, default=1280)
    ap.add_argument('--batch', type=int, default=4)
    ap.add_argument('--name', default='orbbec_crossing')
    ap.add_argument('--out', default=os.path.join(MODEL_DIR, 'orbbec_crossing.pt'))
    args = ap.parse_args()

    print('=' * 60); print(' Orbbec 교차점 YOLO 학습'); print('=' * 60)
    rebuild_split(args.val_every)
    data = write_yaml()

    from ultralytics import YOLO
    model = YOLO(args.base)
    model.train(
        data=data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device='0', patience=40, pretrained=True, optimizer='auto', lr0=0.01,
        project=os.path.join(ROOT, 'runs'), name=args.name, exist_ok=True,
    )
    best = os.path.join(ROOT, 'runs', args.name, 'weights', 'best.pt')
    if os.path.exists(best):
        shutil.copy(best, args.out)
        print(f'\n✅ 학습 완료 → {args.out}')
    else:
        print(f'\n⚠️ best.pt 못찾음: {best}')


if __name__ == '__main__':
    main()
