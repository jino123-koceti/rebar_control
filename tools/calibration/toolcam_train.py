#!/usr/bin/env python3
"""툴캠 교차점 YOLO 학습 (우측+좌측 통합).
train/val 재분할 후 학습. 결과는 **별도 파일**로 저장(기존 toolcam_crossing.pt 미덮어쓰기).

  python3 toolcam_train.py                 # 0061~0181(우+좌), best.pt 전이, → toolcam_crossing_both.pt
  python3 toolcam_train.py --range-min 1   # 전체 포함(구뷰까지)
  python3 toolcam_train.py --epochs 150

학습 후 검증되면 수동으로 교체:
  cp .../model/toolcam_crossing_both.pt .../model/toolcam_crossing.pt
"""
import os, glob, shutil, argparse

ROOT = '/home/koceti/ros2_ws/data/toolcam_dataset'
IMG = os.path.join(ROOT, 'images')
LBL = os.path.join(ROOT, 'labels')
MODEL_DIR = '/home/koceti/ros2_ws/src/rebar_vision/model'


def num(path):
    return int(os.path.basename(path)[5:9])


def rebuild_split(range_min, range_max, val_every):
    """images/{train,val}, labels/{train,val} 재구성 (기존 분할 삭제 후)."""
    imgs = sorted(glob.glob(os.path.join(IMG, 'tcam_*.png')))
    imgs = [p for p in imgs if range_min <= num(p) <= range_max
            and os.path.exists(os.path.join(LBL, os.path.basename(p)[:-4] + '.txt'))]
    if not imgs:
        raise RuntimeError('대상 이미지 없음')
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
        lp = os.path.join(LBL, name[:-4] + '.txt')
        shutil.copy(ip, os.path.join(IMG, sub, name))
        shutil.copy(lp, os.path.join(LBL, sub, name[:-4] + '.txt'))
        if sub == 'val':
            nval += 1
        else:
            ntr += 1
    n_left = sum(1 for p in imgs if num(p) >= 121)
    print(f'  분할: train {ntr}, val {nval} '
          f'(범위 {num(imgs[0])}~{num(imgs[-1])}, 좌측 {n_left}장 포함)')
    return ntr, nval


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--range-min', type=int, default=61,
                    help='최소 tcam 번호 (기본 61=구뷰 0001~0060 제외)')
    ap.add_argument('--range-max', type=int, default=9999)
    ap.add_argument('--val-every', type=int, default=5, help='N장마다 1장 val')
    ap.add_argument('--base', default=os.path.join(MODEL_DIR, 'best.pt'),
                    help='전이학습 베이스 모델')
    ap.add_argument('--epochs', type=int, default=120)
    ap.add_argument('--imgsz', type=int, default=960)
    ap.add_argument('--batch', type=int, default=4)
    ap.add_argument('--name', default='toolcam_both', help='runs/ 하위 이름')
    ap.add_argument('--out', default=os.path.join(MODEL_DIR, 'toolcam_crossing_both.pt'),
                    help='결과 저장 경로 (기존 toolcam_crossing.pt 미덮어쓰기)')
    args = ap.parse_args()

    print('=' * 60)
    print(' 툴캠 교차점 통합 학습 (우측+좌측)')
    print('=' * 60)
    rebuild_split(args.range_min, args.range_max, args.val_every)

    from ultralytics import YOLO
    model = YOLO(args.base)
    model.train(
        data=os.path.join(ROOT, 'data.yaml'),
        epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device='0', patience=30, pretrained=True, optimizer='auto', lr0=0.01,
        project=os.path.join(ROOT, 'runs'), name=args.name, exist_ok=True,
    )
    best = os.path.join(ROOT, 'runs', args.name, 'weights', 'best.pt')
    if os.path.exists(best):
        shutil.copy(best, args.out)
        print(f'\n✅ 학습 완료 → {args.out}')
        print(f'   (기존 toolcam_crossing.pt 는 그대로. 검증 후 수동 교체)')
    else:
        print(f'\n⚠️ best.pt 못찾음: {best}')


if __name__ == '__main__':
    main()
