#!/usr/bin/env python3
"""
YOLO 형식 라벨 데이터셋을 Roboflow(Object Detection) 프로젝트에 업로드

이미지 + YOLO txt 라벨을 함께 올린다. train/val 분할과 클래스명(data.yaml)을 유지.
orbbec 교차점 검출 데이터셋(data/orbbec_crossing_dataset) 업로드용으로 작성.

전제: 대상 Roboflow 프로젝트가 Object Detection 타입이어야 함.

구조(YOLO 표준):
  <dataset>/images/{train,val}/*.png
  <dataset>/labels/{train,val}/*.txt   (같은 stem)
  <dataset>/data.yaml  (names 로 라벨맵 구성)

사용:
  export ROBOFLOW_API_KEY=xxxx
  python3 tools/vision_test/upload_yolo_dataset_to_roboflow.py \
      --workspace test-mxxrn --project <검출프로젝트slug> \
      --dataset data/orbbec_crossing_dataset

옵션: --stride N, --dry-run, --list, --batch NAME
"""

import argparse
import glob
import os
import sys

# YOLO split 폴더명 → Roboflow split 이름
SPLIT_MAP = {"train": "train", "val": "valid", "valid": "valid", "test": "test"}


def load_names(dataset):
    """data.yaml의 names → {idx: name} 라벨맵."""
    ypath = os.path.join(dataset, "data.yaml")
    names = {}
    if os.path.exists(ypath):
        import re
        txt = open(ypath).read()
        m = re.search(r"names:\s*\[([^\]]*)\]", txt)
        if m:
            items = [s.strip().strip("'\"") for s in m.group(1).split(",") if s.strip()]
            names = {i: n for i, n in enumerate(items)}
    return names


def collect(dataset, stride, since_index=None):
    """(image_path, label_path, split) 쌍 리스트.

    since_index: ocross_NNNN 의 NNNN 이 이 값 이상인 것만. 데크플레이트분을
    빼고 RC 수집분만 올릴 때 사용 (RC는 0168부터).
    """
    import re
    pairs = []
    for yolo_split in ("train", "val", "valid", "test"):
        img_dir = os.path.join(dataset, "images", yolo_split)
        lbl_dir = os.path.join(dataset, "labels", yolo_split)
        if not os.path.isdir(img_dir):
            continue
        imgs = sorted(glob.glob(os.path.join(img_dir, "*.png")) +
                      glob.glob(os.path.join(img_dir, "*.jpg")))
        if since_index is not None:
            def idx(p):
                m = re.search(r"(\d+)", os.path.basename(p))
                return int(m.group(1)) if m else -1
            imgs = [p for p in imgs if idx(p) >= since_index]
        if stride > 1:
            imgs = imgs[::stride]
        rf_split = SPLIT_MAP[yolo_split]
        for ip in imgs:
            stem = os.path.splitext(os.path.basename(ip))[0]
            lp = os.path.join(lbl_dir, stem + ".txt")
            pairs.append((ip, lp if os.path.exists(lp) else None, rf_split))
    return pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--api-key", default=os.environ.get("ROBOFLOW_API_KEY"))
    ap.add_argument("--workspace", default=None)
    ap.add_argument("--project", required=True)
    ap.add_argument("--dataset", default="data/orbbec_crossing_dataset")
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--since-index", type=int, default=None,
                    help="파일명 번호가 이 값 이상인 것만 (RC 수집분만: 168)")
    ap.add_argument("--batch", default="orbbec_crossing")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        if not args.api_key:
            print("ERROR: API 키 필요"); sys.exit(1)
        from roboflow import Roboflow
        rf = Roboflow(api_key=args.api_key)
        ws = rf.workspace(args.workspace) if args.workspace else rf.workspace()
        print("워크스페이스:", getattr(ws, "url", "?"))
        try:
            for p in ws.projects():
                print("  -", p.split("/")[-1] if isinstance(p, str) else p)
        except Exception as e:
            print("프로젝트 조회 실패:", e)
        return

    names = load_names(args.dataset)
    pairs = collect(args.dataset, args.stride, args.since_index)
    n_lbl = sum(1 for _, l, _ in pairs if l)
    from collections import Counter
    by_split = Counter(s for _, _, s in pairs)
    print(f"데이터셋: {args.dataset}")
    print(f"라벨맵: {names}")
    print(f"대상: {len(pairs)}장 (라벨있음 {n_lbl}), split별 {dict(by_split)}, stride={args.stride}")

    if args.dry_run:
        print("[dry-run] 업로드 안 함."); return
    if not args.api_key:
        print("ERROR: API 키 필요"); sys.exit(1)
    if not names:
        print("경고: data.yaml에서 라벨맵을 못 읽음 → 클래스명 매핑 없이 업로드됨")

    from roboflow import Roboflow
    rf = Roboflow(api_key=args.api_key)
    ws = rf.workspace(args.workspace) if args.workspace else rf.workspace()
    project = ws.project(args.project)

    ok = fail = 0
    for ip, lp, split in pairs:
        try:
            project.single_upload(
                image_path=ip,
                annotation_path=lp,
                annotation_labelmap=names if (lp and names) else None,
                split=split,
                batch_name=args.batch,
                num_retry_uploads=3,
            )
            ok += 1
        except Exception as e:
            fail += 1
            print(f"  실패 {os.path.basename(ip)}: {e}")
        if (ok + fail) % 25 == 0:
            print(f"  진행 {ok+fail}/{len(pairs)} (성공 {ok}, 실패 {fail})")
    print(f"완료: 성공 {ok}, 실패 {fail} / 총 {len(pairs)}")


if __name__ == "__main__":
    main()
