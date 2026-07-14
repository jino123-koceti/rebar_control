#!/usr/bin/env python3
"""
주행 프레임을 Roboflow 프로젝트에 업로드

전방(drive_20260714_115833) + 후방(drive_20260714_125006) 주행 프레임을
하나의 Roboflow 프로젝트에 올린다. 이후 웹에서 rebar_h/rebar_v/floor 라벨링.

준비:
  pip install roboflow   (이미 설치됨)
  Roboflow → Settings → API Key,  Workspace/Project 확인

사용:
  export ROBOFLOW_API_KEY=xxxxxxxx
  python3 tools/vision_test/upload_to_roboflow.py \
      --workspace <워크스페이스> --project <프로젝트ID>

옵션:
  --stride N     N장마다 1장만 업로드 (연속프레임 중복 감소, 기본 1=전부)
  --dirs ...     업로드할 폴더 (기본: 전방+후방 주행 폴더)
  --batch NAME   Roboflow 배치 이름 (기본: drive_frames)
  --dry-run      실제 업로드 없이 대상만 출력
"""

import argparse
import glob
import os
import sys


DEFAULT_DIRS = [
    "data/vision_test/drive_20260714_115833",   # 전방
    "data/vision_test/drive_20260714_125006",   # 후방
]


def collect(dirs, stride):
    files = []
    for d in dirs:
        fs = sorted(glob.glob(os.path.join(d, "frame_*.jpg")))
        picked = fs[::stride] if stride > 1 else fs
        files.append((d, picked, len(fs)))
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--api-key", default=os.environ.get("ROBOFLOW_API_KEY"))
    ap.add_argument("--workspace", default=None,
                    help="생략 시 API키의 기본 워크스페이스 사용")
    ap.add_argument("--project", required=True)
    ap.add_argument("--dirs", nargs="+", default=DEFAULT_DIRS)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--batch", default="drive_frames")
    ap.add_argument("--split", default="train", choices=["train", "valid", "test"])
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--list", action="store_true",
                    help="API키가 접근 가능한 워크스페이스/프로젝트 ID 나열")
    args = ap.parse_args()

    if args.list:
        if not args.api_key:
            print("ERROR: --api-key 또는 ROBOFLOW_API_KEY 필요"); sys.exit(1)
        from roboflow import Roboflow
        rf = Roboflow(api_key=args.api_key)
        ws = rf.workspace(args.workspace) if args.workspace else rf.workspace()
        print(f"워크스페이스 URL/slug: {getattr(ws, 'url', '?')}")
        try:
            projs = ws.projects()
            print("프로젝트 목록 (이 ID를 --project 에 그대로 사용):")
            for p in projs:
                pid = p.split("/")[-1] if isinstance(p, str) else p
                print(f"  - {pid}")
        except Exception as e:
            print(f"프로젝트 목록 조회 실패: {e}")
        return

    groups = collect(args.dirs, args.stride)
    total = sum(len(p) for _, p, _ in groups)
    print(f"업로드 대상 (stride={args.stride}):")
    for d, picked, n in groups:
        tag = "전방" if "115833" in d else ("후방" if "125006" in d else "")
        print(f"  {tag} {d}: {len(picked)}/{n}장")
    print(f"  합계: {total}장 → project '{args.project}' (batch={args.batch})")

    if args.dry_run:
        print("[dry-run] 실제 업로드 안 함.")
        return
    if not args.api_key:
        print("ERROR: API 키 없음. --api-key 또는 ROBOFLOW_API_KEY 환경변수 설정.")
        sys.exit(1)

    from roboflow import Roboflow
    rf = Roboflow(api_key=args.api_key)
    ws = rf.workspace(args.workspace) if args.workspace else rf.workspace()
    project = ws.project(args.project)

    ok, fail = 0, 0
    for d, picked, _ in groups:
        # 폴더명을 파일명 접두어로 붙여 전/후방 구분 (라벨엔 영향 없음)
        prefix = os.path.basename(d)
        for i, f in enumerate(picked, 1):
            try:
                project.upload(
                    image_path=f,
                    batch_name=args.batch,
                    split=args.split,
                    tag_names=[prefix],
                    num_retry_uploads=3,
                )
                ok += 1
            except Exception as e:
                fail += 1
                print(f"  실패 {f}: {e}")
            if (ok + fail) % 25 == 0:
                print(f"  진행 {ok+fail}/{total} (성공 {ok}, 실패 {fail})")
    print(f"완료: 성공 {ok}, 실패 {fail} / 총 {total}")


if __name__ == "__main__":
    main()
