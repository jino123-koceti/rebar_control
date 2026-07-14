#!/usr/bin/env python3
"""Orbbec 호모그래피 적합: calib3d_data_orbbec.json(픽셀↔로봇XY 쌍) → homography_orbbec.yaml.

자세별(r/l)로 cv2.findHomography RANSAC(기본 5mm)를 써서 이상치 측정쌍을 제거하고,
inlier에 대해 leave-one-out(LOO) 잔차를 계산해 정확도를 리포트한다.
변환식: robot_XY = H @ [u, v, 1] (mm).

사전: calibrate_3d_orbbec.py로 픽셀↔로봇 touch 쌍을 (재)수집해 calib3d_data_orbbec.json 갱신.
  python3 tools/calibration/fit_homography_orbbec.py                 # 기본 경로
  python3 tools/calibration/fit_homography_orbbec.py --ransac 5.0    # RANSAC 임계(mm) 조정
  python3 tools/calibration/fit_homography_orbbec.py --dry           # 파일 안 쓰고 결과만 출력
"""
import os
import json
import argparse
import numpy as np
import cv2
import yaml

ROOT = '/home/koceti/ros2_ws'
DATA = os.path.join(ROOT, 'data/calibration/calib3d_data_orbbec.json')
OUT = os.path.join(ROOT, 'data/calibration/homography_orbbec.yaml')


def _fit(src, dst, ransac):
    """src(N,2 픽셀 u,v) → dst(N,2 로봇 x,y mm) 호모그래피 + inlier mask."""
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, ransac)
    inl = mask.ravel().astype(bool) if mask is not None else np.ones(len(src), bool)
    return H, inl


def _loo(src_in, dst_in, ransac):
    """inlier들에 대해 하나씩 빼고 재적합 → 남긴 점 예측오차(mm) 리스트."""
    errs = []
    n = len(src_in)
    if n < 5:  # LOO에 최소 4쌍 필요(호모그래피) → 5쌍 이상일 때만 의미
        return errs
    for i in range(n):
        keep = [j for j in range(n) if j != i]
        Hi, _ = cv2.findHomography(src_in[keep], dst_in[keep], 0)  # inlier만이라 최소자승
        if Hi is None:
            continue
        u, v = src_in[i]
        p = Hi @ np.array([u, v, 1.0])
        pred = p[:2] / p[2]
        errs.append(float(np.linalg.norm(pred - dst_in[i])))
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default=DATA)
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--ransac', type=float, default=5.0, help='RANSAC reproj 임계 (mm)')
    ap.add_argument('--dry', action='store_true', help='파일 안 쓰고 결과만 출력')
    args = ap.parse_args()

    pairs = json.load(open(args.data))
    out = {'note': 'robot_XY=H@[u,v,1] (mm). RANSAC inlier 호모그래피. 이상치 제외.'}

    for pose, key in (('r', 'right'), ('l', 'left')):
        pp = [p for p in pairs if p.get('pose') == pose]
        if len(pp) < 4:
            print(f'[{key}] 쌍 부족: {len(pp)} (>=4 필요) — 스킵')
            continue
        src = np.array([p['pixel'] for p in pp], float)       # u,v
        dst = np.array([p['ptool'][:2] for p in pp], float)   # 로봇 x,y mm
        H, inl = _fit(src, dst, args.ransac)
        if H is None:
            print(f'[{key}] findHomography 실패 — 스킵')
            continue
        errs = _loo(src[inl], dst[inl], args.ransac)
        rec = {
            'H': H.tolist(),
            'loo_xy_mm': float(np.mean(errs)) if errs else -1.0,
            'loo_max_mm': float(np.max(errs)) if errs else -1.0,
            'num_pairs': int(inl.sum()),
            'total_pairs': len(pp),
        }
        out[key] = rec
        print(f'[{key}] inlier {rec["num_pairs"]}/{rec["total_pairs"]}, '
              f'LOO 평균 {rec["loo_xy_mm"]:.2f}mm / 최대 {rec["loo_max_mm"]:.2f}mm')

    if args.dry:
        print('--dry: 파일 저장 안 함')
        return
    yaml.safe_dump(out, open(args.out, 'w'), allow_unicode=True, sort_keys=True)
    print(f'저장: {args.out}')


if __name__ == '__main__':
    main()
