#!/usr/bin/env python3
"""[좌우 스테레오 캘리브 2단계] 캡처한 쌍으로 상대포즈(R,T) 계산.
각 쌍에서 ChArUco 코너 검출 → 공유 코너로 cv2.stereoCalibrate(인트린식 고정).
결과: 우측카메라 기준 좌측카메라의 상대 R,T → 와이드베이스라인 삼각측량용.

사용: python3 stereo_calibrate.py
저장: data/calibration/stereo_extrinsics.yaml
"""
import os, glob
import numpy as np
import cv2, yaml

DIR = '/home/koceti/ros2_ws/data/calibration/stereo_pairs'
OUT = '/home/koceti/ros2_ws/data/calibration/stereo_extrinsics.yaml'
DICT = cv2.aruco.DICT_4X4_50
SQX, SQY, CHK, MRK = 8, 6, 0.030, 0.022  # 30mm 사각형 보드


def main():
    intr = yaml.safe_load(open(f'{DIR}/intrinsics.yaml'))
    K_L = np.array(intr['K_L']); K_R = np.array(intr['K_R'])
    W, H = intr['image_size']
    dist = np.zeros(5)  # rect 이미지라 왜곡 ~0

    d = cv2.aruco.getPredefinedDictionary(DICT)
    board = cv2.aruco.CharucoBoard((SQX, SQY), CHK, MRK, d)
    board.setLegacyPattern(True)
    cdet = cv2.aruco.CharucoDetector(board)
    obj_all = board.getChessboardCorners()  # (N,3) 보드 3D 코너

    pairs = sorted(glob.glob(f'{DIR}/pair_*_L.png'))
    print(f'{len(pairs)}쌍 처리...')
    objp, imgL, imgR = [], [], []
    for pL in pairs:
        pR = pL.replace('_L.png', '_R.png')
        gL = cv2.cvtColor(cv2.imread(pL), cv2.COLOR_BGR2GRAY)
        gR = cv2.cvtColor(cv2.imread(pR), cv2.COLOR_BGR2GRAY)
        cL, idL, _, _ = cdet.detectBoard(gL)
        cR, idR, _, _ = cdet.detectBoard(gR)
        if idL is None or idR is None:
            continue
        idL = idL.flatten(); idR = idR.flatten()
        common = np.intersect1d(idL, idR)
        if len(common) < 6:
            continue
        mapL = {int(i): cL[k][0] for k, i in enumerate(idL)}
        mapR = {int(i): cR[k][0] for k, i in enumerate(idR)}
        op = np.array([obj_all[i] for i in common], np.float32)
        ipL = np.array([mapL[int(i)] for i in common], np.float32)
        ipR = np.array([mapR[int(i)] for i in common], np.float32)
        objp.append(op); imgL.append(ipL); imgR.append(ipR)
        print(f'  {os.path.basename(pL)[:9]}: 공유코너 {len(common)}')
    if len(objp) < 5:
        print(f'유효쌍 부족 ({len(objp)}) — 더 캡처 필요'); return

    flags = cv2.CALIB_FIX_INTRINSIC
    ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        objp, imgL, imgR, K_L, dist, K_R, dist, (W, H),
        flags=flags, criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
    baseline = np.linalg.norm(T) * 1000.0
    print(f'\n=== 스테레오 캘리브 결과 ({len(objp)}쌍) ===')
    print(f'  재투영 RMS: {ret:.3f}px  (<0.5 좋음, <1 양호)')
    print(f'  베이스라인 |T| = {baseline:.0f}mm  (설계 ~500mm와 비교)')
    print(f'  R(우→좌):\n{np.round(R,4)}')
    print(f'  T(mm): {np.round(T.flatten()*1000,1)}')
    yaml.safe_dump({'R_RtoL': R.tolist(), 'T_RtoL_m': T.flatten().tolist(),
                    'baseline_mm': float(baseline), 'rms_px': float(ret),
                    'K_L': K_L.tolist(), 'K_R': K_R.tolist(),
                    'note': 'P_left = R_RtoL @ P_right + T. 우측 카메라 기준.'},
                   open(OUT, 'w'))
    print(f'\n저장: {OUT}')
    if baseline < 300 or baseline > 700:
        print('⚠️ 베이스라인이 500mm와 많이 다름 — 캡처/검출 확인 필요')
    if ret > 1.5:
        print('⚠️ RMS 큼 — 코너검출 품질/쌍 다양성 확인')


if __name__ == '__main__':
    main()
