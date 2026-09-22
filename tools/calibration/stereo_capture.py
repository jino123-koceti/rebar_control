#!/usr/bin/env python3
"""[좌우 스테레오 캘리브 1단계] ChArUco 보드 좌우 동시 캡처.
Enter마다 좌우 최신 프레임을 잡아 ChArUco 검출 → 양쪽 공유코너 충분하면 저장.
보드를 위치·각도·거리 바꿔가며 15~20쌍 모으세요.

사용: python3 stereo_capture.py
저장: data/calibration/stereo_pairs/pair_NN_{L,R}.png + intrinsics.yaml
"""
import os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2, yaml

OUT = '/home/koceti/ros2_ws/data/calibration/stereo_pairs'
TOPICS = {'L': '/zedxmini1/zed_node/left/image_rect_color',
          'R': '/zedxmini2/zed_node/left/image_rect_color'}
INFO = {'L': '/zedxmini1/zed_node/left/camera_info',
        'R': '/zedxmini2/zed_node/left/camera_info'}
DICT = cv2.aruco.DICT_4X4_50
SQX, SQY, CHK, MRK = 8, 6, 0.030, 0.022  # 30mm 사각형 보드
MIN_SHARED = 6


def main():
    os.makedirs(OUT, exist_ok=True)
    rclpy.init(); n = Node('stereo_cap'); br = CvBridge()
    buf = {'L': [], 'R': []}; K = {'L': None, 'R': None}
    for k in ('L', 'R'):
        n.create_subscription(Image, TOPICS[k],
            lambda m, kk=k: buf[kk].append(br.imgmsg_to_cv2(m, 'bgr8')),
            qos_profile_sensor_data)
        n.create_subscription(CameraInfo, INFO[k],
            lambda m, kk=k: K.__setitem__(kk, np.array(m.k).reshape(3, 3)),
            qos_profile_sensor_data)
    d = cv2.aruco.getPredefinedDictionary(DICT)
    board = cv2.aruco.CharucoBoard((SQX, SQY), CHK, MRK, d)
    board.setLegacyPattern(True)
    cdet = cv2.aruco.CharucoDetector(board)

    def spin(sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(n, timeout_sec=0.02)

    def detect(img):
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cc, cid, _, _ = cdet.detectBoard(g)
        return cid.flatten() if cid is not None else np.array([], int)

    spin(2.0)
    # 인트린식 저장
    if K['L'] is not None and K['R'] is not None:
        yaml.safe_dump({'K_L': K['L'].tolist(), 'K_R': K['R'].tolist(),
                        'image_size': [960, 600]},
                       open(f'{OUT}/intrinsics.yaml', 'w'))
        print(f'인트린식 저장: {OUT}/intrinsics.yaml')
    # 기존 쌍 개수
    cnt = len([f for f in os.listdir(OUT) if f.endswith('_L.png')])
    print('=' * 60)
    print(f' 스테레오 ChArUco 라이브 캡처 (현재 {cnt}쌍)')
    print(' 보드를 천천히 움직이며 "공유"가 8↑ 되는 위치 찾기 → 자동저장')
    print(' Ctrl-C로 종료.  목표: 위치·각도 다양하게 15~20쌍')
    print('=' * 60)
    last_save = 0.0
    try:
        while True:
            buf['L'].clear(); buf['R'].clear()
            spin(0.35)
            if not buf['L'] or not buf['R']:
                print('\r  프레임 대기...', end='', flush=True); continue
            iL, iR = buf['L'][-1], buf['R'][-1]
            idL, idR = detect(iL), detect(iR)
            shared = len(set(idL.tolist()) & set(idR.tolist()))
            bar = '█' * shared
            now = time.time()
            status = ''
            if shared >= MIN_SHARED and now - last_save > 1.5:
                cv2.imwrite(f'{OUT}/pair_{cnt:02d}_L.png', iL)
                cv2.imwrite(f'{OUT}/pair_{cnt:02d}_R.png', iR)
                cnt += 1; last_save = now
                status = f'  → 저장✅ {cnt}쌍'
            print(f'\r  L{len(idL):2d} R{len(idR):2d} 공유{shared:2d} {bar:<12}'
                  f'{status}      ', end='', flush=True)
    except (KeyboardInterrupt, EOFError):
        pass
    print(f'\n\n총 {cnt}쌍. 다음: python3 stereo_calibrate.py')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
