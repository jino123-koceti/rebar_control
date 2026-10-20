"""좌우 사이의 회전만 다시 푼다.

내부 파라미터(초점거리·주점·왜곡)는 공장값을 그대로 쓴다 — 리그가 틀어진
것과 무관하게 멀쩡하다. 좌우 **상대 회전**만 고쳐서 보정 후 영상에서
세로 어긋남이 0 이 되게 한다.

yaw(Y축 회전)는 건드리지 않는다. yaw 는 가로 시차 = 거리값을 좌우하는데,
세로 어긋남으로는 yaw 를 관측할 수 없다. 관측되지 않는 것을 움직이면
거리만 망가진다. 그래서 pitch(X)·roll(Z) 두 개만 푼다.
"""
import sys, json, numpy as np, cv2

SN = 43764735
W, H = 1920, 1080


def grab_pairs(n=6):
    import pyzed.sl as sl
    cam = sl.Camera(); ip = sl.InitParameters()
    ip.set_from_serial_number(SN)
    ip.camera_resolution = sl.RESOLUTION.HD1080
    ip.camera_fps = 15
    ip.depth_mode = sl.DEPTH_MODE.NONE
    ip.camera_disable_self_calib = True
    if cam.open(ip) != sl.ERROR_CODE.SUCCESS:
        sys.exit('카메라를 열 수 없다')
    L, R = sl.Mat(), sl.Mat()
    for _ in range(12):
        cam.grab()
    pairs = []
    for _ in range(n):
        for _ in range(3):
            cam.grab()
        cam.retrieve_image(L, sl.VIEW.LEFT_UNRECTIFIED)
        cam.retrieve_image(R, sl.VIEW.RIGHT_UNRECTIFIED)
        pairs.append((cv2.cvtColor(L.get_data()[:, :, :3], cv2.COLOR_BGR2GRAY),
                      cv2.cvtColor(R.get_data()[:, :, :3], cv2.COLOR_BGR2GRAY)))
    c = cam.get_camera_information().camera_configuration.calibration_parameters_raw
    tf = c.stereo_transform
    M = np.array([[tf[i, j] for j in range(4)] for i in range(4)])
    calib = dict(
        K1=np.array([[c.left_cam.fx, 0, c.left_cam.cx],
                     [0, c.left_cam.fy, c.left_cam.cy], [0, 0, 1]]),
        K2=np.array([[c.right_cam.fx, 0, c.right_cam.cx],
                     [0, c.right_cam.fy, c.right_cam.cy], [0, 0, 1]]),
        d1=np.array(list(c.left_cam.disto)[:8]),
        d2=np.array(list(c.right_cam.disto)[:8]),
        R0=M[:3, :3], T=M[:3, 3])
    cam.close()
    return pairs, calib


def match(l, r):
    orb = cv2.ORB_create(8000)
    k1, d1 = orb.detectAndCompute(l, None)
    k2, d2 = orb.detectAndCompute(r, None)
    if d1 is None or d2 is None:
        return np.empty((0, 2)), np.empty((0, 2))
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    good = [m for m, n in bf.knnMatch(d1, d2, k=2) if m.distance < 0.75 * n.distance]
    p1 = np.float32([k1[m.queryIdx].pt for m in good])
    p2 = np.float32([k2[m.trainIdx].pt for m in good])
    return p1, p2


def rectified_dy(delta, cal, p1, p2):
    """보정 회전 delta(pitch, roll)를 넣었을 때 각 매칭점의 세로 어긋남."""
    rvec = np.array([delta[0], 0.0, delta[1]])
    dR, _ = cv2.Rodrigues(rvec)
    R = dR @ cal['R0']
    R1, R2, P1, P2, *_ = cv2.stereoRectify(
        cal['K1'], cal['d1'], cal['K2'], cal['d2'], (W, H), R, cal['T'],
        flags=0, alpha=0)
    q1 = cv2.undistortPoints(p1.reshape(-1, 1, 2), cal['K1'], cal['d1'], R=R1, P=P1)
    q2 = cv2.undistortPoints(p2.reshape(-1, 1, 2), cal['K2'], cal['d2'], R=R2, P=P2)
    return q1.reshape(-1, 2), q2.reshape(-1, 2)


def main():
    pairs, cal = grab_pairs()
    P1, P2 = [], []
    for l, r in pairs:
        a, b = match(l, r)
        P1.append(a); P2.append(b)
    p1 = np.vstack(P1); p2 = np.vstack(P2)
    print(f'원시 매칭 {len(p1)}점 ({len(pairs)}장)')

    # 공장값 기준으로 한 번 정합해서 오매칭을 걸러낸다
    q1, q2 = rectified_dy([0.0, 0.0], cal, p1, p2)
    dx = q1[:, 0] - q2[:, 0]
    dy = q1[:, 1] - q2[:, 1]
    med = np.median(dy)
    keep = (dx > 1) & (dx < 400) & (np.abs(dy - med) < 8)
    p1, p2 = p1[keep], p2[keep]
    print(f'유효 매칭 {len(p1)}점 · 공장값에서의 세로 어긋남 중앙값 {med:+.2f} px')

    def resid(delta):
        a, b = rectified_dy(delta, cal, p1, p2)
        return a[:, 1] - b[:, 1]

    try:
        from scipy.optimize import least_squares
        sol = least_squares(resid, [0.0, 0.0], loss='soft_l1', f_scale=1.0,
                            diff_step=1e-4)
        delta = sol.x
        print(f'scipy 최적화 완료 (비용 {sol.cost:.1f})')
    except ImportError:
        # scipy 가 없으면 격자 탐색 + 세분화
        delta = np.array([0.0, 0.0])
        span = np.array([0.05, 0.02])
        for _ in range(7):
            best, bd = None, None
            for a in np.linspace(delta[0] - span[0], delta[0] + span[0], 15):
                for c in np.linspace(delta[1] - span[1], delta[1] + span[1], 15):
                    v = np.median(np.abs(resid([a, c])))
                    if best is None or v < best:
                        best, bd = v, np.array([a, c])
            delta, span = bd, span / 3
        print('격자 탐색 완료')

    for tag, dlt in (('공장값', [0.0, 0.0]), ('보정 후', delta)):
        a, b = rectified_dy(dlt, cal, p1, p2)
        e = a[:, 1] - b[:, 1]
        print(f'{tag:>8s}  세로 어긋남 중앙값 {np.median(e):+7.2f} px · '
              f'표준편차 {e.std():5.2f} · |dy|>1px {100*(np.abs(e)>1).mean():5.1f}%')

    print(f'\n보정 회전  pitch {delta[0]:+.6f} rad ({np.degrees(delta[0]):+.3f}°) · '
          f'roll {delta[1]:+.6f} rad ({np.degrees(delta[1]):+.3f}°)')
    rvec = np.array([delta[0], 0.0, delta[1]])
    dR, _ = cv2.Rodrigues(rvec)
    R = dR @ cal['R0']
    json.dump(dict(sn=SN, delta=list(map(float, delta)),
                   R=R.tolist(), T=cal['T'].tolist(),
                   K1=cal['K1'].tolist(), K2=cal['K2'].tolist(),
                   d1=cal['d1'].tolist(), d2=cal['d2'].tolist()),
              open('extrinsic_fix_43764735.json', 'w'), indent=1)
    print('저장: extrinsic_fix_43764735.json')


main()
