#!/usr/bin/env python3
"""
AprilTag 월드 좌표 캘리브레이션 (v2)

EEF에 부착된 기준 태그(ID10)와 대상 태그(ID0)를 동시에 검출하여,
대상 태그의 3D 월드 좌표를 자동 산출합니다.

원리 (v2 - 카메라→월드 회전행렬 직접 산출):
  Phase 1: EEF를 X, Y, Z 방향으로 이동시켜 t_ref 변화량 관찰
           → 카메라→월드 회전행렬 (R_cam2world) 산출
  Phase 2: R_cam2world로 (t_target - t_ref)를 월드 좌표로 변환
           → 대상 태그의 월드 좌표 산출

  태그가 EEF에 어떤 방향으로 붙어있든 무관!

사용법:
  python3 calibrate_tag_position.py --camera right
"""

import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import yaml
import json
from datetime import datetime

WS_DIR = '/home/koceti/ros2_ws'
DATA_DIR = os.path.join(WS_DIR, 'data', 'calibration')

CAMERA_CONFIG = {
    'right': {
        'image_topic': '/zedxmini2/zed_node/left/image_rect_color',
        'info_topic': '/zedxmini2/zed_node/left/camera_info',
        'camera_name': 'Right Camera (zedxmini2)',
    },
    'left': {
        'image_topic': '/zedxmini1/zed_node/left/image_rect_color',
        'info_topic': '/zedxmini1/zed_node/left/camera_info',
        'camera_name': 'Left Camera (zedxmini1)',
    },
}


class TagCalibrationNode(Node):
    def __init__(self, camera_side='right', ref_tag_id=10, target_tag_id=0,
                 tag_size=0.05):
        super().__init__('tag_calibration')
        self.bridge = CvBridge()
        self.camera_image = None
        self.got_image = False

        cfg = CAMERA_CONFIG[camera_side]
        self.camera_name = cfg['camera_name']
        self.camera_side = camera_side

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        self.image_sub = self.create_subscription(
            Image, cfg['image_topic'], self.image_callback, 1)
        self.info_sub = self.create_subscription(
            CameraInfo, cfg['info_topic'], self.camera_info_callback, 1)

        # AprilTag 검출기
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16H5)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)

        self.tag_size = tag_size
        s = tag_size / 2
        self.obj_points = np.array([
            [-s,  s, 0], [ s,  s, 0],
            [ s, -s, 0], [-s, -s, 0],
        ], dtype=np.float64)

        self.ref_tag_id = ref_tag_id
        self.target_tag_id = target_tag_id

        self.get_logger().info(f'[{self.camera_name}] Tag Calibration v2')
        self.get_logger().info(f'  Ref: ID{ref_tag_id}, Target: ID{target_tag_id}')

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info(
                f"Camera intrinsics: fx={self.camera_matrix[0,0]:.1f}")

    def image_callback(self, msg):
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.got_image = True

    def capture_image(self):
        self.got_image = False
        timeout = time.monotonic() + 5.0
        while not self.got_image and time.monotonic() < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.camera_image

    def wait_for_camera_info(self):
        print("  camera_info 대기 중...")
        timeout = time.monotonic() + 10.0
        while not self.camera_info_received and time.monotonic() < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        if not self.camera_info_received:
            print("  [WARN] camera_info 미수신, 기본값 사용")
            self.camera_matrix = np.array([
                [360.5, 0, 640], [0, 360.5, 360], [0, 0, 1]
            ], dtype=np.float64)
            self.dist_coeffs = np.zeros(5, dtype=np.float64)
        else:
            print(f"  camera_info OK: fx={self.camera_matrix[0,0]:.1f}")

    def detect_both_tags(self):
        """두 태그 동시 검출 + pose 추정"""
        image = self.capture_image()
        if image is None:
            return None, None, None, None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        ref_pose = None
        target_pose = None

        if ids is None:
            return image, None, None, None

        for i, tag_id in enumerate(ids.flatten()):
            corner = corners[i][0]
            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, corner,
                self.camera_matrix, self.dist_coeffs
            )
            if not success:
                continue

            rmat, _ = cv2.Rodrigues(rvec)
            pose = {
                'rvec': rvec,
                'tvec': tvec.flatten(),
                'rmat': rmat,
                'center': np.mean(corner, axis=0),
                'corners': corner,
            }

            if tag_id == self.ref_tag_id:
                ref_pose = pose
            elif tag_id == self.target_tag_id:
                target_pose = pose

        detected = list(ids.flatten()) if ids is not None else []
        return image, ref_pose, target_pose, detected

    def capture_multi_frames(self, n_frames=5):
        """여러 프레임 캡처하여 평균 tvec 반환"""
        ref_tvecs = []
        target_tvecs = []

        for f in range(n_frames):
            image, ref_pose, target_pose, detected = self.detect_both_tags()

            if ref_pose is None:
                print(f'    프레임 {f+1}: 기준 태그 미검출')
                continue
            if target_pose is None:
                print(f'    프레임 {f+1}: 대상 태그 미검출')
                continue

            ref_tvecs.append(ref_pose['tvec'])
            target_tvecs.append(target_pose['tvec'])

            ref_d = np.linalg.norm(ref_pose['tvec'])
            tgt_d = np.linalg.norm(target_pose['tvec'])
            print(f'    프레임 {f+1}: ref_d={ref_d:.3f}m, tgt_d={tgt_d:.3f}m ✓')
            time.sleep(0.1)

        if not ref_tvecs:
            return None, None, image

        avg_ref = np.mean(ref_tvecs, axis=0)
        avg_target = np.mean(target_tvecs, axis=0)
        return avg_ref, avg_target, image


def compute_R_cam2world(t_ref_P0, t_ref_P1, t_ref_P2, t_ref_P3=None,
                         delta_mm=100.0):
    """
    카메라→월드 회전행렬 산출

    EEF가 각 축으로 delta_mm만큼 이동했을 때 t_ref 변화량으로
    카메라 좌표계 → 월드 좌표계 회전행렬을 구합니다.

    delta_t_cam = t_ref_Pn - t_ref_P0 (카메라 프레임 변화)
    이것이 월드에서 (delta_mm, 0, 0), (0, delta_mm, 0) 등에 대응
    """
    delta_m = delta_mm / 1000.0  # mm → m

    # 카메라 프레임에서 EEF 이동에 따른 t_ref 변화
    # EEF가 +X로 이동 → ID10도 +X → 카메라에서 보면 t_ref 변화
    dx_cam = t_ref_P1 - t_ref_P0  # X 이동 시 카메라 프레임 변화
    dy_cam = t_ref_P2 - t_ref_P0  # Y 이동 시

    # 월드에서의 단위 이동벡터 (m)
    # dx_cam은 월드 X축 delta_m 이동에 대응
    # → 카메라 프레임에서 월드 X축 단위벡터 = dx_cam / delta_m
    x_axis_in_cam = dx_cam / delta_m  # 월드 X축이 카메라에서 어느 방향인지
    y_axis_in_cam = dy_cam / delta_m

    if t_ref_P3 is not None:
        dz_cam = t_ref_P3 - t_ref_P0
        z_axis_in_cam = dz_cam / delta_m
    else:
        # Z축은 X×Y 외적으로 산출
        z_axis_in_cam = np.cross(x_axis_in_cam, y_axis_in_cam)

    # 정규직교화 (Gram-Schmidt)
    x_hat = x_axis_in_cam / np.linalg.norm(x_axis_in_cam)
    y_hat = y_axis_in_cam - np.dot(y_axis_in_cam, x_hat) * x_hat
    y_hat = y_hat / np.linalg.norm(y_hat)
    z_hat = np.cross(x_hat, y_hat)

    # R_world2cam: 월드 축이 카메라에서 어떻게 보이는지
    # 각 열이 월드 축의 카메라 방향
    R_world2cam = np.column_stack([x_hat, y_hat, z_hat])

    # R_cam2world = R_world2cam^T
    R_cam2world = R_world2cam.T

    # 검증: 직교 행렬인지
    det = np.linalg.det(R_cam2world)
    print(f'  R_cam2world det={det:.4f} (1.0이어야 정상)')
    if abs(det - 1.0) > 0.1:
        print(f'  ⚠ 회전행렬 비정상! 이동량이 부정확하거나 태그 검출 오류')

    return R_cam2world


def main():
    parser = argparse.ArgumentParser(description='AprilTag 월드 좌표 캘리브레이션 v2')
    parser.add_argument('--camera', choices=['left', 'right'], default='right')
    parser.add_argument('--ref-tag', type=int, default=10)
    parser.add_argument('--target-tag', type=int, default=0)
    parser.add_argument('--tag-size', type=float, default=0.05)
    parser.add_argument('--delta', type=float, default=100.0,
                        help='EEF 이동량 mm (default: 100)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TagCalibrationNode(
        camera_side=args.camera,
        ref_tag_id=args.ref_tag,
        target_tag_id=args.target_tag,
        tag_size=args.tag_size,
    )

    delta = args.delta

    print('=' * 60)
    print(f' AprilTag 월드 좌표 캘리브레이션 v2')
    print(f' 카메라: {node.camera_name}')
    print(f' 기준: ID{args.ref_tag} (EEF), 대상: ID{args.target_tag}')
    print(f' 이동량: {delta}mm')
    print('=' * 60)
    print()
    print(' Phase 1: 카메라→월드 회전행렬 산출')
    print(f'   P0: EEF=(0,0,0)')
    print(f'   P1: EEF=({delta},0,0)')
    print(f'   P2: EEF=(0,{delta},0)')
    print(f'   P3: EEF=(0,0,{delta}) [선택]')
    print()
    print(' Phase 2: 대상 태그 월드 좌표 산출')
    print('=' * 60)

    node.wait_for_camera_info()

    # ===== Phase 1: 회전행렬 산출 =====
    positions = [
        ('P0 (원점)', [0, 0, 0]),
        (f'P1 (X+{delta:.0f})', [delta, 0, 0]),
        (f'P2 (Y+{delta:.0f})', [0, delta, 0]),
        (f'P3 (Z+{delta:.0f})', [0, 0, delta]),
    ]

    ref_tvecs = {}   # {name: avg_tvec}
    tgt_tvecs = {}
    images = {}

    try:
        print('\n===== Phase 1: 카메라→월드 회전행렬 =====')

        for name, eef_pos in positions:
            print(f'\n  EEF를 {name} = ({eef_pos[0]},{eef_pos[1]},{eef_pos[2]})mm 으로 이동')
            cmd = input('  Enter=캡처, s=스킵, q=Phase2로: ').strip().lower()

            if cmd == 'q':
                break
            if cmd == 's':
                print('  스킵')
                continue

            avg_ref, avg_tgt, image = node.capture_multi_frames(n_frames=5)
            if avg_ref is None:
                print('  ⚠ 유효 프레임 없음')
                continue

            ref_tvecs[name] = avg_ref
            tgt_tvecs[name] = avg_tgt
            images[name] = image

            print(f'  t_ref = [{avg_ref[0]:.4f}, {avg_ref[1]:.4f}, {avg_ref[2]:.4f}]m')
            print(f'  t_tgt = [{avg_tgt[0]:.4f}, {avg_tgt[1]:.4f}, {avg_tgt[2]:.4f}]m')

        # 필수: P0, P1, P2
        p0_name = positions[0][0]
        p1_name = positions[1][0]
        p2_name = positions[2][0]
        p3_name = positions[3][0]

        if p0_name not in ref_tvecs or p1_name not in ref_tvecs or p2_name not in ref_tvecs:
            print('\n  ⚠ P0, P1, P2 최소 3개 필요합니다!')
            return

        t_ref_P3 = ref_tvecs.get(p3_name, None)

        R_cam2world = compute_R_cam2world(
            ref_tvecs[p0_name], ref_tvecs[p1_name], ref_tvecs[p2_name],
            t_ref_P3, delta_mm=delta
        )

        print(f'\n  R_cam2world:\n{R_cam2world}')

        # 검증: P0에서의 변환으로 각 위치 확인
        print('\n  검증 (t_ref 변환 → 월드 좌표 변화):')
        for name, eef_pos in positions:
            if name not in ref_tvecs:
                continue
            delta_cam = ref_tvecs[name] - ref_tvecs[p0_name]
            delta_world = R_cam2world @ delta_cam * 1000.0
            expected = np.array(eef_pos)
            error = delta_world - expected
            print(f'    {name}: 예상=({eef_pos[0]},{eef_pos[1]},{eef_pos[2]}) '
                  f'실제=({delta_world[0]:.1f},{delta_world[1]:.1f},{delta_world[2]:.1f}) '
                  f'오차=({error[0]:.1f},{error[1]:.1f},{error[2]:.1f})mm')

        # ===== Phase 2: 대상 태그 좌표 산출 =====
        print('\n===== Phase 2: 대상 태그 월드 좌표 =====')

        estimates = []
        for name, eef_pos in positions:
            if name not in ref_tvecs or name not in tgt_tvecs:
                continue

            # 카메라 프레임에서 ID10→ID0 벡터
            delta_cam = tgt_tvecs[name] - ref_tvecs[name]

            # 월드 좌표로 변환
            delta_world = R_cam2world @ delta_cam * 1000.0  # m → mm

            # ID0_world = EEF_pos + delta_world
            eef = np.array(eef_pos, dtype=np.float64)
            id0_world = eef + delta_world

            estimates.append(id0_world)
            print(f'  {name}: EEF=({eef_pos[0]},{eef_pos[1]},{eef_pos[2]}) '
                  f'→ ID{args.target_tag}=({id0_world[0]:.1f}, {id0_world[1]:.1f}, {id0_world[2]:.1f})mm')

        if not estimates:
            print('  데이터 없음')
            return

        final = np.mean(estimates, axis=0)
        final_std = np.std(estimates, axis=0)

        print(f'\n  {"="*50}')
        print(f'  최종 결과: ID{args.target_tag} = '
              f'({final[0]:.1f}, {final[1]:.1f}, {final[2]:.1f})mm')
        print(f'  위치간 편차: ({final_std[0]:.1f}, {final_std[1]:.1f}, {final_std[2]:.1f})mm')

        max_std = np.max(final_std)
        if max_std > 20:
            print(f'  ⚠ 편차 {max_std:.1f}mm (큼)')
        else:
            print(f'  ✓ 편차 양호 (max {max_std:.1f}mm)')

        # 저장
        save = input('\n  결과 저장? (Enter=저장, n=취소): ').strip().lower()
        if save != 'n':
            result = {
                'target_tag_id': args.target_tag,
                'ref_tag_id': args.ref_tag,
                'camera': args.camera,
                'tag_size_m': args.tag_size,
                'delta_mm': delta,
                'timestamp': datetime.now().isoformat(),
                'position_3d': {
                    'x_mm': float(final[0]),
                    'y_mm': float(final[1]),
                    'z_mm': float(final[2]),
                },
                'std_mm': {
                    'x': float(final_std[0]),
                    'y': float(final_std[1]),
                    'z': float(final_std[2]),
                },
                'R_cam2world': R_cam2world.tolist(),
                'observations': [
                    {'eef_pos': positions[i][1],
                     'estimate': estimates[i].tolist()}
                    for i in range(len(estimates))
                ],
            }

            yaml_path = os.path.join(
                DATA_DIR, f'apriltag_world_pos_ID{args.target_tag}_{args.camera}.yaml')
            with open(yaml_path, 'w') as f:
                yaml.dump(result, f, default_flow_style=False)

            json_path = os.path.join(
                DATA_DIR, f'apriltag_world_pos_ID{args.target_tag}_{args.camera}.json')
            with open(json_path, 'w') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)

            print(f'  저장: {yaml_path}')
            print(f'\n  ID{args.target_tag} = ({final[0]:.1f}, {final[1]:.1f}, {final[2]:.1f})mm')

    except KeyboardInterrupt:
        print('\n종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
