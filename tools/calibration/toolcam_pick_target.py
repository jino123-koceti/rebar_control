#!/usr/bin/env python3
"""툴캠 목표픽셀 클릭 선택기.
라이브 뷰에서 공구 결속 작용점을 좌클릭 → 목표픽셀 설정 → 's'로 yaml 저장.
YOLO 교차점(초록)도 참고표시. 저장 시 확인용 png도 남김.

  python3 toolcam_pick_target.py --pose left

  좌클릭  목표픽셀 선택(자주색 X)
  s       toolcam_gain_{pose}.yaml 의 target_pixel 로 저장 + /tmp/toolcam_target_{pose}.png
  d       YOLO 검출 on/off
  +/-     표시 배율
  q       종료
"""
import os, argparse, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
from ultralytics import YOLO

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/toolcam_crossing.pt'
TOPIC = '/zedxone/zed_node/rgb/rect/image'
CDIR = '/home/koceti/ros2_ws/data/calibration'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', choices=['right', 'left'], default='left')
    ap.add_argument('--scale', type=float, default=0.5)
    args = ap.parse_args()
    yml = os.path.join(CDIR, f'toolcam_gain_{args.pose}.yaml')

    # 현재 목표 로드 (있으면)
    cur_target = None
    if os.path.exists(yml):
        g = yaml.safe_load(open(yml)).get('toolcam_gain', {})
        if g.get('target_pixel'):
            cur_target = tuple(g['target_pixel'])

    rclpy.init()
    node = Node('toolcam_pick_target')
    br = CvBridge(); buf = []
    node.create_subscription(Image, TOPIC,
                             lambda m: buf.append(br.imgmsg_to_cv2(m, 'bgr8')),
                             qos_profile_sensor_data)
    model = YOLO(MODEL)

    state = {'pick': (list(cur_target) if cur_target else None), 'sc': args.scale,
             'detect': True, 'crossings': [], 'last_infer': 0.0}

    def on_mouse(ev, x, y, flags, param):
        if ev == cv2.EVENT_LBUTTONDOWN:
            sc = state['sc']
            state['pick'] = [int(x / sc), int(y / sc)]  # 원본좌표
            print(f'  선택: ({state["pick"][0]}, {state["pick"][1]})')

    win = 'pick target (L-click=set, s=save, d=detect, q=quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 1280, 800)
    cv2.setMouseCallback(win, on_mouse)
    print('=' * 60)
    print(f' 목표픽셀 클릭 선택 — 자세={args.pose}, 현재목표={cur_target}')
    print('  공구 결속 작용점(교차점이 물리는 자리)을 좌클릭 → s 저장')
    print('=' * 60)

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            if not buf:
                continue
            img = buf[-1]
            if len(buf) > 5:
                del buf[:-2]
            if state['detect'] and time.time() - state['last_infer'] > 0.4:
                r = model(img, conf=0.15, verbose=False)[0]
                state['crossings'] = [((float(b.xyxy[0][0]+b.xyxy[0][2])/2),
                                       (float(b.xyxy[0][1]+b.xyxy[0][3])/2))
                                      for b in r.boxes]
                state['last_infer'] = time.time()
            sc = state['sc']
            disp = cv2.resize(img, None, fx=sc, fy=sc)
            for (cx, cy) in state['crossings']:
                cv2.circle(disp, (int(cx*sc), int(cy*sc)), 12, (0, 220, 0), 2)
            if cur_target:
                cv2.drawMarker(disp, (int(cur_target[0]*sc), int(cur_target[1]*sc)),
                               (120, 120, 120), cv2.MARKER_TILTED_CROSS, 22, 1)
            if state['pick']:
                px, py = state['pick']
                cv2.drawMarker(disp, (int(px*sc), int(py*sc)), (255, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 30, 2)
                cv2.putText(disp, f'({px},{py})', (int(px*sc)+14, int(py*sc)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            cv2.putText(disp, f'pose={args.pose}  L-click=set  s=save  d=detect({state["detect"]})  q',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(win, disp)
            k = cv2.waitKey(20) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord('d'):
                state['detect'] = not state['detect']
            elif k in (ord('+'), ord('=')):
                state['sc'] = min(1.0, state['sc'] + 0.1)
            elif k == ord('-'):
                state['sc'] = max(0.2, state['sc'] - 0.1)
            elif k == ord('s'):
                if not state['pick']:
                    print('  먼저 좌클릭으로 목표 선택'); continue
                g = {}
                if os.path.exists(yml):
                    g = yaml.safe_load(open(yml)) or {}
                g.setdefault('toolcam_gain', {})
                g['toolcam_gain']['target_pixel'] = [int(state['pick'][0]),
                                                     int(state['pick'][1])]
                yaml.safe_dump(g, open(yml, 'w'), default_flow_style=False)
                # 확인용 이미지 저장
                out = f'/tmp/toolcam_target_{args.pose}.png'
                cv2.imwrite(out, disp)
                print(f'  ✅ 저장: target_pixel={state["pick"]} → {yml}')
                print(f'     확인이미지 → {out}')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
