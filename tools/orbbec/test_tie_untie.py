#!/usr/bin/env python3
"""Orbbec Gemini2L 실영상으로 결속상태 분류 모델을 검증한다.

## 왜
`tie_untie_weights.pt`는 교차점을 crossing / tie / untie 로 구분한다.
이 분류가 맞아야 **이미 결속된 점을 다시 치지 않는다**(이중결속 방지).
숫자만으로는 맞는지 알 수 없으므로 **주석 이미지로 눈으로 확인**해야 한다.

## 무엇을 보나
1. 클래스별 검출 수와 신뢰도 — 모델이 실제로 구분하는가
2. 주석 이미지 — 결속된 점을 tie로, 안 된 점을 untie로 보는가
3. (--full) 전체 파이프라인 — 병합·다수결·로봇좌표 변환·결속대상 필터까지

## 판독
- 결속된 점이 untie로 나오면 → **이중결속 위험**. conf를 올리거나 재학습 필요
- 미결속 점이 tie로 나오면 → **누락**. tie_classes에 crossing 추가 검토
- crossing이 많으면 → 상태 판별이 애매한 조건(조명·각도). 그 비율을 봐야 한다

사용:
    python3 tools/orbbec/test_tie_untie.py                    # 검출만, 이미지 저장
    python3 tools/orbbec/test_tie_untie.py --full             # 전체 파이프라인
    python3 tools/orbbec/test_tie_untie.py --conf 0.4 --n 5
"""
import argparse
import os
import time
from collections import defaultdict

MODEL = '/home/koceti/ros2_ws/src/rebar_vision/model/tie_untie_weights.pt'
OUT_DIR = '/home/koceti/ros2_ws/data/tie_untie_test'

# 클래스별 색 (BGR). untie=주황(결속 대상), tie=청록(제외), crossing=파랑(애매)
COLOR = {'untie': (52, 104, 235), 'tie': (122, 175, 27), 'crossing': (214, 120, 42)}


def annotate(img, dets, names):
    import cv2
    out = img.copy()
    for (x1, y1, x2, y2, cls, cf) in dets:
        c = COLOR.get(cls, (128, 128, 128))
        cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)), c, 2)
        label = f'{cls} {cf:.2f}'
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        ty = max(int(y1) - 4, th + 4)
        cv2.rectangle(out, (int(x1), ty - th - 4), (int(x1) + tw + 4, ty + 2), c, -1)
        cv2.putText(out, label, (int(x1) + 2, ty - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    # 범례
    y = 22
    for k in ('untie', 'tie', 'crossing'):
        n = sum(1 for d in dets if d[4] == k)
        cv2.rectangle(out, (10, y - 12), (26, y + 2), COLOR[k], -1)
        cv2.putText(out, f'{k}: {n}', (32, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(out, f'{k}: {n}', (32, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
        y += 24
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--topic', default='/camera/color/image_raw')
    ap.add_argument('--conf', type=float, default=0.3)
    ap.add_argument('--n', type=int, default=3, help='프레임 수 (전체 파이프라인용)')
    ap.add_argument('--wait', type=float, default=15.0, help='영상 대기 초')
    ap.add_argument('--full', action='store_true',
                    help='OrbbecLocalizer 전체 파이프라인 (병합·로봇좌표·결속대상)')
    ap.add_argument('--tie-classes', default='untie')
    ap.add_argument('--out', default=OUT_DIR)
    a = ap.parse_args()

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from ultralytics import YOLO

    rclpy.init()
    node = rclpy.create_node('tie_untie_test')
    br = CvBridge()
    buf = []
    node.create_subscription(Image, a.topic,
                             lambda m: buf.append(br.imgmsg_to_cv2(m, 'bgr8')),
                             qos_profile_sensor_data)
    print(f'영상 대기: {a.topic} …')
    t0 = time.time()
    while time.time() - t0 < a.wait and not buf:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not buf:
        print(f'❌ {a.wait:.0f}초 안에 영상 없음 — Orbbec 실행 여부를 확인할 것')
        node.destroy_node(); rclpy.shutdown(); return

    model = YOLO(a.model)
    print(f'모델 {os.path.basename(a.model)}  클래스 {model.names}')
    print(f'영상 {buf[-1].shape[1]}x{buf[-1].shape[0]}  conf≥{a.conf}\n')

    # ── 1) 단일 프레임 검출 + 주석 이미지 ────────────────
    img = buf[-1]
    r = model(img, conf=a.conf, verbose=False)[0]
    dets = []
    stat = defaultdict(list)
    for b in r.boxes:
        cls = model.names[int(b.cls[0])]
        cf = float(b.conf[0])
        xy = b.xyxy[0]
        dets.append((float(xy[0]), float(xy[1]), float(xy[2]), float(xy[3]), cls, cf))
        stat[cls].append(cf)

    print('=' * 56)
    print('클래스별 검출  (단일 프레임)')
    print('=' * 56)
    if not stat:
        print('  검출 0개 — conf를 낮추거나 카메라 시야를 확인할 것')
    for k in sorted(stat):
        v = stat[k]
        print('  %-9s %3d개   신뢰도 평균 %.2f  최소 %.2f  최대 %.2f'
              % (k, len(v), sum(v) / len(v), min(v), max(v)))
    tot = sum(len(v) for v in stat.values())
    if tot:
        tc = {c.strip() for c in a.tie_classes.split(',') if c.strip()}
        n_tie = sum(len(v) for k, v in stat.items() if k in tc)
        print(f'\n  결속 대상({sorted(tc)}) = {n_tie}/{tot}개  '
              f'→ 제외 {tot - n_tie}개')

    os.makedirs(a.out, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    p_raw = os.path.join(a.out, f'{stamp}_raw.jpg')
    p_ann = os.path.join(a.out, f'{stamp}_annotated.jpg')
    cv2.imwrite(p_raw, img)
    cv2.imwrite(p_ann, annotate(img, dets, model.names))
    print(f'\n원본   {p_raw}')
    print(f'주석   {p_ann}')

    # ── 2) 전체 파이프라인 ───────────────────────────────
    if a.full:
        print('\n' + '=' * 56)
        print('전체 파이프라인 (병합 → 다수결 → 로봇좌표 → 결속대상)')
        print('=' * 56)
        try:
            from rebar_vision.orbbec_detector import OrbbecLocalizer
        except ImportError:
            print('  ❌ rebar_vision 미설치 — source install/setup.bash 후 재실행')
            node.destroy_node(); rclpy.shutdown(); return
        loc = OrbbecLocalizer(
            node, a.model,
            a.topic, '/camera/depth/image_raw', '/camera/color/camera_info',
            yrange={'r': (-3000, 3000), 'l': (-3000, 3000)},
            x_min=-3000, x_max=3000)
        t0 = time.time()
        while time.time() - t0 < a.wait and not loc.ready():
            rclpy.spin_once(node, timeout_sec=0.2)
        if not loc.ready():
            print('  ❌ depth/intrinsic 미수신 — depth_registration:=true 확인')
            node.destroy_node(); rclpy.shutdown(); return
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
        sets = loc.localize('r', conf=a.conf, n_frames=a.n)
        tc = {c.strip() for c in a.tie_classes.split(',') if c.strip()}
        print()
        for pose in ('r', 'l'):
            seg = sets.get(pose, [])
            if not seg:
                continue
            print(f'  [{pose}자세] {len(seg)}점')
            for c in seg:
                x, y, u, v = c[0], c[1], c[2], c[3]
                cls = c[4] if len(c) > 4 else '?'
                cf = c[5] if len(c) > 5 else 0.0
                mark = '🔩 결속' if (not loc.state_aware or cls in tc) else '— 제외'
                print('     X%8.1f Y%8.1f  (px %4d,%4d)  %-9s %.2f  %s'
                      % (x, y, u, v, cls, cf, mark))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
