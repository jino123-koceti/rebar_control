#!/usr/bin/env python3
"""DeepLabV3+ (SMP, MobileNetV2, 5클래스) 철근/주행영역 세그멘테이션 추론·테스트.

auto_best.pt = PyTorch Lightning 학습 체크포인트(model. 접두어) → SMP DeepLabV3Plus 재구성 후 로드.
5클래스: 0=background 1=floor 2=obstacle 3=rebar_h(가로) 4=rebar_v(세로)
⚠ 주행규칙: 로봇은 무한궤도로 **가로 철근 위를 밟고** 주행 → 지지면=철근, **floor·obstacle 둘 다 주행불가**.

출력: 클래스별 마스크 오버레이 + 픽셀 카운트 + 가로/세로 철근 개수(투영 피크) + 주행불가 비율.

  # 이미지 1장
  python3 tools/vision_test/deeplabv3_seg_test.py --image path/to.jpg
  # 폴더 일괄
  python3 tools/vision_test/deeplabv3_seg_test.py --dir path/to/imgs
  # ROS 토픽 (주행 카메라 스트림)
  python3 tools/vision_test/deeplabv3_seg_test.py --topic /zedxmini2/zed_node/rgb/color/rect/image

  --weights src/rebar_vision/model/auto_best.pt  --imgsz 512  --device cpu|cuda  --save-dir data/seg_test

전제: pip install segmentation-models-pytorch  (numpy1.26.4/opencv4.10 핀 깨지지 않게 주의 — 아래 README 참조)
"""
import os
import argparse
import glob
import numpy as np
import cv2
import torch

CLASS_MAP = {0: 'background', 1: 'floor', 2: 'human', 3: 'obstacle',
             4: 'rebar_h', 5: 'rebar_v', 6: 'wall'}
# BGR 팔레트. 지지면=철근(초록), 주행불가=floor(주황)/obstacle(진빨강)/human(밝은빨강)/wall(회색)
PALETTE = {0: (0, 0, 0), 1: (0, 140, 255), 2: (0, 0, 255), 3: (0, 0, 160),
           4: (0, 200, 0), 5: (0, 200, 120), 6: (150, 150, 150)}
IDX = {v: k for k, v in CLASS_MAP.items()}   # 이름→인덱스
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], np.float32)


class RebarSegmenter:
    """SMP DeepLabV3+ 로드 + 추론. (노드에서도 재사용 가능)"""

    def __init__(self, weights, device='cpu', imgsz=512, encoder='mobilenet_v2'):
        import segmentation_models_pytorch as smp
        self.device = device
        self.imgsz = imgsz
        self.net = smp.DeepLabV3Plus(encoder_name=encoder, encoder_weights=None,
                                     in_channels=3, classes=len(CLASS_MAP))
        ck = torch.load(weights, map_location='cpu', weights_only=False)
        sd = ck['state_dict'] if 'state_dict' in ck else ck
        # Lightning 'model.' 접두어 제거
        sd = {k[len('model.'):] if k.startswith('model.') else k: v for k, v in sd.items()}
        missing, unexpected = self.net.load_state_dict(sd, strict=False)
        if missing or unexpected:
            print(f'  [load] missing={len(missing)} unexpected={len(unexpected)} '
                  f'(0/0 이어야 정상)')
        self.net.eval().to(device)

    @torch.no_grad()
    def predict(self, bgr):
        """BGR 원본 → 원해상도 클래스 마스크(HxW uint8)."""
        h0, w0 = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        img = cv2.resize(rgb, (self.imgsz, self.imgsz)).astype(np.float32) / 255.0
        img = (img - IMAGENET_MEAN) / IMAGENET_STD
        x = torch.from_numpy(img.transpose(2, 0, 1)).unsqueeze(0).float().to(self.device)
        logits = self.net(x)                          # 1 x 5 x H x W
        m = logits.argmax(1)[0].to('cpu').numpy().astype(np.uint8)
        return cv2.resize(m, (w0, h0), interpolation=cv2.INTER_NEAREST)


def count_bars(mask, cls_idx, axis):
    """투영 피크로 철근 개수 추정. axis=0(열합=가로바 개수), 1(행합=세로바 개수)."""
    binm = (mask == cls_idx).astype(np.float32)
    if binm.sum() < 50:
        return 0, None
    prof = binm.sum(axis=axis)                        # 1D 프로파일
    prof = cv2.GaussianBlur(prof.reshape(-1, 1), (1, 9), 0).ravel()
    thr = max(prof.max() * 0.3, 1.0)
    above = prof > thr
    # 연속 구간 = 바 1개
    cnt, prev = 0, False
    for a in above:
        if a and not prev:
            cnt += 1
        prev = a
    return cnt, prof


def analyze(mask):
    """클래스별 픽셀수/비율 + 가로세로 개수 + 주행불가 비율.
    로봇은 철근 위를 주행 → 지지면=rebar, 주행불가=floor+obstacle."""
    total = mask.size
    counts = {CLASS_MAP[i]: int((mask == i).sum()) for i in CLASS_MAP}
    nh, _ = count_bars(mask, IDX['rebar_h'], axis=1)   # 가로철근: 행별 피크
    nv, _ = count_bars(mask, IDX['rebar_v'], axis=0)   # 세로철근: 열별 피크
    support = counts['rebar_h'] + counts['rebar_v']            # 궤도 지지면(철근)
    # 로봇은 철근 위를 주행 → floor(빈공간)·obstacle·human·wall 전부 주행불가
    nondrive = counts['floor'] + counts['obstacle'] + counts['human'] + counts['wall']
    nondrive_ratio = 100.0 * nondrive / max(total, 1)
    support_ratio = 100.0 * support / max(total, 1)
    return counts, nh, nv, nondrive_ratio, support_ratio


def colorize(mask):
    out = np.zeros((*mask.shape, 3), np.uint8)
    for i, c in PALETTE.items():
        out[mask == i] = c
    return out


def process(seg, bgr, name, save_dir):
    mask = seg.predict(bgr)
    counts, nh, nv, nondrive, support = analyze(mask)
    print(f'\n[{name}]')
    for k, v in counts.items():
        print(f'   {k:<10} {v:>8}px ({100.0*v/mask.size:4.1f}%)')
    print(f'   → 가로철근 ~{nh}개, 세로철근 ~{nv}개 | 철근지지면 {support:.0f}% | '
          f'주행불가(floor+obstacle) {nondrive:.0f}%')
    over = cv2.addWeighted(bgr, 0.6, colorize(mask), 0.4, 0)
    cv2.putText(over, f'H:{nh} V:{nv} rebar:{support:.0f}% noDrive:{nondrive:.0f}%', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    os.makedirs(save_dir, exist_ok=True)
    p = os.path.join(save_dir, f'{os.path.splitext(os.path.basename(name))[0]}_seg.png')
    cv2.imwrite(p, over)
    print(f'   저장 {p}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='/home/koceti/ros2_ws/src/rebar_vision/model/auto_best.pt')
    ap.add_argument('--image'); ap.add_argument('--dir'); ap.add_argument('--topic')
    ap.add_argument('--imgsz', type=int, default=512)
    ap.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    ap.add_argument('--save-dir', default='/home/koceti/ros2_ws/data/seg_test')
    args = ap.parse_args()

    print(f'모델 로드: {args.weights} (device={args.device}, imgsz={args.imgsz})')
    seg = RebarSegmenter(args.weights, args.device, args.imgsz)
    print('  로드 완료.')

    if args.image:
        process(seg, cv2.imread(args.image), args.image, args.save_dir)
    elif args.dir:
        for f in sorted(glob.glob(os.path.join(args.dir, '*'))):
            if f.lower().endswith(('.jpg', '.png', '.jpeg', '.bmp')):
                process(seg, cv2.imread(f), f, args.save_dir)
    elif args.topic:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        rclpy.init(); node = Node('seg_test'); br = CvBridge(); buf = []
        node.create_subscription(Image, args.topic,
                                 lambda m: buf.append(br.imgmsg_to_cv2(m, 'bgr8')),
                                 qos_profile_sensor_data)
        print(f'토픽 {args.topic} 구독 — Ctrl+C 로 종료, 매 프레임 분석')
        try:
            i = 0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if buf:
                    process(seg, buf[-1], f'frame_{i:04d}', args.save_dir); i += 1; buf.clear()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node(); rclpy.shutdown()
    else:
        print('⚠ --image / --dir / --topic 중 하나 지정')


if __name__ == '__main__':
    main()
