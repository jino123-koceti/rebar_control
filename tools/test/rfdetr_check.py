#!/usr/bin/env python3
"""교차점 검출 모델(RF-DETR) 확인 도구 — 이식 전에 이것부터 돌린다.

## 왜 필요한가

모델은 `new_weights_260930.pt` (RF-DETR **Medium**, Roboflow 학습, 3클래스)이다.
그런데 체크포인트 안에 **클래스 이름이 없다** (`args.class_names is None`). rfdetr 은
`class_id` 를 "class_names 의 0-기반 인덱스" 로만 돌려주므로, 이름 순서는 **우리가**
알려줘야 한다. 순서를 틀리면 crossing 과 tie 가 조용히 뒤바뀌고, 그건 곧
**이미 묶은 곳을 다시 묶는(이중결속)** 사고다. 그래서 이식 전에 실제 영상으로
`class_id` ↔ 이름을 눈으로 확인한다.

또 하나. 2차년도는 3클래스(crossing/tie/untie)를 2026-09-08 에 **철회**했다.
같은 교차점이 반복 검출에서 75% 뒤집혀서였다. 신규 모델도 같은 문제가 있는지
`--repeat` 로 같은 장면을 여러 번 검출해 **뒤집힘 비율**을 재 본다.

## 쓰는 법

  # 저장된 영상으로
  python3 tools/test/rfdetr_check.py 사진.jpg

  # 카메라(교차점 = Gemini 2 L)에서 받아서. 드라이버가 떠 있어야 한다
  python3 tools/test/rfdetr_check.py --topic /camera/color/image_raw

  # 클래스 순서를 바꿔 보며 어느 쪽이 맞는지 판단
  python3 tools/test/rfdetr_check.py 사진.jpg --classes tie,crossing,untie

  # 같은 장면 10회 검출 → 클래스 뒤집힘 비율
  python3 tools/test/rfdetr_check.py --topic /camera/color/image_raw --repeat 10

결과 영상은 `--out` (기본 /tmp/rfdetr_check.jpg) 에 저장된다. 박스 중심에 십자를
그린다 — 2차년도 파이프라인이 쓰는 좌표가 박스 중심이기 때문이다.
"""

import argparse
import os
import sys
import warnings
from collections import Counter, defaultdict

warnings.filterwarnings('ignore')

WEIGHTS = '/home/koceti/ros2_ws/src/rebar_control/model/new_weights_260930.pt'
# [확정] 2026-09-30: Roboflow 클래스 순서는 crossing, tie, untie 이고,
# **인덱스는 1부터**다 (slot0 은 자리표시). 체크포인트에 이름이 없어 코드가 준다.
#   slot0 (자리표시)  — threshold 0.05 에서도 안 뜬다
#   slot1 crossing    — 안 뜬다. 라벨링을 묶임 여부로만 한 것으로 보인다
#   slot2 tie   기결속 — 영상에서 검은 클립이 보이는 교차점에 붙었다
#   slot3 untie 미결속 — 맨 교차점에 붙었다
DEFAULT_CLASSES = ('_', 'crossing', 'tie', 'untie')
# 이 체크포인트의 분류 슬롯 수 (class_embed 출력). 이름 개수와 다를 수 있다
NUM_SLOTS = 4
# 25px: 2차년도 병합 반경. 같은 교차점인지 판단하는 기준으로 그대로 쓴다
MERGE_PX = 25


def load_model(weights, num_classes, fp16=False):
    import contextlib
    import io
    import rfdetr
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        # Medium 으로 확정: 체크포인트 텐서 509개가 값까지 전부 일치한다
        # (Nano 44개 누락, Small 22개 누락, Base 는 patch_size 불일치로 거부)
        m = rfdetr.RFDETRMedium(pretrain_weights=weights, num_classes=num_classes)
        if fp16:
            # rfdetr 이 직접 권하는 최적화. 메서드 이름은 `inference()` 다
            # (`optimize_for_inference` 가 아니다 — 경고 문구가 잘못 안내한다).
            import torch
            m.inference(dtype=torch.float16)
    return m


def slot_name(cid, classes):
    """슬롯 번호 → 표시할 이름. 이름을 모르면 번호를 그대로 쓴다."""
    if classes and 0 <= cid < len(classes):
        return classes[cid]
    return f'slot{cid}'


def detect(model, bgr, classes, threshold):
    """2차년도 `detect_crossings()` 가 기대하는 형식으로 돌려준다.

    반환: [(u, v, 클래스이름, 신뢰도, 슬롯번호), ...]  — u,v 는 **박스 중심**
    앞 네 개가 2차년도 형식이고 슬롯번호는 확인용으로 덧붙인 것이다.
    그 뒤 병합·depth 역투영·CAD 강체변환은 2차년도 코드를 그대로 쓴다.
    """
    # OpenCV BGR → 모델은 RGB 를 받는다.
    # `bgr[:, :, ::-1]` 은 쓰면 안 된다 — 음수 stride 뷰가 되어 torch 가 거부한다.
    import cv2
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    det = model.predict(rgb, threshold=threshold, include_source_image=False)
    out = []
    for (x1, y1, x2, y2), cid, conf in zip(det.xyxy, det.class_id, det.confidence):
        cid = int(cid)
        out.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0,
                    slot_name(cid, classes), float(conf), cid))
    return out


def grab_from_topic(topic, timeout):
    """ROS 토픽에서 한 장 받는다. 드라이버가 떠 있어야 한다."""
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node('rfdetr_check')
    bridge = CvBridge()
    box = {}
    node.create_subscription(Image, topic, lambda m: box.setdefault('img', m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while rclpy.ok() and 'img' not in box:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_clock().now().nanoseconds / 1e9 > end:
            break
    node.destroy_node()
    rclpy.shutdown()
    if 'img' not in box:
        raise TimeoutError(f"{topic} 에서 {timeout:.0f}초 동안 영상이 오지 않았다. "
                           f"드라이버가 떠 있나? (ros2 topic hz {topic})")
    return bridge.imgmsg_to_cv2(box['img'], desired_encoding='bgr8')


def annotate(bgr, dets, path):
    """박스 중심에 십자를 그린다 — 2차년도 파이프라인이 쓰는 좌표가 박스 중심이다.

    슬롯마다 색을 달리한다. 같은 지점에 여러 슬롯이 겹쳐 뜨는지 눈으로 보려면
    색이 달라야 한다.
    """
    import cv2
    # 슬롯별 색 (BGR). 겹침을 구별할 수 있게 확실히 다른 색을 쓴다
    COLORS = [(0, 255, 255), (255, 128, 0), (0, 255, 0), (255, 0, 255),
              (0, 0, 255), (255, 255, 0)]
    img = bgr.copy()
    for u, v, name, conf, cid in sorted(dets, key=lambda d: d[3]):
        c = COLORS[cid % len(COLORS)]
        u, v = int(round(u)), int(round(v))
        cv2.drawMarker(img, (u, v), c, cv2.MARKER_CROSS, 20, 2)
        cv2.putText(img, f'{name} {conf:.2f}', (u + 12, v - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2, cv2.LINE_AA)
    # 범례
    slots = sorted({d[4] for d in dets})
    for i, cid in enumerate(slots):
        c = COLORS[cid % len(COLORS)]
        nm = next(d[2] for d in dets if d[4] == cid)
        cv2.putText(img, f'{nm}', (12, 28 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, c, 2, cv2.LINE_AA)
    cv2.imwrite(path, img)


def repeat_stability(model, bgr, classes, threshold, n):
    """같은 장면을 n 회 검출해 두 가지를 **따로** 센다.

    · 프레임 **내** 중복 — 한 프레임에서 같은 지점에 여러 슬롯이 동시에 뜨는 것.
      DETR 은 NMS 를 쓰지 않아 흔히 일어난다. 중복 자체는 불안정이 아니다.
    · 프레임 **간** 뒤집힘 — 프레임마다 그 지점의 **최고 신뢰도 슬롯**이 달라지는 것.
      2차년도가 3클래스를 2026-09-08 에 철회한 근거가 이것이다 (75% 뒤집힘).

    이 둘을 섞어 세면 중복을 뒤집힘으로 오판한다.
    """
    top_per_frame = defaultdict(list)     # 대표좌표 → 프레임별 최고 슬롯 이름
    dup_frames = defaultdict(int)         # 대표좌표 → 중복이 난 프레임 수
    anchors = []                          # 대표좌표 목록

    def anchor_of(u, v):
        for a in anchors:
            if abs(a[0] - u) <= MERGE_PX and abs(a[1] - v) <= MERGE_PX:
                return a
        anchors.append((u, v))
        return anchors[-1]

    for _ in range(n):
        per_point = defaultdict(list)
        for u, v, name, conf, _cid in detect(model, bgr, classes, threshold):
            per_point[anchor_of(u, v)].append((conf, name))
        for a, lst in per_point.items():
            top_per_frame[a].append(max(lst)[1])
            if len({nm for _c, nm in lst}) > 1:
                dup_frames[a] += 1

    print(f"\n■ 반복 검출 {n}회 (병합 반경 {MERGE_PX}px, 지점 {len(anchors)}개)")

    print("\n  [프레임 내 중복] 한 프레임에서 같은 지점에 여러 슬롯이 동시에 뜨는가")
    dups = [(a, c) for a, c in dup_frames.items() if c]
    if dups:
        for a, c in sorted(dups, key=lambda x: -x[1]):
            print(f"    ({a[0]:.0f},{a[1]:.0f})  {c}/{n} 프레임에서 중복")
        print("    → DETR 은 NMS 가 없어 정상일 수 있다. 다만 이식할 때 지점당 하나로")
        print("      줄여야 한다 (최고 신뢰도만 남기기).")
    else:
        print("    중복 없음.")

    print("\n  [프레임 간 뒤집힘] 프레임마다 그 지점의 최고 슬롯이 달라지는가")
    flipped = [(a, Counter(v)) for a, v in top_per_frame.items() if len(set(v)) > 1]
    if flipped:
        for a, c in flipped:
            detail = ", ".join(f'{nm}×{k}' for nm, k in c.most_common())
            print(f"    ({a[0]:.0f},{a[1]:.0f})  {detail}")
        rate = 100.0 * len(flipped) / max(1, len(anchors))
        print(f"    → 지점 {len(anchors)}개 중 {len(flipped)}개 ({rate:.0f}%) 가 뒤집힌다.")
        print("      2차년도는 75% 에서 3클래스를 철회했다. 단일 클래스로 접거나")
        print("      다수결(여러 프레임 투표)을 쓸지 판단할 것.")
    else:
        print("    뒤집힘 없음 — 최고 슬롯은 프레임마다 일정하다.")


def main():
    ap = argparse.ArgumentParser(description='RF-DETR 교차점 검출 확인')
    ap.add_argument('image', nargs='?', help='검사할 영상 파일')
    ap.add_argument('--topic', help='대신 ROS 영상 토픽에서 한 장 받는다')
    ap.add_argument('--timeout', type=float, default=10.0, help='토픽 대기 시간(초)')
    ap.add_argument('--weights', default=WEIGHTS)
    ap.add_argument('--classes', default=','.join(DEFAULT_CLASSES),
                    help='슬롯 순서대로 클래스 이름 (기본 _,crossing,tie,untie). '
                         'slot0 은 자리표시라 "_" 로 둔다')
    ap.add_argument('--threshold', type=float, default=0.3,
                    help='신뢰도 하한 (2차년도 conf 0.3 과 같게)')
    ap.add_argument('--repeat', type=int, default=0,
                    help='같은 장면을 n 회 검출해 클래스 뒤집힘을 센다')
    ap.add_argument('--fp16', action='store_true',
                    help='FP16 추론 최적화를 켠다 (rfdetr 이 권하는 방법)')
    ap.add_argument('--out', default='/tmp/rfdetr_check.jpg')
    a = ap.parse_args()

    if not a.image and not a.topic:
        ap.error('영상 파일이나 --topic 중 하나는 필요하다')
    if not os.path.exists(a.weights):
        print(f"실패 — 가중치가 없다: {a.weights}")
        return 1

    import cv2
    if a.topic:
        try:
            bgr = grab_from_topic(a.topic, a.timeout)
        except Exception as e:
            print(f"실패 — {e}")
            return 1
        print(f"■ 영상: {a.topic} 에서 받음  {bgr.shape[1]}x{bgr.shape[0]}")
    else:
        bgr = cv2.imread(a.image)
        if bgr is None:
            print(f"실패 — 영상을 읽을 수 없다: {a.image}")
            return 1
        print(f"■ 영상: {a.image}  {bgr.shape[1]}x{bgr.shape[0]}")

    classes = (tuple(x.strip() for x in a.classes.split(',') if x.strip())
               if a.classes else None)
    if classes:
        print("■ 클래스 순서: " + ', '.join(f'{i}={c}' for i, c in enumerate(classes)))
    else:
        print(f"■ 클래스 이름 미지정 — 슬롯 번호로 표시한다 "
              f"(이 체크포인트의 분류 슬롯 {NUM_SLOTS}개)")

    # num_classes 는 실제로는 체크포인트가 결정한다 (3 을 넣어도 4 를 넣어도 결과가
    # 같았다). 그래도 인자가 필수라 슬롯 수를 넘긴다.
    model = load_model(a.weights, len(classes) if classes else NUM_SLOTS, fp16=a.fp16)
    dets = detect(model, bgr, classes, a.threshold)

    print(f"\n■ 검출 {len(dets)}개 (신뢰도 ≥ {a.threshold})   형식: (u, v, 이름, 신뢰도)")
    for u, v, name, conf, _cid in sorted(dets, key=lambda d: -d[3]):
        print(f"  ({u:7.1f}, {v:7.1f})  {name:10s} {conf:.3f}")
    if dets:
        print("\n■ 클래스별 개수: " +
              ", ".join(f'{k}={v}' for k, v in Counter(d[2] for d in dets).items()))
    else:
        print("  검출이 없다. 철근 교차점이 보이는 장면인지, threshold 가 높지 않은지 확인할 것.")

    annotate(bgr, dets, a.out)
    print(f"\n■ 결과 영상: {a.out}")
    print("  ⚠ 박스 중심에 십자를, 슬롯마다 다른 색으로 그렸다. 확인할 것:")
    print("    · 십자가 교차점 위에 오는가 (2차년도 좌표변환이 박스 중심을 쓴다)")
    print("    · 슬롯 번호가 실제 무엇에 붙는가 — 한 교차점에 두 슬롯이 겹치면")
    print("      학습 데이터가 교차점 박스와 상태 박스를 둘 다 달았다는 뜻이다")
    print("    이름 순서를 알게 되면 --classes 로 넘긴다.")

    if a.repeat > 1:
        repeat_stability(model, bgr, classes, a.threshold, a.repeat)
    return 0


if __name__ == '__main__':
    sys.exit(main())
