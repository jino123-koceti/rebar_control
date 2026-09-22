#!/usr/bin/env python3
"""deck_edge 판정을 실시간 추적한다 — 장비를 움직이며 "언제 바뀌나"를 보는 용도.

## 왜 별도 도구인가
deck_edge_debug.py는 **한 장**만 보고 끝난다. 임계를 넘는 지점을 찾으려면
장비를 조금씩 움직이며 rebar_frac이 어떻게 변하는지 **연속으로** 봐야 한다.

## 하는 일
- /deck_edge_block 을 구독해 rebar_frac·verdict·차단방향을 표로 찍는다
- **판정(verdict/차단방향)이 바뀌는 순간** 그 시점 카메라 영상을 저장한다
  → 나중에 "0.45를 넘던 그 장면"을 눈으로 확인할 수 있다
- 종료 시 요약: 판정별 rebar_frac 범위, 전이 목록

⚠ YOLO를 돌리지 않는다(원본 영상만 저장). 이미 도는 deck_edge_node의 판정을
   그대로 받아쓰므로 GPU를 추가로 쓰지 않고, 실제 운용 판정과 100% 일치한다.

사용:
    python3 tools/vision_test/deck_edge_watch.py                 # back(기본)
    python3 tools/vision_test/deck_edge_watch.py --cam front --sec 300
"""
import argparse
import json
import os
import time

OUT = '/home/koceti/ros2_ws/data/deck_edge_watch'
TOPIC = {
    'front': '/zedxmini2/zed_node/rgb/color/rect/image/compressed',
    'back': '/zedxmini1/zed_node/rgb/color/rect/image/compressed',
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', default='back', choices=['front', 'back'])
    ap.add_argument('--sec', type=float, default=180.0)
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('deck_edge_watch')
    br = CvBridge()
    state = {'img': None, 'last_key': None, 'rows': [], 'trans': []}
    stamp = time.strftime('%Y%m%d_%H%M%S')

    def on_img(m):
        state['img'] = br.compressed_imgmsg_to_cv2(m, 'bgr8')

    def on_block(m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        frac = float(d.get('rebar_frac', 0.0))
        verdict = str(d.get('verdict', '?'))
        fwd, bwd = bool(d.get('forward')), bool(d.get('backward'))
        cam = str(d.get('active_cam', '?'))
        if cam != a.cam:                      # 다른 카메라 판정은 무시
            return
        now = time.time()
        state['rows'].append((now, verdict, frac, fwd, bwd))

        key = (verdict, fwd, bwd)
        if key != state['last_key']:
            prev = state['last_key']
            state['trans'].append((now, prev, key, frac))
            blocked = ' '.join(x for x, v in (('전진', fwd), ('후진', bwd)) if v) or '없음'
            print(f"\n{'='*64}\n"
                  f"[{time.strftime('%H:%M:%S')}] ★ 판정 변화: "
                  f"{prev[0] if prev else '-'} → {verdict}\n"
                  f"   rebar_frac={frac:.3f}   차단={blocked}\n"
                  f"   {d.get('reason', '')}\n{'='*64}", flush=True)
            if state['img'] is not None:
                p = os.path.join(
                    a.out, f"{stamp}_{a.cam}_{verdict}_{frac:.3f}.jpg")
                cv2.imwrite(p, state['img'])
                print(f'   영상 저장: {p}', flush=True)
            state['last_key'] = key

    node.create_subscription(CompressedImage, TOPIC[a.cam], on_img,
                             qos_profile_sensor_data)
    node.create_subscription(String, '/deck_edge_block', on_block, 10)

    print(f'■ {a.cam} 카메라 판정 추적 시작 ({a.sec:.0f}초). 장비를 조금씩 움직이세요.\n'
          f'  판정이 바뀔 때마다 표시하고 그 순간 영상을 저장합니다.\n', flush=True)

    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < a.sec:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.time()
        if now - last_print >= 2.0 and state['rows']:      # 2초마다 현재값
            _, v, f, fw, bw = state['rows'][-1]
            blocked = ' '.join(x for x, q in (('전진', fw), ('후진', bw)) if q) or '없음'
            print(f"  [{time.strftime('%H:%M:%S')}] {v:5s} "
                  f"rebar_frac={f:.3f}  차단={blocked}", flush=True)
            last_print = now

    # ── 요약 ────────────────────────────────────────────
    print(f"\n{'='*64}\n■ 요약", flush=True)
    if not state['rows']:
        print('  수신 없음 — deck_edge_node가 이 카메라를 보고 있는지 확인할 것')
    else:
        by = {}
        for _, v, f, _, _ in state['rows']:
            by.setdefault(v, []).append(f)
        for v, fs in by.items():
            print(f'  {v:5s}  rebar_frac {min(fs):.3f} ~ {max(fs):.3f}  '
                  f'({len(fs)}회)')
        print(f'\n  판정 전이 {len(state["trans"])}회:')
        for t, prev, key, f in state['trans']:
            print(f'    {time.strftime("%H:%M:%S", time.localtime(t))}  '
                  f'{prev[0] if prev else "-":5s} → {key[0]:5s}  '
                  f'rebar_frac={f:.3f}')
    print(f'  영상: {a.out}', flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
