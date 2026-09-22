#!/usr/bin/env python3
"""현장 테스트 데이터 수집 — 주행 카메라 + 작업영역 Orbbec.

## 설계 원칙
1. **구독만 한다.** 아무 토픽도 발행하지 않아 주행/결속에 영향이 없다.
2. **비슷한 프레임은 안 담는다.** 현장에서 몇 시간 돌리면 같은 장면 수천 장이
   쌓이는데, 라벨링 비용과 Roboflow 할당량만 먹고 학습에는 도움이 안 된다.
   → 직전 저장분과 **화면이 얼마나 달라졌는지**로 거른다(`--diff`).
3. **디스크를 지킨다.** 남은 용량이 `--min-free-gb` 아래로 떨어지면 자동 정지.
   현장에서 디스크가 차서 주행 로그까지 못 쓰는 게 최악이다.
4. **업로드는 나중에.** 현장 네트워크를 믿을 수 없다. 여기선 폴더에만 모으고
   업로드는 돌아와서 `upload_to_roboflow.py`로 한다(`--print-upload-cmd` 참조).

## 저장물
    data/field/<세션>/<카메라>/NNNNNN_<시각>.jpg
    data/field/<세션>/meta.jsonl        프레임별 메타(시각·카메라·주행상태)
    data/field/<세션>/summary.json      세션 요약(종료 시)

메타에 **주행 방향·deck_edge 판정·엔코더 위치**를 같이 남긴다. 나중에
"주행 불가로 판정한 그 장면"만 뽑아 보는 식으로 쓸 수 있다.

## 사용
    python3 tools/collect/field_collect.py                    # 전 카메라, 1Hz
    python3 tools/collect/field_collect.py --cams front,back  # 일부만
    python3 tools/collect/field_collect.py --hz 0.5 --diff 12 # 느리게·더 엄격히
    python3 tools/collect/field_collect.py --list             # 카메라 목록만 보기

Ctrl+C로 정지하면 요약을 출력한다.
"""
import argparse
import json
import os
import shutil
import time

# 카메라 정의: 이름 → (토픽, 설명). compressed가 있으면 대역폭이 훨씬 적다.
CAMS = {
    'front':  ('/zedxmini2/zed_node/rgb/color/rect/image/compressed', '전진 주행'),
    'back':   ('/zedxmini1/zed_node/rgb/color/rect/image/compressed', '후진 주행'),
    # ⚠ 2026-09-16 갱신: 좌측이 Orbbec305 → **ZED X One**으로 교체됐다(2026-09-14).
    #   옛 'work_l'(/camera_left/...)은 **존재하지 않는 토픽**이라 지정해도 0장이 담긴다.
    'right':  ('/zedxone/zed_node/rgb/color/rect/image/compressed',   '우측 측면(모노)'),
    'left':   ('/zedxone_left/zed_node/rgb/color/rect/image/compressed', '좌측 측면(모노)'),
    'work':   ('/camera/color/image_raw/compressed',                  '작업영역 Orbbec'),
}
# 옛 이름 호환 — 'tool'은 우측 모노의 예전 이름이다.
_ALIAS = {'tool': 'right'}
# depth를 같이 담을 카메라 → depth 토픽. ⚠ RGB와 **짝**으로만 저장한다 —
# 라벨은 컬러에 붙고 depth는 그 라벨 위치의 높이를 읽는 용도라, 시각이 어긋나면
# 쓸모가 없다. 그래서 컬러를 저장하는 순간의 **가장 최근 depth**를 같이 쓰고
# 둘의 시차(depth_dt_ms)를 메타에 남긴다.
DEPTH_TOPICS = {
    'work':   '/camera/depth/image_raw',
    'work_l': '/camera_left/depth/image_raw',
}
OUT_ROOT = '/home/koceti/ros2_ws/data/field'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cams', default='all',
                    help=f"쉼표구분. 가능: {','.join(CAMS)} (기본 all)")
    ap.add_argument('--hz', type=float, default=1.0, help='카메라당 최대 저장률')
    ap.add_argument('--diff', type=float, default=6.0,
                    help='직전 저장분과 평균 픽셀차가 이 값 미만이면 건너뜀(0=끔)')
    ap.add_argument('--quality', type=int, default=90, help='JPEG 품질')
    ap.add_argument('--min-free-gb', type=float, default=5.0,
                    dest='min_free_gb', help='남은 용량이 이 아래면 정지')
    ap.add_argument('--max-min', type=float, default=0.0, dest='max_min',
                    help='이 시간(분) 뒤 자동 종료 (0=무제한)')
    ap.add_argument('--out', default=OUT_ROOT)
    ap.add_argument('--tag', default='', help='세션 이름에 붙일 꼬리표')
    # 폴더명을 밖에서 지정. ⚠ 없으면 이 도구가 자기 시각으로 만드는데, 띄우는
    #   쪽이 그 이름을 추측하면 1초 차이로 어긋난다(실측 2026-09-08).
    ap.add_argument('--session', default='', help='세션 폴더명을 직접 지정')
    # ★ 레벨봉 학습데이터용 (2026-09-02).
    #   deck_edge가 이미 rod_n을 발행하므로 **GPU를 추가로 쓰지 않는다**.
    #   ⚠ deck_edge는 **진행방향 카메라만** 추론한다(active_only). 그래서 rod 정보는
    #     그 카메라 기준이고, 측면(work_l 등)은 별도 요청이 있어야 판정된다.
    #     → 측면 데이터는 --only-rods 없이 그냥 모으는 게 낫다.
    ap.add_argument('--only-rods', action='store_true', dest='only_rods',
                    help='레벨봉이 검출된 프레임만 저장(진행방향 카메라 기준)')
    # ★ 2층 배근용 (2026-09-08). 상단근/하단근은 층간 150mm라 **깊이가 있어야**
    #   교차점이 진짜인지(같은 층인지) 가릴 수 있다. 컬러만으론 판별 불가.
    ap.add_argument('--depth', action='store_true',
                    help='Orbbec depth를 컬러와 짝지어 16비트 PNG로 같이 저장')
    ap.add_argument('--list', action='store_true', help='카메라 목록만 출력')
    a = ap.parse_args()

    if a.list:
        print('사용 가능한 카메라:')
        for k, (t, d) in CAMS.items():
            print(f'  {k:<8} {d:<20} {t}')
        return

    names = (list(CAMS) if a.cams == 'all'
             else [_ALIAS.get(c.strip(), c.strip()) for c in a.cams.split(',')])
    bad = [n for n in names if n not in CAMS]
    if bad:
        print(f'❌ 모르는 카메라: {bad}   (가능: {list(CAMS)})')
        return

    import cv2
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    # ⚠ `/encoder_odom`은 이름과 달리 **PoseStamped**다(Odometry 아님).
    #   타입을 틀리면 구독이 조용히 안 붙어 메타의 odom_mm이 계속 null이 된다.
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import qos_profile_sensor_data
    from rebar_base_interfaces.msg import DriveControl
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String

    stamp = a.session or (time.strftime('%Y%m%d_%H%M%S')
                          + (f'_{a.tag}' if a.tag else ''))
    root = os.path.join(a.out, stamp)
    for n in names:
        os.makedirs(os.path.join(root, n), exist_ok=True)
    depth_cams = [n for n in names if a.depth and n in DEPTH_TOPICS]
    for n in depth_cams:
        os.makedirs(os.path.join(root, n + '_depth'), exist_ok=True)
    meta_path = os.path.join(root, 'meta.jsonl')

    rclpy.init()
    node = rclpy.create_node('field_collect')
    br = CvBridge()
    # 주행 맥락 — 나중에 "그 장면"을 찾아내는 열쇠가 된다
    # ⚠ `/travel_direction`은 **자율주행(rebar_drive)만 발행**한다. 리모콘 수동
    #   주행에서는 아무도 안 쏘므로 그것만 보면 방향을 알 수 없다.
    #   → 실제 궤도 지령(`/drive_control`, 20Hz)에서 움직임과 방향을 뽑는다.
    ctx = {'dir': None, 'verdict': None, 'rebar_frac': None, 'odom_mm': None,
           'moving': False, 'motion': None, 'lr': None,
           'rod_n': None, 'rod_near_frac': None, 'rod_cam': None}
    state = {n: {'last_t': 0.0, 'last_small': None, 'saved': 0, 'seen': 0,
                 'skipped': 0} for n in names}
    depth_buf = {n: {'img': None, 't': 0.0} for n in depth_cams}
    stop = {'v': False, 'why': ''}
    meta_f = open(meta_path, 'a')

    def on_dir(m):
        ctx['dir'] = m.data

    def on_deck(m):
        try:
            d = json.loads(m.data)
        except (ValueError, TypeError):
            return
        ctx['verdict'] = d.get('verdict')
        ctx['rebar_frac'] = d.get('rebar_frac')
        ctx['rod_n'] = d.get('rod_n')
        ctx['rod_near_frac'] = d.get('rod_near_frac')
        ctx['rod_cam'] = d.get('cam')

    def on_odom(m):
        # 부호 규약은 rebar_drive와 동일: x_ctrl = -pose.position.x (전진=증가)
        ctx['odom_mm'] = round(-m.pose.position.x * 1000.0, 1)

    def on_drive(m):
        l, r = float(m.left_speed), float(m.right_speed)
        ctx['lr'] = [round(l, 1), round(r, 1)]
        ctx['moving'] = abs(l) > 1.0 or abs(r) > 1.0
        if not ctx['moving']:
            ctx['motion'] = 'stop'
        elif l * r < 0:
            ctx['motion'] = 'spin_left' if l < 0 else 'spin_right'
        elif (l + r) / 2 > 0:
            ctx['motion'] = 'forward' if abs(l - r) < 10 else 'fwd_turn'
        else:
            ctx['motion'] = 'backward' if abs(l - r) < 10 else 'rev_turn'

    def mk_depth(name):
        def cb(msg):
            try:
                d = br.imgmsg_to_cv2(msg, 'passthrough')
            except Exception:
                return
            depth_buf[name]['img'] = d
            depth_buf[name]['t'] = time.time()
        return cb

    def mk(name):
        def cb(msg):
            st = state[name]
            st['seen'] += 1
            now = time.time()
            if a.hz > 0 and now - st['last_t'] < 1.0 / a.hz:
                return
            if a.only_rods and not (ctx['rod_n'] or 0):
                st['skipped'] += 1
                return
            # depth를 쓰기로 했으면 **짝이 생긴 뒤부터** 담는다. 시작 직후
            # depth 콜백이 아직 안 와서 depth 없는 컬러가 섞이면, 나중에
            # "이 프레임만 왜 depth가 없지" 를 다시 따져야 한다.
            if name in depth_buf and depth_buf[name]['img'] is None:
                st['skipped'] += 1
                return
            try:
                img = br.compressed_imgmsg_to_cv2(msg, 'bgr8')
            except Exception:
                return
            # ── 유사 프레임 제거 ──
            small = cv2.resize(img, (64, 40)).astype(np.int16)
            if a.diff > 0 and st['last_small'] is not None:
                if float(np.abs(small - st['last_small']).mean()) < a.diff:
                    st['skipped'] += 1
                    return
            st['last_small'] = small
            st['last_t'] = now

            fn = f"{st['saved']:06d}_{time.strftime('%H%M%S')}.jpg"
            try:
                cv2.imwrite(os.path.join(root, name, fn), img,
                            [cv2.IMWRITE_JPEG_QUALITY, a.quality])
            except Exception as e:
                node.get_logger().error(f'저장 실패: {e}')
                return
            st['saved'] += 1
            dfile = ddt = None
            if name in depth_buf and depth_buf[name]['img'] is not None:
                dn = fn[:-4] + '.png'
                try:
                    # 16비트 PNG = 무손실. mm 값이 그대로 남아야 층 판별에 쓴다.
                    cv2.imwrite(os.path.join(root, name + '_depth', dn),
                                depth_buf[name]['img'])
                    dfile = f'{name}_depth/{dn}'
                    ddt = round((now - depth_buf[name]['t']) * 1000.0, 1)
                except Exception as e:
                    node.get_logger().error(f'depth 저장 실패: {e}')
            meta_f.write(json.dumps({
                't': round(now, 3), 'cam': name, 'file': f'{name}/{fn}',
                'depth_file': dfile, 'depth_dt_ms': ddt,
                'dir': ctx['dir'], 'verdict': ctx['verdict'],
                'rebar_frac': ctx['rebar_frac'], 'odom_mm': ctx['odom_mm'],
                'motion': ctx['motion'], 'lr': ctx['lr'],
                'rod_n': ctx['rod_n'], 'rod_near_frac': ctx['rod_near_frac'],
                'rod_cam': ctx['rod_cam'],
            }, ensure_ascii=False) + '\n')
            meta_f.flush()
        return cb

    for n in names:
        node.create_subscription(CompressedImage, CAMS[n][0], mk(n),
                                 qos_profile_sensor_data)
    for n in depth_cams:
        node.create_subscription(Image, DEPTH_TOPICS[n], mk_depth(n),
                                 qos_profile_sensor_data)
    node.create_subscription(String, '/travel_direction', on_dir, 10)
    node.create_subscription(String, '/deck_edge_status', on_deck, 10)
    node.create_subscription(PoseStamped, '/encoder_odom', on_odom, 10)
    node.create_subscription(DriveControl, '/drive_control', on_drive, 10)

    print(f'■ 현장 수집 시작 → {root}')
    print(f'  카메라 {len(names)}대: ' +
          ', '.join(f'{n}({CAMS[n][1]})' for n in names))
    print(f'  최대 {a.hz}Hz/대,  유사프레임 임계 {a.diff},  '
          f'디스크 여유 {a.min_free_gb}GB 미만이면 정지')
    if depth_cams:
        print(f"  depth 동시저장: {', '.join(depth_cams)} "
              f"(16비트 PNG, <카메라>_depth/)")

    # ⚠ 구독이 붙기 전에 저장하면 메타(odom/verdict/motion)가 전부 null이 된다.
    #   DDS 디스커버리에 1초 남짓 걸리므로 잠깐 돌려 맥락을 채우고 시작한다.
    warm = time.time()
    while time.time() - warm < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    for st in state.values():          # 워밍업 중 들어온 프레임은 저장하지 않았지만
        st['last_t'] = 0.0             # 시작 직후 첫 장은 바로 담는다
    print(f"  준비됨 (odom={ctx['odom_mm']}, 판정={ctx['verdict']})")
    print('  Ctrl+C 로 정지\n')

    t0 = time.time()
    last_report = t0
    try:
        while not stop['v']:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()
            if a.max_min > 0 and now - t0 > a.max_min * 60:
                stop['v'] = True; stop['why'] = f'{a.max_min}분 경과'
            if now - last_report >= 10.0:
                last_report = now
                free = shutil.disk_usage(root).free / 1e9
                if free < a.min_free_gb:
                    stop['v'] = True
                    stop['why'] = f'디스크 여유 {free:.1f}GB < {a.min_free_gb}GB'
                tot = sum(s['saved'] for s in state.values())
                det = '  '.join(f"{n}:{state[n]['saved']}" for n in names)
                rod = (f"  봉{ctx['rod_n']}"
                       f"({ctx['rod_near_frac']})" if ctx['rod_n'] else '')
                print(f"  [{(now-t0)/60:5.1f}분] 저장 {tot}장   {det}   "
                      f"여유 {free:.1f}GB   {ctx['motion'] or '-'}{rod}", flush=True)
    except KeyboardInterrupt:
        stop['why'] = '사용자 정지(Ctrl+C)'

    # ── 요약 ────────────────────────────────────────────
    meta_f.close()
    dur = time.time() - t0
    size = sum(os.path.getsize(os.path.join(dp, f))
               for dp, _, fs in os.walk(root) for f in fs) / 1e6
    summary = {
        'session': stamp, 'sec': round(dur, 1), 'reason': stop['why'],
        'cams': {n: {'saved': state[n]['saved'], 'seen': state[n]['seen'],
                     'skipped_similar': state[n]['skipped'],
                     'topic': CAMS[n][0]} for n in names},
        'size_mb': round(size, 1),
    }
    with open(os.path.join(root, 'summary.json'), 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    print(f"\n{'='*60}\n■ 종료 — {stop['why']}")
    print(f'  {dur/60:.1f}분,  {size:.0f}MB')
    print(f"\n{'카메라':<10}{'수신':>8}{'저장':>8}{'유사제외':>10}")
    for n in names:
        s = state[n]
        print(f"{n:<10}{s['seen']:>8}{s['saved']:>8}{s['skipped']:>10}")
    print(f'\n  저장 위치: {root}')
    got = [n for n in names if state[n]['saved']]
    if got:
        print(f'\n▶ Roboflow 업로드 (돌아와서 실행):')
        print(f'    export ROBOFLOW_API_KEY=xxxx')
        print(f'    python3 tools/vision_test/upload_to_roboflow.py \\')
        print(f'        --project <프로젝트ID> --batch field_{stamp} \\')
        print(f"        --dirs {' '.join(root + '/' + n for n in got)}")
        print(f'    (카메라를 나눠 올리려면 --dirs 를 하나씩 주고 --batch 도 나눌 것)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
