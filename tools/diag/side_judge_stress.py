#!/usr/bin/env python3
"""테스트 A — 측면 판정만 반복해 프리즈를 재현/배제한다 (모션 없음).

## 왜 (2026-08-13)
자율주행 중 **두 번 모두 횡이동 단계에서 즉사**했다. 그 단계에만 있는 요인이 둘이다:
  ① 측면 카메라(특히 ZED X One)의 구독을 짧은 간격으로 붙였다 뗐다 한다
  ② 0x143 횡이동 모터가 돈다 (전류 서지 → 전원 강하 가능)
프리즈 프로브가 리셋 직전 1초까지 CPU·IO·메모리·온도 전부 정상으로 기록했으므로
자원 고갈은 아니다. 남은 건 ①과 ② — 이 스크립트는 **①만** 반복해 둘을 가른다.

  죽으면   → 카메라 스택(ZED X One / Argus / GMSL) 확정. 전원 무관.
  버티면   → ①은 무죄. 다음은 ②만 따로(0x143 회전만) 돌려본다.

## 안전
모터를 전혀 건드리지 않는다. `/deck_edge/side_request` 발행 + 응답 대기뿐이라
로봇은 완전히 정지해 있다. 프리즈가 재현되면 보드가 리셋될 뿐 기구 위험은 없다.

## 기록
매 시도를 **fsync** 하며 남긴다 — 프리즈는 하드 리셋이라 fsync 없이는 마지막 몇
분이 통째로 사라진다(1·2차 프리즈에서 ROS 로그·journald 모두 그렇게 잃었다).
재부팅 후 이 파일의 마지막 줄이 곧 '몇 번째 시도에서 죽었는지'가 된다.

사용:
    python3 tools/diag/side_judge_stress.py --side right --n 30 --gap 2
    python3 tools/diag/side_judge_stress.py --side both  --n 30    # 좌우 교대
"""
import argparse
import json
import os
import time

import rclpy
from std_msgs.msg import String

OUT = '/var/log/robot_control/side_stress.csv'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', default='right', choices=['right', 'left', 'both'])
    ap.add_argument('--n', type=int, default=30, help='반복 횟수')
    ap.add_argument('--gap', type=float, default=2.0, help='시도 간 간격(초)')
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()

    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    new = not os.path.exists(a.out) or os.path.getsize(a.out) == 0
    log = open(a.out, 'a')
    if new:
        log.write('time,up,iter,side,elapsed_s,result\n')

    rclpy.init()
    node = rclpy.create_node('side_judge_stress')
    pub = node.create_publisher(String, '/deck_edge/side_request', 10)
    box = {}
    node.create_subscription(
        String, '/deck_edge/side_status',
        lambda m: box.update(t=time.time(), d=m.data), 10)
    time.sleep(2.0)     # 디스커버리

    print(f'테스트 A: {a.side} 판정 {a.n}회 (간격 {a.gap}s) — 모션 없음')
    ok = fail = 0
    for i in range(1, a.n + 1):
        box.clear()
        m = String(); m.data = a.side
        pub.publish(m)
        t0 = time.time()
        while 't' not in box and time.time() - t0 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        el = time.time() - t0
        if 't' in box:
            r = json.loads(box['d'])
            res = ' '.join(
                f"{k}={v.get('rebar_frac', v.get('reason'))}"
                for k, v in r.items() if isinstance(v, dict))
            ok += 1
        else:
            res = 'TIMEOUT'
            fail += 1
        up = open('/proc/uptime').read().split()[0]
        log.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{float(up):.0f},"
                  f"{i},{a.side},{el:.2f},{res}\n")
        log.flush()
        os.fsync(log.fileno())        # ★ 리셋에서 살아남게
        print(f'  {i:3d}/{a.n}  {el:5.2f}s  {res}', flush=True)
        time.sleep(a.gap)

    print(f'\n완료: 성공 {ok} / 타임아웃 {fail}  → 기록 {a.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
