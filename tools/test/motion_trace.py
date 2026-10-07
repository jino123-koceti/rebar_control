#!/usr/bin/env python3
"""결속 동작의 **겹침과 끊김**을 측정한다.

사용자 요구(2026-10-04, 재확인 2026-10-07): "연결동작이나 동시에 움직일 수 있는
시퀀스들은 살짝씩 걸쳐서 동작하게 하자. 지금은 하나하나씩 움직여서 좀 멋이없다."

눈으로 "뚝뚝 끊긴다" 를 고치려면 **어디서 몇 초가 비는지** 숫자로 봐야 한다.
그래서 두 가지를 뽑는다:
  · **빈 구간** — 어느 축도 움직이지 않는 시간. 이게 끊김의 정체다
  · **겹침 구간** — 두 종류 이상이 동시에 움직인 시간 (yaw+XY, Z+XY)

⚠ `/stage/status` 는 20Hz 다 (2026-10-04 에 2Hz 였고, 그때는 겹침이 **일어나는데도
  로그에 안 보였다** — 상태 주기가 느려서 못 잡은 것이었다). 그보다 촘촘히 볼
  수는 없으므로 0.05초 미만의 겹침은 측정 한계 밖이다.

사용
  python3 motion_trace.py --out /tmp/trace.csv --sec 600
  python3 motion_trace.py --summary /tmp/trace.csv
"""

import argparse
import csv
import json
import sys
import time

AXES = ('x', 'y', 'z')


def run(path, seconds):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node('motion_trace')
    state = {'stage': None, 'seq': None, 'plan': None}
    for key, topic in (('stage', '/stage/status'), ('seq', '/tying/status'),
                       ('plan', '/plan/status')):
        node.create_subscription(
            String, topic,
            (lambda k: (lambda m: state.__setitem__(
                k, _safe(m.data))))(key), 10)
    f = open(path, 'a', newline='')
    w = csv.writer(f)
    w.writerow(['t_s', 'step', 'phase', 'moving', 'yaw_moving',
                'x_mm', 'y_mm', 'z_mm', 'detail'])
    f.flush()
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
            st, sq, pl = state['stage'], state['seq'], state['plan']
            if st is None:
                continue
            cur = st.get('current_mm') or {}
            w.writerow([f"{time.time() - t0:.3f}",
                        (sq or {}).get('step'), (pl or {}).get('phase'),
                        st.get('moving'), st.get('yaw_moving'),
                        cur.get('x'), cur.get('y'), cur.get('z'),
                        (sq or {}).get('detail', '')])
            f.flush()
    except KeyboardInterrupt:
        pass
    f.close()
    rclpy.shutdown()
    print(f"기록 → {path}")
    return 0


def _safe(s):
    try:
        return json.loads(s)
    except ValueError:
        return None


def _moved(a, b, tol=0.15):
    """두 표본 사이에 선형축이 실제로 움직였는가.

    `moving` 플래그만 믿지 않는다 — 목표를 접수했지만 아직 안 움직이는 구간이
    있고, 거기까지 '움직임' 으로 세면 빈 구간이 숨는다.
    """
    out = set()
    for k in AXES:
        va, vb = a.get(k), b.get(k)
        if va is None or vb is None:
            continue
        try:
            if abs(float(vb) - float(va)) > tol:
                out.add(k)
        except (TypeError, ValueError):
            continue
    return out


def summary(path):
    rows = list(csv.DictReader(open(path, newline='')))
    if len(rows) < 3:
        print("데이터가 부족합니다.")
        return 1
    # 표본마다 "무엇이 움직였나" 를 만든다
    marks = []
    for i in range(1, len(rows)):
        a, b = rows[i - 1], rows[i]
        mv = _moved(a, b)
        if str(b.get('yaw_moving')).lower() == 'true':
            mv.add('yaw')
        marks.append((float(b['t_s']), mv, b.get('step'), b.get('detail', '')))

    # 단계 타임라인
    print("=== 단계 타임라인")
    cur, t_in = None, marks[0][0]
    for t, _mv, step, _d in marks:
        if step != cur:
            if cur is not None:
                print(f"  {t_in:7.2f}s  {cur:<12s} {t - t_in:5.2f}초")
            cur, t_in = step, t
    print(f"  {t_in:7.2f}s  {str(cur):<12s} {marks[-1][0] - t_in:5.2f}초 (마지막)")

    # 빈 구간 — 아무것도 안 움직인 연속 구간
    print("\n=== 빈 구간 (아무 축도 안 움직임, 0.4초 이상)")
    gap0, total_gap = None, 0.0
    gaps = []
    for t, mv, step, detail in marks:
        if not mv:
            if gap0 is None:
                gap0 = (t, step, detail)
        else:
            if gap0 is not None:
                dt = t - gap0[0]
                total_gap += dt
                if dt >= 0.4:
                    gaps.append((gap0[0], dt, gap0[1], gap0[2]))
                gap0 = None
    for t, dt, step, detail in gaps:
        print(f"  {t:7.2f}s  {dt:5.2f}초  [{str(step):<11s}] {detail[:50]}")
    print(f"  → 0.4초 이상 빈 구간 {len(gaps)}개, 합계 "
          f"{sum(g[1] for g in gaps):.1f}초 / 전체 빈 시간 {total_gap:.1f}초")

    # 겹침 — 두 종류 이상 동시
    print("\n=== 겹침 구간 (동시에 움직인 조합)")
    combo = {}
    for t, mv, _s, _d in marks:
        if len(mv) >= 2:
            k = '+'.join(sorted(mv))
            combo[k] = combo.get(k, 0) + 1
    if not combo:
        print("  없음 — 모든 동작이 **직렬**로 돌았다 (끊김의 원인)")
    for k, n in sorted(combo.items(), key=lambda x: -x[1]):
        print(f"  {k:<14s} 표본 {n}개 ≈ {n * 0.05:.1f}초")
    span = marks[-1][0] - marks[0][0]
    print(f"\n전체 {span:.1f}초")
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out')
    p.add_argument('--sec', type=float, default=600.0)
    p.add_argument('--summary')
    a = p.parse_args()
    if a.summary:
        return summary(a.summary)
    if not a.out:
        print("--out 또는 --summary 가 필요합니다.")
        return 1
    return run(a.out, a.sec)


if __name__ == '__main__':
    sys.exit(main())
