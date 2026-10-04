#!/usr/bin/env python3
"""`tying_sequence` 를 가짜 `stage_node` 로 검증한다 (하드웨어 없이).

## 왜 필요한가

이 시퀀스의 실패는 **프레임 충돌**이다 — 실장비에서 처음 돌려 보며 찾을 종류가
아니다. 특히 "회전 전에 XY 를 후퇴시킨다" 를 빠뜨리거나 순서를 뒤집으면,
회전 도중에 중간 자세의 간섭 범위를 넘는다.

## 가짜 stage_node 가 하는 일

실물과 **같은 거부 규칙**을 쓴다 (`envelope_violation`, `transit_window`). 그래서
시퀀서가 순서를 틀리면 여기서 거부로 드러난다 — 통과했다면 실장비에서도 그
순서가 안전하다는 뜻이다. 축은 목표까지 선형으로 "순간이동" 하지 않고 단계적으로
움직여, 이동 중 상태(`moving`)도 재현한다.

⚠ 재현하지 않는 것: 정지마찰·전류·브레이크 타이밍·기구 스톨. 그건 실장비 몫이다.

## 격리

`ROS_DOMAIN_ID` 를 따로 쓴다 (기본 77). 실장비(33)와 섞이면 가짜 상태가 진짜
`tying_sequence` 에 들어가므로 **반드시** 분리한다.

사용:
    ROS_DOMAIN_ID=77 ros2 run rmd_robot_control tying_sequence &
    ROS_DOMAIN_ID=77 python3 tools/test/tying_sim.py --pose 1 --x 400 --y 100 \
        --goal-x 100 --goal-y 300
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String

from rmd_robot_control.axis_config import (envelope_violation, load_envelope,
                                           load_pose_id, pose_label,
                                           transit_window)

STEP_MM, STEP_GUN = 12.0, 1.2      # tick 당 이동량 (0.1s tick)
# ⚠ Z 는 `tick` 의 축 루프가 그대로 처리한다 (getattr/setattr 로 축 이름을
#   쓴다). 'z' 를 목표에 넣기만 하면 X·Y 와 같은 속도로 움직인다.


class FakeStage(Node):
    def __init__(self, pose, x, y, z=0.0):
        super().__init__('fake_stage')
        self.env = load_envelope()
        self.info = load_pose_id('yaw')
        self.x, self.y, self.z = x, y, z
        self.pose = pose                  # 정수 자세 또는 None(자세 사이)
        # 자세 사이는 1번과 2번의 중간으로 둔다 — 실물도 그 구간은 후보가 둘이라
        # 판별을 거부한다
        self.gun = (self.info['poses'][pose]['gun'] if pose is not None
                    else (self.info['poses'][1]['gun']
                          + self.info['poses'][2]['gun']) / 2)
        self.tgt = {}                     # 축 → mm
        self.yaw_tgt = None
        self.detail = '대기'
        self.rejects = 0                  # 실물과 같다 — 상위가 이걸로 가린다
        self.log = []

        self.st = self.create_publisher(String, '/stage/status', 10)
        self.create_subscription(Point, '/stage/goal', self._on_goal, 10)
        self.create_subscription(Int32, '/stage/yaw_pose', self._on_yaw, 10)
        self.create_subscription(Empty, '/stage/stop', self._on_stop, 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(0.1, self.pub)

    # ---- 실물과 같은 거부 규칙 -------------------------------------------
    def _reject(self, why):
        self.rejects += 1
        self.detail = f'거부: {why}'
        self.log.append(('거부', why))
        print(f"    [가짜 stage] 거부 — {why}")

    def _busy(self):
        return bool(self.tgt) or self.yaw_tgt is not None

    def _on_goal(self, m):
        want = {k: v for k, v in (('x', m.x), ('y', m.y), ('z', m.z))
                if not math.isnan(v)}
        if not want:
            return self._reject('목표가 비어 있다')
        if self._busy():                 # 실물과 같다 — 한 번에 한 동작
            return self._reject('이미 이동 중이다')
        # 작업영역 검사는 X·Y 만이다 — 실물도 Z 는 자세별 실측이 없다
        bad = envelope_violation(self.env, self.pose,
                                 {k: v for k, v in want.items() if k != 'z'})
        if bad:
            return self._reject(bad)
        self.tgt = want
        self.detail = '이동'
        self.log.append(('이동', dict(want)))
        print(f"    [가짜 stage] 이동 수락 {', '.join(f'{k}={v:.1f}' for k, v in sorted(want.items()))}")

    def _on_yaw(self, m):
        want = int(m.data)
        if self._busy():                 # 실물과 같다 — 한 번에 한 동작
            return self._reject('이미 이동 중이다')
        if self.pose is None:
            return self._reject('현재 yaw 자세를 못 가린다')
        if want == self.pose:
            return
        win, passed = transit_window(self.env, self.info, self.pose, want)
        for ax, v in (('x', self.x), ('y', self.y)):
            if not (win[ax][0] <= v <= win[ax][1]):
                names = ', '.join(passed)
                return self._reject(
                    f'{pose_label(self.pose)}→{pose_label(want)} 회전은 [{names}] 를 '
                    f'지난다 — 지금 {ax}={v:.1f}mm 가 그 교집합 '
                    f'{win[ax][0]:.1f}~{win[ax][1]:.1f}mm 밖이다')
        self.yaw_tgt = want
        self.detail = '회전'
        self.log.append(('회전', want))
        print(f"    [가짜 stage] 회전 수락 → {pose_label(want)}")

    def _on_stop(self, m):
        self.tgt, self.yaw_tgt = {}, None
        self.detail = '정지'

    # ---- 축 거동 ---------------------------------------------------------
    def tick(self):
        for ax in list(self.tgt):
            cur = getattr(self, ax)
            d = self.tgt[ax] - cur
            if abs(d) <= 0.5:
                setattr(self, ax, self.tgt.pop(ax))
            else:
                setattr(self, ax, cur + math.copysign(min(STEP_MM, abs(d)), d))
        if self.yaw_tgt is not None:
            want_gun = self.info['poses'][self.yaw_tgt]['gun']
            d = want_gun - self.gun
            if abs(d) <= 0.05:
                self.gun, self.pose, self.yaw_tgt = want_gun, self.yaw_tgt, None
            else:
                self.gun += math.copysign(min(STEP_GUN, abs(d)), d)
                self.pose = None          # 자세 사이 — 실물도 판별이 안 된다
        if not self.tgt and self.yaw_tgt is None and self.detail in ('이동', '회전'):
            self.detail = '목표 도달'

    def pub(self):
        self.st.publish(String(data=json.dumps({
            'moving': bool(self.tgt) or self.yaw_tgt is not None,
            'current_mm': {'x': round(self.x, 2), 'y': round(self.y, 2),
                           'z': round(self.z, 2)},
            'pose': self.pose,
            'pose_detail': (pose_label(self.pose) if self.pose is not None
                            else f'자세 사이 (건 {self.gun:+.2f}°)'),
            'detail': self.detail,
            'homed': ['x', 'y', 'z'],
            'rejects': self.rejects,
        }, ensure_ascii=False)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', type=int, default=1,
                    help='시작 자세. -1 이면 **자세 사이**(판별 불가)로 시작한다')
    ap.add_argument('--x', type=float, default=400.0)
    ap.add_argument('--y', type=float, default=100.0)
    ap.add_argument('--z', type=float, default=0.0, help='시작 Z mm')
    ap.add_argument('--goal-x', type=float, required=True)
    ap.add_argument('--goal-y', type=float, required=True)
    ap.add_argument('--goal-z', type=float, default=float('nan'),
                    help='결속 깊이 mm. 주면 Z 하강·결속건·Z 상승까지 돈다')
    ap.add_argument('--goal-pose', type=int, default=0,
                    help='자세를 지정한다 (0=지정 없음 → 사분면 규칙)')
    ap.add_argument('--timeout', type=float, default=60.0)
    a = ap.parse_args()

    rclpy.init()
    fake = FakeStage(None if a.pose < 0 else a.pose, a.x, a.y, a.z)
    seen = []
    fake.create_subscription(
        String, '/tying/status',
        lambda m: seen.append(json.loads(m.data)), 10)
    gp = fake.create_publisher(Point, '/tying/goal', 10)
    pp = fake.create_publisher(Int32, '/tying/goal_pose', 10)

    def pump(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(fake, timeout_sec=0.02)

    pump(2.0)
    print(f"시작  {'자세 사이' if a.pose < 0 else pose_label(a.pose)}"
          f"  X {a.x:.1f}  Y {a.y:.1f}")
    print(f"목표  X {a.goal_x:.1f}  Y {a.goal_y:.1f}"
          + ('' if math.isnan(a.goal_z) else f"  Z {a.goal_z:.1f}")
          + ('' if a.goal_pose <= 0 else f"  자세 {a.goal_pose} 지정") + "\n")
    if a.goal_pose > 0:
        pp.publish(Int32(data=a.goal_pose))
        pump(0.3)                     # 자세 지정이 목표보다 **먼저** 가야 한다
    gp.publish(Point(x=a.goal_x, y=a.goal_y, z=a.goal_z))

    last, t0 = None, time.time()
    while time.time() - t0 < a.timeout:
        pump(0.1)
        if not seen:
            continue
        s = seen[-1]
        key = (s['step'], s['detail'])
        if key != last:
            last = key
            print(f"  {time.time()-t0:5.1f}s  [{s['step']:<8}] "
                  f"X {s['current_mm']['x'] or 0:6.1f}  "
                  f"Y {s['current_mm']['y'] or 0:6.1f}  "
                  f"Z {s['current_mm'].get('z') or 0:6.1f}  "
                  f"자세 {str(s['pose_now']):<5} → {str(s['pose_want']):<5} "
                  f"{s['detail']}")
        if s['step'] in ('done', 'failed'):
            break

    ok = bool(seen) and seen[-1]['step'] == 'done'
    print(f"\n결과: {'성공' if ok else '실패'}")
    print(f"가짜 stage 가 받은 명령 순서:")
    for kind, v in fake.log:
        print(f"  {kind:<4} {v}")
    bad = [w for k, w in fake.log if k == '거부']
    if bad:
        print(f"\n⚠ 거부 {len(bad)}건 — 시퀀서가 순서를 틀렸다:")
        for w in bad:
            print(f"    {w}")
    rclpy.shutdown()
    return 0 if (ok and not bad) else 1


if __name__ == '__main__':
    sys.exit(main())
