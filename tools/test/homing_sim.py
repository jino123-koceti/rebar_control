#!/usr/bin/env python3
"""호밍 전 시퀀스 모사 검증 — 하드웨어 없이 Z→Yaw→X→Y→READY 를 끝까지 돌린다.

## 왜 필요한가

실장비에서 바로 돌리면 **방향 부호가 틀렸을 때 축이 반대 리미트나 기계 끝단으로
달린다** (2026-09-30 에 yaw 가 그렇게 10.7A/86°C 까지 갔다). 기존
`homing_dryrun.py` 는 리미트를 사람이 토글하는 방식이라 단계 전이만 봤다.
여기서는 **축 위치를 모사**해서 리미트가 위치에 따라 켜지게 하고, 다음을 확인한다:

  · 축 순서가 z → yaw → x → y 인가
  · 각 축이 **홈 쪽으로** 가는가 (부호가 반대면 리미트에 영영 못 닿는다)
  · yaw 자세 판별이 맞는 탐색 방향을 고르는가 (자세별로 ± 두 가지다)
  · yaw 탐색 거리 상한이 **방향을 뒤집지 않고 멈추는가**
  · BACK_OFF → FINE 재접근이 도는가
  · 준비자세(Y 중앙 → yaw 1번)가 순서대로 가는가

## 부호 규약

`/motor_*_position` 토픽은 counts 부호와 반대다(토픽 = −counts/728.18).
즉 **명령 부호가 양수면 토픽은 감소한다.** 모사도 그렇게 적분한다.

## 쓰는 법

    # ezi_io_node 가 떠 있으면 진짜 리미트와 충돌한다 — 끄고 돌릴 것.
    # 모터로 명령이 가지 않게 remap 해서 띄운다:
    ros2 run rmd_robot_control homing_node --ros-args \
      -r /joint_3/speed:=/dry/joint_3/speed -r /joint_4/speed:=/dry/joint_4/speed \
      -r /joint_5/speed:=/dry/joint_5/speed -r /joint_6/speed:=/dry/joint_6/speed \
      -r /joint_3/position:=/dry/joint_3/position \
      -r /joint_4/position:=/dry/joint_4/position \
      -r /joint_5/position:=/dry/joint_5/position \
      -r /joint_6/position:=/dry/joint_6/position &
    python3 tools/test/homing_sim.py --prefix /dry --pose 3
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String

CPR, GEAR = 262144, 12.5
CPD = CPR / 360.0                     # counts / 모터축 1도
CPG = CPD * GEAR                      # counts / 건 1도
LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')

# 축 → (joint 번호, CAN ID 문자열, 홈 리미트, 먼 리미트)
AX = {
    'x':   (3, '0x145', 'x_min', 'x_max'),
    'y':   (4, '0x146', 'y_min', 'y_max'),
    'z':   (5, '0x147', 'z_min', 'z_max'),
    'yaw': (6, '0x148', 'yaw_home', None),
}
# 자세별 단회전값 (axes.yaml 과 같은 계산)
NOON = 4055
POSE_GUN = {0: 0.0, 1: -16.52, 2: -5.46, 3: 6.81, 4: 18.80}
POSE_SINGLE = {n: int(round((NOON + g * CPG) % CPR)) for n, g in POSE_GUN.items()}
# yaw_home 감지 구간 (단회전). 2026-10-03 실측: 감소 접근 시 225014 에서 켜짐,
# 폭 18857 counts(건 2.07°).
# 2026-10-03 모터 구동 재측정: 증가 접근 ON 213898 / 감소 접근 ON 218011.
# ON 구간은 그 둘을 포함하는 범위다 (꺼짐은 203175 / 224520 에서 관측).
YAW_ZONE = (205000, 224000)
# **yaw 기계적 끝단 (12시 기준 건 각도).** 결속건 간섭으로 한 바퀴를 못 돈다.
# 끝단이 없으면 부호가 틀려도 반대쪽으로 돌아 감지 구간에 들어가 버려서,
# 보호가 걸리는지 시험할 수 없다 (실제로 그렇게 헛통과했다).
# 자세 범위는 건 -16.52~+18.80° 이고 그 바로 밖에서 막힌다.
YAW_END_GUN = (-20.0, 22.0)

# X·Y·Z 모사: 홈 리미트가 켜지는 토픽 각도 구간과 반대쪽 리미트 위치.
# 명령 + 는 토픽 감소 → 홈을 **작은 쪽**에 둔다 (x/y 는 dir=+1).
# z 는 dir=-1 이므로 홈을 **큰 쪽**에 둔다.
# 시험을 빠르게 하려고 거리를 짧게 잡았다. 실제 스트로크(X 4.34바퀴 등)가 아니다 —
# 여기서 보는 것은 **부호와 전이**이고, 거리는 시간만 늘린다.
LIN = {
    'x':   dict(start=200.0, home=(0.0, 40.0),   far=900.0),
    'y':   dict(start=200.0, home=(0.0, 40.0),   far=900.0),
    'z':   dict(start=200.0, home=(300.0, 340.0), far=0.0),
}


class Sim(Node):
    def __init__(self, prefix, pose, bad_dir, dead_sensor=False):
        super().__init__('homing_sim')
        self.prefix, self.bad_dir = prefix, bad_dir
        # 센서 고장 흉내 — 축은 움직이는데 리미트가 영영 안 켜진다.
        # 이때는 스톨이 안 걸리므로 **탐색 거리 상한**만이 막는다.
        self.dead_sensor = dead_sensor
        # **접두어 아래로 발행한다.** ezi_io_node 가 떠 있으면 진짜 리미트와
        # 충돌하므로, homing_node 를 remap 해서 이쪽을 보게 만든다.
        self.limit_pubs = {n: self.create_publisher(
            Bool, f'{prefix}/limit_sensors/{n}', 10) for n in LIMITS}
        self.cmd_pub = self.create_publisher(String, '/homing_cmd', 10)
        # 권한 중재기도 격리한다. 여기서 보는 것은 **호밍 시퀀스 논리**이고,
        # 진짜 중재기는 비상정지·소유권 같은 다른 조건으로 거절할 수 있다
        # (2026-10-03: estop 이 걸려 있어 manual 고정이었다 — 중재기가 맞게 동작한 것).
        self.mode_pub = self.create_publisher(String, f'{prefix}/control_mode', 10)
        self.single_pubs, self.brake_pubs, self.pos_pubs = {}, {}, {}
        self.cmd = {}                 # 축 → 마지막 속도 명령
        self.goal = {}                # 축 → (목표각, 속도) 위치 명령
        self.pos = {}                 # 축 → 모사 토픽 각도
        for a, (j, mid, _, _) in AX.items():
            self.single_pubs[a] = self.create_publisher(
                Int32, f'{prefix}/motor_{mid}/encoder_single', 10)
            self.brake_pubs[a] = self.create_publisher(
                Bool, f'{prefix}/motor_{mid}/brake', 10)
            self.pos_pubs[a] = self.create_publisher(
                Float32, f'{prefix}/motor_{mid}_position', 10)
            self.create_subscription(Float32, f'{prefix}/joint_{j}/speed',
                                     (lambda k: (lambda m: self.cmd.__setitem__(k, m.data)))(a), 10)
            self.create_subscription(Float64MultiArray, f'{prefix}/joint_{j}/position',
                                     (lambda k: (lambda m: self.goal.__setitem__(
                                         k, tuple(m.data))))(a), 10)
            self.cmd[a] = 0.0
        for a in LIN:
            self.pos[a] = LIN[a]['start']
        # yaw: 자세에서 출발. **건 각도를 상태로 들고** 끝단에서 멈춘다.
        # counts·토픽은 거기서 파생한다 (counts 는 감싸지 않는다 — 0x92 는 멀티턴).
        self.yaw_gun = float(POSE_GUN[pose])
        self.yaw_counts = NOON + self.yaw_gun * CPG
        self.pos['yaw'] = -self.yaw_counts / CPD
        self.create_subscription(String, '/homing_status', self._on_status, 10)
        self.status, self.seen_axes = [], []
        self.t0 = time.time()
        self.last = time.time()

    # ---- 호밍 노드 상태 ----------------------------------------------------
    def _on_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        rec = (round(time.time() - self.t0, 2), d.get('state'), d.get('axis'),
               d.get('detail'))
        if not self.status or self.status[-1][1:3] != rec[1:3]:
            self.status.append(rec)
            ax = rec[2]
            if ax and (not self.seen_axes or self.seen_axes[-1] != ax):
                self.seen_axes.append(ax)
            print(f"  [{rec[0]:6.2f}s] {str(rec[1]):<9} {str(ax):<4} {rec[3]}")

    # ---- 모사 ------------------------------------------------------------
    def step(self):
        now = time.time()
        dt = min(0.1, now - self.last)
        self.last = now
        for a in AX:
            v = float(self.cmd.get(a) or 0.0)
            if self.bad_dir == a:
                v = -v                       # 부호를 일부러 뒤집어 본다
            g = self.goal.get(a)
            if g and abs(v) < 0.01:
                # 위치 명령 — 목표로 수렴시킨다. **yaw 도 똑같이 처리해야 한다**
                # (처음엔 yaw 만 빠뜨려 준비자세가 영영 안 끝났다).
                tgt = float(g[0])
                spd = float(g[1]) if len(g) > 1 else 60.0
                d = tgt - self.pos[a]
                dp = max(-spd * dt, min(spd * dt, d))
            else:
                dp = -v * dt                 # 명령 + → 토픽 감소
            if a == 'yaw':
                # 토픽이 줄면 counts 가 늘고 건 각도도 는다.
                want = self.yaw_gun + (-dp) * CPD / CPG
                self.yaw_gun = (want if self.dead_sensor      # 끝단 없는 상황
                                else max(YAW_END_GUN[0], min(YAW_END_GUN[1], want)))
                self.yaw_counts = NOON + self.yaw_gun * CPG
                self.pos['yaw'] = -self.yaw_counts / CPD
            else:
                self.pos[a] += dp
        self.publish()

    def publish(self):
        st = {n: False for n in LIMITS}
        for a, cfgs in LIN.items():
            lo, hi = cfgs['home']
            st[AX[a][2]] = lo <= self.pos[a] <= hi
            far = AX[a][3]
            if far:
                st[far] = (self.pos[a] >= cfgs['far'] if cfgs['far'] > lo
                           else self.pos[a] <= cfgs['far'])
        s = int(self.yaw_counts) % CPR
        st['yaw_home'] = (not self.dead_sensor) and YAW_ZONE[0] <= s <= YAW_ZONE[1]
        self.mode_pub.publish(String(data='{"mode": "homing", "owner": "homing_node"}'))
        for n, v in st.items():
            self.limit_pubs[n].publish(Bool(data=v))
        for a in AX:
            self.brake_pubs[a].publish(Bool(data=True))     # 즉시 해제됐다고 본다
            self.pos_pubs[a].publish(Float32(data=float(self.pos[a])))
            self.single_pubs[a].publish(Int32(data=(
                s if a == 'yaw' else int(abs(self.pos[a]) * CPD) % CPR)))


def spin(n, sec):
    end = time.time() + sec
    while rclpy.ok() and time.time() < end:
        n.step()
        rclpy.spin_once(n, timeout_sec=0.01)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prefix', default='/dry')
    ap.add_argument('--pose', type=int, default=3, choices=[0, 1, 2, 3, 4],
                    help='yaw 시작 자세 (0=12시)')
    ap.add_argument('--cmd', default='all')
    ap.add_argument('--bad-dir', default=None, choices=list(AX),
                    help='그 축의 부호를 뒤집어 상한·보호가 잡는지 본다')
    ap.add_argument('--dead-sensor', action='store_true',
                    help='리미트가 영영 안 켜지는 경우 — 거리 상한만이 막는다')
    ap.add_argument('--sec', type=float, default=60.0)
    a = ap.parse_args()

    rclpy.init()
    n = Sim(a.prefix, a.pose, a.bad_dir, a.dead_sensor)
    print(f"■ yaw 시작 자세 {a.pose}번 (단회전 {POSE_SINGLE[a.pose]}), "
          f"명령 '{a.cmd}'" + (f", 부호 뒤집기: {a.bad_dir}" if a.bad_dir else ""))
    spin(n, 2.0)                      # 토픽을 '신선하게' 만든다
    # **구독자가 붙기를 기다린다.** 한 번만 쏘면 discovery 전이라 유실된다
    # (2026-10-03 에 그래서 노드가 명령을 못 받고 idle 에 머물렀다).
    for _ in range(100):
        if n.cmd_pub.get_subscription_count() > 0:
            break
        spin(n, 0.1)
    else:
        print("✗ /homing_cmd 를 듣는 노드가 없다 — homing_node 가 떠 있는지 확인")
        n.destroy_node(); rclpy.shutdown(); return 1
    print(f"→ /homing_cmd '{a.cmd}'")
    n.cmd_pub.publish(String(data=a.cmd))
    spin(n, a.sec)

    states = [s[1] for s in n.status]
    print()
    print(f"축 진행 순서: {' → '.join(n.seen_axes)}")
    ok = True
    if a.dead_sensor:
        hit = any('상한' in str(x[3]) for x in n.status)
        if hit:
            print("✓ 탐색 거리 상한이 막았다")
        else:
            print(f"✗ 거리 상한이 안 걸렸다 (마지막 상태 {states[-1] if states else '없음'})")
            ok = False
    elif a.bad_dir:
        if 'failed' not in states:
            print("✗ 부호가 틀렸는데 실패하지 않았다 — 보호가 안 걸렸다")
            ok = False
        else:
            print("✓ 부호 오류를 잡아 실패 처리했다")
    else:
        want = ['z', 'yaw', 'x', 'y']
        got = [x for x in n.seen_axes if x in want]
        if got[:4] != want:
            print(f"✗ 축 순서가 다르다 — 기대 {want}, 실제 {got[:4]}")
            ok = False
        if 'done' not in states:
            print(f"✗ 완료에 도달하지 못했다 (마지막 상태 {states[-1] if states else '없음'})")
            ok = False
        if 'ready' not in states:
            print("✗ 준비자세(READY) 단계를 거치지 않았다")
            ok = False
        if ok:
            print("✓ z→yaw→x→y → 준비자세 → 완료")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
