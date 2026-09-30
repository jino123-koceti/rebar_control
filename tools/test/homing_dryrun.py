#!/usr/bin/env python3
"""호밍 상태기계 무부하 검증 — 하드웨어 없이 돌린다.

가짜 리미트 신호를 발행하고 `/joint_N/speed` 명령을 받아 시퀀스가 맞는지 본다.
**모터도 EZIO 도 필요 없다.** 아키텍처 문서 §1.5 의 "하드웨어 없이 검증할 수단"에 해당한다.

왜 필요한가: 3차년도에서 호밍을 한 번도 돌려본 적이 없다. 실장비에서 바로 돌리면
방향 부호가 틀렸을 때 축이 반대 리미트로 달려간다. 상태 전이와 정지 조건을 먼저
여기서 확인한다.

**주의:** `ezi_io_node` 가 떠 있으면 진짜 리미트와 충돌한다. 반드시 끄고 돌릴 것.

사용:
    # 1) ezi_io_node 를 끈 상태에서
    ros2 run rmd_robot_control homing_node &
    python3 tools/test/homing_dryrun.py            # 정상 시나리오 (X축)
    python3 tools/test/homing_dryrun.py --case far     # 반대쪽 리미트에 닿는 경우
    python3 tools/test/homing_dryrun.py --case stale   # 리미트 신호가 끊긴 경우
    python3 tools/test/homing_dryrun.py --case athome   # 이미 원점에 있는 경우
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')


class DryRun(Node):
    def __init__(self, case, prefix=''):
        super().__init__('homing_dryrun')
        self.case = case
        self.prefix = prefix
        self.limit_pubs = {n: self.create_publisher(Bool, f'/limit_sensors/{n}', 10)
                           for n in LIMITS}
        self.cmd_pub = self.create_publisher(String, '/homing_cmd', 10)
        self.state = {n: False for n in LIMITS}
        self.speeds = []        # (시각, dps)
        self.status = []        # (시각, state, detail)
        self.create_subscription(Float32, f'{prefix}/joint_3/speed', self._on_speed, 10)
        self.create_subscription(String, '/homing_status', self._on_status, 10)
        self.t0 = time.time()
        self.publish_limits = True

    def _on_speed(self, msg):
        self.speeds.append((time.time() - self.t0, msg.data))

    def _on_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        rec = (round(time.time() - self.t0, 2), d.get('state'), d.get('detail'), d.get('refs'))
        if not self.status or self.status[-1][1:3] != rec[1:3]:
            self.status.append(rec)
            print(f"  [{rec[0]:5.2f}s] 상태={rec[1]:<9} {rec[2]}")

    def pump_limits(self):
        if not self.publish_limits:
            return
        for n, v in self.state.items():
            self.limit_pubs[n].publish(Bool(data=v))

    def last_speed(self):
        return self.speeds[-1][1] if self.speeds else None

    def stopped(self):
        """마지막 명령이 0 인가.

        ⚠ `abs(last_speed() or 1) < 0.01` 로 쓰면 안 된다 — 0.0 은 falsy 라서
        `or 1` 이 1 로 바꿔버리고, 정지했는데도 '안 멈췄다'로 판정한다(실제로 그랬다).
        """
        v = self.last_speed()
        return v is not None and abs(v) < 0.01


def spin(node, sec):
    end = time.time() + sec
    while rclpy.ok() and time.time() < end:
        node.pump_limits()
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--case', default='normal',
                    choices=['normal', 'far', 'stale', 'athome'])
    # 실제 모터로 명령이 가지 않게 호밍 노드를 remap 해서 띄우고, 같은 접두어를 준다:
    #   ros2 run rmd_robot_control homing_node --ros-args \
    #       -r /joint_3/speed:=/dry/joint_3/speed -r /joint_4/speed:=/dry/joint_4/speed \
    #       -r /joint_5/speed:=/dry/joint_5/speed -r /joint_6/speed:=/dry/joint_6/speed
    ap.add_argument('--prefix', default='', help="예: /dry (모터로 안 보내고 시험만)")
    a = ap.parse_args()

    rclpy.init()
    n = DryRun(a.case, a.prefix)
    ok = True
    print(f"■ 시나리오: {a.case}  (X축)")

    # 리미트 신호를 먼저 흘려 '신선한' 상태를 만든다
    spin(n, 1.5)

    if a.case == 'athome':
        n.state['x_min'] = True
        spin(n, 0.5)

    print("→ /homing_cmd 'x' 전송")
    n.cmd_pub.publish(String(data='x'))
    spin(n, 1.0)

    if a.case == 'stale':
        print("→ 리미트 발행을 끊는다 (센서 두절 흉내)")
        n.publish_limits = False
        spin(n, 2.5)
        states = [s[1] for s in n.status]
        ok = 'failed' in states and n.stopped()
        print(f"판정: {'통과' if ok else '실패'} — 센서 두절 시 정지·실패로 가야 한다")

    elif a.case == 'far':
        print("→ 반대쪽 리미트(x_max)를 눌러본다 (방향 부호가 틀린 상황)")
        n.state['x_max'] = True
        spin(n, 1.5)
        states = [s[1] for s in n.status]
        ok = 'failed' in states and n.stopped()
        print(f"판정: {'통과' if ok else '실패'} — 반대 리미트면 즉시 정지·실패로 가야 한다")

    else:
        # 탐색 방향 확인
        seek = n.last_speed()
        print(f"  탐색 속도: {seek} dps")
        if seek is None or abs(seek) < 1:
            print("판정: 실패 — 탐색 명령이 안 나왔다"); ok = False
        else:
            # 원점 리미트 도달 → 후퇴가 시작되는지 곧바로 확인한다.
            # (후퇴 제한시간 back_off_sec 이 지나기 전에 센서를 풀어줘야 한다 —
            #  실제 장비에서도 후퇴하면 바로 센서가 떨어진다)
            print("→ x_min 눌림")
            n.state['x_min'] = True
            spin(n, 0.3)
            back = n.last_speed()
            print(f"  후퇴 속도: {back} dps  (탐색과 부호가 반대여야 한다)")
            if back is None or seek * back >= 0:
                print("판정: 실패 — 후퇴 방향이 잘못됐다"); ok = False
            # 후퇴 중 센서 해제 → 후퇴 시간이 끝나면 정밀 재접근으로 간다
            print("→ x_min 해제 (후퇴 성공)")
            n.state['x_min'] = False
            spin(n, 1.2)
            fine = n.last_speed()
            print(f"  정밀 재접근 속도: {fine} dps  (탐색과 같은 부호, 더 느려야 한다)")
            if fine is None or seek * fine <= 0 or abs(fine) >= abs(seek):
                print("판정: 실패 — 정밀 재접근이 잘못됐다"); ok = False
            print("→ x_min 다시 눌림 (원점 확정)")
            n.state['x_min'] = True
            spin(n, 1.5)
            states = [s[1] for s in n.status]
            stop = n.stopped()
            if not (('done' in states) and stop):
                print(f"판정: 실패 — 완료로 가고 정지해야 한다 (상태 {states}, 마지막 {n.last_speed()})")
                ok = False
            else:
                print("판정: 통과 — 탐색 → 후퇴 → 정밀 → 완료, 마지막에 정지")

    print(f"\n■ 결과: {'통과' if ok else '실패'}   (상태 전이 {len(n.status)}회, 명령 {len(n.speeds)}건)")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
