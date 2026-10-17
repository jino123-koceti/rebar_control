#!/usr/bin/env python3
"""호밍 상태기계 무부하 검증 — 하드웨어 없이 돌린다.

가짜 리미트 신호를 발행하고 `/joint_N/speed` 명령을 받아 시퀀스가 맞는지 본다.
**모터도 EZIO 도 필요 없다.**

⚠⚠ [2026-10-17 전면 수정] **전 축(`all`) 으로 바꿨다.**
  예전에는 `'x'` 한 축만 보냈는데, 노드가 그걸 거부하도록 바뀌면서
  (`"호밍은 전 축('all')으로만 한다"`) **어떤 시나리오도 통과할 수 없게** 됐다.
  도구가 조용히 못 쓰게 된 것을 2026-10-17 에 발견했다.

⚠⚠ **반드시 실장비와 다른 ROS_DOMAIN_ID 로 돌릴 것.**
  같은 도메인에서 돌리면 진짜 리미트와 충돌하고, 가짜 센서 신호가 실제
  호밍 노드에 들어간다. 2026-10-17 에 실수로 도메인 33 에서 돌렸다.

사용:
    export ROS_DOMAIN_ID=77                      # 실장비(33)와 분리
    ros2 run rmd_robot_control homing_node &
    python3 tools/test/homing_dryrun.py                      # 전 축 정상
    python3 tools/test/homing_dryrun.py --case athome        # 전 축 센서 위에서 시작
    python3 tools/test/homing_dryrun.py --case yaw_athome    # yaw 만 센서 위 (B″ 경로)
    python3 tools/test/homing_dryrun.py --case far           # 반대쪽 리미트
    python3 tools/test/homing_dryrun.py --case stale         # 센서 두절
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')
# 축 → (명령 토픽 번호, 원점 리미트). axes.yaml 과 같아야 한다.
AX = {'z': (5, 'z_min'), 'x': (3, 'x_min'), 'yaw': (6, 'yaw_home'), 'y': (4, 'y_min')}
ORDER = ('z', 'x', 'yaw', 'y')


class DryRun(Node):
    def __init__(self, prefix='', yaw_gun=None):
        super().__init__('homing_dryrun')
        self.limit_pubs = {n: self.create_publisher(Bool, f'/limit_sensors/{n}', 10)
                           for n in LIMITS}
        self.cmd_pub = self.create_publisher(String, '/homing_cmd', 10)
        # yaw 전제 검사는 `/stage/status` 의 `gun_deg`(멀티턴 기준점)를 먼저 본다.
        # 무부하에서는 엔코더가 없으므로 여기서 흉내 내 준다.
        self.stage_pub = self.create_publisher(String, '/stage/status', 10)
        self.yaw_gun = yaw_gun
        self.state = {n: False for n in LIMITS}
        self.speed = {}          # 축 → 마지막 dps
        self.status = []         # (t, state, axis, detail)
        for a, (j, _) in AX.items():
            self.create_subscription(Float32, f'{prefix}/joint_{j}/speed',
                                     self._mk(a), 10)
        self.create_subscription(String, '/homing_status', self._on_status, 10)
        self.t0 = time.time()
        self.publish_limits = True

    def _mk(self, axis):
        def cb(msg):
            self.speed[axis] = msg.data
        return cb

    def _on_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        rec = (round(time.time() - self.t0, 2), d.get('state'), d.get('axis'),
               d.get('detail'))
        if not self.status or self.status[-1][1:4] != rec[1:4]:
            self.status.append(rec)
            print(f"  [{rec[0]:5.2f}s] {str(rec[1]):<9} {str(rec[2]):<4} {rec[3]}")

    def pump(self):
        if self.publish_limits:
            for n, v in self.state.items():
                self.limit_pubs[n].publish(Bool(data=v))
        if self.yaw_gun is not None:
            self.stage_pub.publish(String(data=json.dumps(
                {'gun_deg': self.yaw_gun, 'current_mm': {'x': 0.0, 'y': 0.0, 'z': 0.0}})))

    def phase(self):
        return self.status[-1][1] if self.status else None

    def axis(self):
        return self.status[-1][2] if self.status else None

    def states(self):
        return [s[1] for s in self.status]


def spin(n, sec):
    end = time.time() + sec
    while rclpy.ok() and time.time() < end:
        n.pump()
        rclpy.spin_once(n, timeout_sec=0.02)


def wait_for(n, pred, sec, what):
    """조건이 참이 될 때까지 돌린다. 되면 True."""
    end = time.time() + sec
    while rclpy.ok() and time.time() < end:
        n.pump()
        rclpy.spin_once(n, timeout_sec=0.02)
        if pred():
            return True
        if n.phase() == 'failed':
            return False
    print(f"    ⏱ 대기 초과: {what}")
    return False


def run_axis(n, axis, start_at_home, fail):
    """한 축의 탐색→후퇴→정밀→확정을 몰아준다. 성공 여부."""
    j, lim = AX[axis]
    if not wait_for(n, lambda: n.axis() == axis, 25, f"{axis} 차례"):
        return fail(f"{axis}: 차례가 오지 않았다 (상태 {n.phase()})")
    if start_at_home:
        # 차례가 오기 전에 눌러 둬야 하지만, ARM 중에 눌러도 BACK_OFF 로 간다
        n.state[lim] = True
    if not wait_for(n, lambda: n.phase() in ('seek', 'back_off'), 12,
                    f"{axis} seek/back_off"):
        return fail(f"{axis}: 탐색·후퇴로 가지 않았다")

    if n.phase() == 'seek':
        if not wait_for(n, lambda: abs(n.speed.get(axis) or 0) > 0.5, 6,
                        f"{axis} 탐색 속도"):
            return fail(f"{axis}: 탐색 명령이 안 나왔다")
        seek = n.speed[axis]
        print(f"    탐색 {seek:+.1f} dps → {lim} 누름")
        n.state[lim] = True
    else:
        seek = None
        print(f"    이미 {lim} → 후퇴부터")

    if not wait_for(n, lambda: n.phase() == 'back_off', 8, f"{axis} back_off"):
        return fail(f"{axis}: 후퇴로 가지 않았다")
    if not wait_for(n, lambda: abs(n.speed.get(axis) or 0) > 0.5, 4, f"{axis} 후퇴 속도"):
        return fail(f"{axis}: 후퇴 명령이 안 나왔다")
    back = n.speed[axis]
    if seek is not None and seek * back >= 0:
        return fail(f"{axis}: 후퇴 부호가 탐색과 같다 ({seek:+.1f} / {back:+.1f})")
    print(f"    후퇴 {back:+.1f} dps → {lim} 해제")
    n.state[lim] = False

    if not wait_for(n, lambda: n.phase() == 'fine', 8, f"{axis} fine"):
        return fail(f"{axis}: 정밀 재접근으로 가지 않았다")
    if not wait_for(n, lambda: abs(n.speed.get(axis) or 0) > 0.1, 4, f"{axis} 정밀 속도"):
        return fail(f"{axis}: 정밀 명령이 안 나왔다")
    fine = n.speed[axis]
    if fine * back >= 0:
        return fail(f"{axis}: 정밀 방향이 후퇴와 같다 ({back:+.1f} / {fine:+.1f})")
    if abs(fine) >= abs(back) * 3:
        print(f"    ⚠ 정밀 {fine:+.1f} dps 가 후퇴 {back:+.1f} 보다 많이 빠르다")
    print(f"    정밀 {fine:+.1f} dps → {lim} 다시 누름 (원점 확정)")
    n.state[lim] = True
    spin(n, 1.2)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--case', default='normal',
                    choices=['normal', 'far', 'stale', 'athome', 'yaw_athome'])
    ap.add_argument('--prefix', default='', help="예: /dry (모터로 안 보내고 시험만)")
    ap.add_argument('--yaw-gun', type=float, default=-16.6,
                    help='가짜 멀티턴 기준점(건 각도°). yaw 전제 검사에 쓰인다')
    a = ap.parse_args()

    rclpy.init()
    n = DryRun(a.prefix, yaw_gun=a.yaw_gun)
    bad = []

    def fail(msg):
        print(f"    ✘ {msg}")
        bad.append(msg)
        return False

    print(f"■ 시나리오: {a.case}  (전 축 {'→'.join(ORDER)}, yaw 기준점 건 {a.yaw_gun:+.1f}°)")
    spin(n, 1.5)            # 리미트·상태를 '신선하게' 만든다

    if a.case in ('athome',):
        for ax in ORDER:
            n.state[AX[ax][1]] = True
    if a.case == 'yaw_athome':
        n.state['yaw_home'] = True
    spin(n, 0.5)

    print("→ /homing_cmd 'all' 전송")
    n.cmd_pub.publish(String(data='all'))
    spin(n, 1.0)

    if a.case == 'stale':
        print("→ 리미트 발행을 끊는다 (센서 두절 흉내)")
        n.publish_limits = False
        spin(n, 3.0)
        if not ('failed' in n.states()):
            fail("센서가 두절됐는데 실패로 가지 않았다")
    elif a.case == 'far':
        wait_for(n, lambda: n.axis() == 'z' and n.phase() == 'seek', 10, 'z 탐색')
        print("→ 반대쪽 리미트(z_max)를 누른다 (방향 부호가 틀린 상황)")
        n.state['z_max'] = True
        spin(n, 2.0)
        if not ('failed' in n.states()):
            fail("반대 리미트를 눌렀는데 실패로 가지 않았다")
    else:
        at_home = (a.case == 'athome')
        for ax in ORDER:
            print(f"  ── {ax} ──")
            start = at_home or (a.case == 'yaw_athome' and ax == 'yaw')
            if not run_axis(n, ax, start, fail):
                break
        spin(n, 3.0)
        if 'failed' in n.states():
            fail(f"실패 상태로 끝났다 — {n.status[-1][3]}")

    print(f"\n■ 결과: {'통과' if not bad else '실패'}   "
          f"(상태 전이 {len(n.status)}회)")
    for m in bad:
        print(f"   · {m}")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if not bad else 1


if __name__ == '__main__':
    sys.exit(main())
