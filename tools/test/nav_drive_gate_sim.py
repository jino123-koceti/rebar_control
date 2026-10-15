#!/usr/bin/env python3
"""`navigator` 가 **Z 가 내려간 채 주행을 시작하는 경우가 있는가** 를 전수로 본다.

주행이 시작되는 경로는 둘이고 둘 다 Z 검사가 있다고 코드에 적혀 있다. 그런데
그 검사가 실제로 모든 조합에서 동작하는지는 돌려 봐야 안다. 2026-10-12 에
`park_ok=False` 경로가 Z 를 안 보고 주행을 시작해 사람이 e-stop 으로 세운 적이 있다.

가짜 `tying_planner` + 가짜 `drive_node` 로 조합을 바꿔 가며 `/drive/step` 이
나가는지만 본다. 실장비와 분리하려고 ROS_DOMAIN_ID 를 따로 쓴다.
"""
import argparse, json, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Empty


class Fake(Node):
    def __init__(self, phase_seq, z_mm, park_ok):
        super().__init__('nav_drive_gate_sim')
        self.seq, self.z_mm, self.park_ok = phase_seq, z_mm, park_ok
        self.i = 0
        self.t0 = None
        self.drive_cmds = []
        self.mission = {}
        self.plan_p = self.create_publisher(String, '/plan/status', 10)
        self.drv_p = self.create_publisher(String, '/drive/status', 10)
        self.create_subscription(Float32, '/drive/step', self._drive, 10)
        self.create_subscription(Empty, '/plan/start', self._start, 10)
        self.create_subscription(String, '/mission/status', self._mis, 10)
        self.create_timer(0.1, self.pub)

    def _drive(self, m): self.drive_cmds.append(float(m.data))
    def _start(self, m): self.t0 = time.time()
    def _mis(self, m):
        try: self.mission = json.loads(m.data)
        except ValueError: pass

    def pub(self):
        ph = 'idle'
        if self.t0 is not None:
            # 1.2초마다 다음 phase 로 넘어간다
            k = min(int((time.time() - self.t0) / 1.2), len(self.seq) - 1)
            ph = self.seq[k]
        d = {'phase': ph, 'park_ok': self.park_ok, 'detail': '시뮬'}
        if self.z_mm is not None:
            d['z_mm'] = self.z_mm
        self.plan_p.publish(String(data=json.dumps(d, ensure_ascii=False)))
        self.drv_p.publish(String(data=json.dumps(
            {'moving': False, 'rejects': 0, 'left_mm': 0.0})))


def run(label, phase_seq, z_mm, park_ok, expect):
    rclpy.init()
    n = Fake(phase_seq, z_mm, park_ok)
    start = n.create_publisher(Empty, '/mission/start', 10)
    def pump(s):
        t = time.time()
        while time.time() - t < s:
            rclpy.spin_once(n, timeout_sec=0.02)
    pump(2.0)
    start.publish(Empty())
    pump(9.0)
    drove = bool(n.drive_cmds)
    ok = (drove == (expect == 'drive'))
    zt = '미확인' if z_mm is None else f'{z_mm:+.1f}mm'
    print(f"  {'✅' if ok else '❌'} {label}")
    print(f"      Z {zt} · park_ok {park_ok} · phase {'→'.join(phase_seq)}")
    print(f"      주행명령 {'나감 ' + str(n.drive_cmds) if drove else '안 나감'}"
          f"   기대 {expect}   미션 {n.mission.get('phase')}")
    if not ok:
        print(f"      ⚠ detail: {n.mission.get('detail')}")
    rclpy.shutdown()
    return ok


if __name__ == '__main__':
    DONE = ['ready', 'run', 'done']          # 복귀까지 끝난 정상 종료
    PARK = ['ready', 'run', 'park', 'park']  # 복귀 중 (중첩 경로가 타는 상태)
    cases = [
        # (설명, phase 흐름, Z, park_ok, 기대)
        ("복귀완료 + Z 올라옴",            DONE, 0.0,   True,  'drive'),
        ("복귀완료 + Z 내려감 (-30)",      DONE, -30.0, True,  'abort'),
        ("복귀실패 + Z 내려감 (-30)",      DONE, -30.0, False, 'abort'),
        ("복귀실패 + Z 올라옴",            DONE, 0.0,   False, 'drive'),
        ("복귀실패 + Z 미확인",            DONE, None,  False, 'abort'),
        ("중첩(복귀중) + Z 올라옴",        PARK, 0.0,   True,  'drive'),
        ("중첩(복귀중) + Z 내려감 (-30)",  PARK, -30.0, True,  'abort'),
        ("중첩(복귀중) + Z 미확인",        PARK, None,  True,  'abort'),
    ]
    sel = sys.argv[1:] or [str(i) for i in range(len(cases))]
    i = int(sel[0])
    c = cases[i]
    print(f"[{i}] {c[0]}")
    sys.exit(0 if run(*c) else 1)
