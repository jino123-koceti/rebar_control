#!/usr/bin/env python3
"""상부축 가감속을 올리면서 **전류를 재는** 도구.

왜 전용 도구가 필요한가 — 2026-10-04 에 X(X4-10)의 가감속을 12000 까지 올려
240mm 왕복을 반복했고, 약 20분 뒤 그 모터가 CAN 에서 영구 침묵했다. 그때
전류를 사후에 한 번 보고 10.66A(정격 137%)인 것을 알았다. 올리는 **그 순간에**
보고 즉시 되돌릴 수단이 없었던 것이 문제다. 여분 모터가 전부 X4-36 이라
X4-10 축(Y·Z·yaw)을 하나라도 더 잃으면 짝이 없다.

세 가지를 반드시 한다:
  1. **쓰기 전에 0x42 로 원래 값을 읽어 파일에 적는다.** 2026-10-04 에 X·Y 의
     원래 가감속을 안 읽고 덮어써서 기준을 잃었다. `--restore` 로 되돌린다.
  2. **감속 구간을 따로 본다.** 피크는 가속이 아니라 감속에서 나왔다
     (10.66A). 속도 부호·크기로 구간을 갈라 각각의 피크를 낸다.
  3. **한계를 넘으면 그 자리에서 멈추고 되돌린다.** `/stage/stop` 을 보내고
     원래 가감속을 쓴 뒤 0 이 아닌 코드로 끝낸다.

이동은 **`/stage/goal` 로 보낸다** — stage_node 의 가동범위·충돌감지·원점 검사를
그대로 통과시키기 위해서다. 직접 0xA4 를 쏘면 그 보호가 전부 빠진다.
그래서 **해당 축이 호밍돼 있어야 한다** (mm 목표는 원점 없이는 거부된다).

샘플링은 0x9C 능동 폴링이다. 노드 폴링(35Hz)에 기대면 0.7초짜리 감속 피크를
놓칠 수 있다. 횡이동 감시가 축당 300Hz 로 도는 것이 검증돼 있어 150Hz 는 안전하다.

사용:
  # 원래 값 확인만 (아무것도 쓰지 않는다)
  python3 accel_sweep.py --axis x --read-only

  # 5000 → 14400 까지 단계적으로, 6.1A 넘으면 중단하고 되돌린다
  python3 accel_sweep.py --axis x --a-mm 100 --b-mm 250 \
      --speed-mm-s 80 --accels 5000,7000,10000,14400 --limit-a 6.1

  # 되돌리기
  python3 accel_sweep.py --axis x --restore
"""

import argparse
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time

FMT = "IB3x8s"
CMD_READ_ACCEL = 0x42
CMD_STATUS2 = 0x9C
SAVE_DIR = os.path.expanduser("~/lateral_logs")

# 축 → (CAN ID, 모델, 감속비, 데이터시트 정격 A)
AXIS = {
    'x':   (0x145, 'RMD-X4-36', 36.0, 6.1),
    'y':   (0x146, 'RMD-X4-10', 12.5, 7.8),
    'z':   (0x147, 'RMD-X4-10', 12.5, 7.8),
    'yaw': (0x148, 'RMD-X4-10', 12.5, 7.8),
}
ACCEL_IDX = ('위치가속', '위치감속', '속도가속', '속도감속')


class Can:
    """0x9C·0x42 만 보낸다 — 이 도구는 모터를 직접 움직이지 않는다."""

    def __init__(self, iface):
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((iface,))
        self.lock = threading.Lock()

    def _drain(self):
        self.s.settimeout(0.01)
        while True:
            try:
                self.s.recv(16)
            except OSError:
                return

    def ask(self, cid, data, want=None, win=0.12, tries=4):
        """질의 1회. 응답 ID 가 cid+0x100 인 것만 인정한다 — 노드의 **명령**
        프레임(cid 그대로)을 응답으로 오인하지 않기 위해서다."""
        for _ in range(tries):
            with self.lock:
                self._drain()
                self.s.send(struct.pack(FMT, cid, 8, bytes(data)))
                end = time.time() + win
                while time.time() < end:
                    self.s.settimeout(max(0.005, end - time.time()))
                    try:
                        raw = self.s.recv(16)
                    except OSError:
                        break
                    rid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
                    d = raw[8:16]
                    if rid == cid + 0x100 and d[0] == data[0]:
                        if want is None or d[1] == want:
                            return d
        return None

    def accel(self, cid, idx):
        d = self.ask(cid, [CMD_READ_ACCEL, idx, 0, 0, 0, 0, 0, 0], want=idx)
        return None if d is None else struct.unpack("<I", d[4:8])[0]

    def state(self, cid):
        """(온도℃, 전류A, 속도dps) — 0x9C 한 방에 다 온다."""
        d = self.ask(cid, [CMD_STATUS2, 0, 0, 0, 0, 0, 0, 0], win=0.04, tries=2)
        if d is None:
            return None
        return (d[1], struct.unpack("<h", d[2:4])[0] * 0.01,
                struct.unpack("<h", d[4:6])[0])


class Sampler(threading.Thread):
    """이동 중 0x9C 를 계속 떠서 (t, 전류, 속도, 온도) 를 쌓는다."""

    def __init__(self, can, cid, hz, limit_a, on_over):
        super().__init__(daemon=True)
        self.can, self.cid, self.dt = can, cid, 1.0 / hz
        self.limit_a, self.on_over = limit_a, on_over
        self.rows, self.stop_flag, self.tripped = [], False, None

    def run(self):
        t0 = time.time()
        while not self.stop_flag:
            st = self.can.state(self.cid)
            if st is not None:
                temp, cur, spd = st
                self.rows.append((time.time() - t0, cur, spd, temp))
                if self.tripped is None and abs(cur) > self.limit_a:
                    self.tripped = (abs(cur), time.time() - t0)
                    self.on_over(abs(cur))
            time.sleep(self.dt)


def phases(rows):
    """속도 크기의 증감으로 가속·순항·감속을 가른다.

    각 구간의 피크를 따로 내는 것이 요점이다 — 2026-10-04 의 10.66A 는 감속
    구간에서 나왔고, 전체 피크만 보면 어느 구간이 문제인지 알 수 없다.
    """
    out = {'가속': [], '순항': [], '감속': []}
    moving = [r for r in rows if abs(r[2]) > 2]
    if len(moving) < 4:
        return out
    top = max(abs(r[2]) for r in moving)
    for r in moving:
        v = abs(r[2])
        if v > 0.9 * top:
            out['순항'].append(r)
        else:
            out['가속' if r[0] < moving[len(moving) // 2][0] else '감속'].append(r)
    return out


def peak(rows):
    return max((abs(r[1]) for r in rows), default=0.0)


def save_path(cid):
    return os.path.join(SAVE_DIR, f"accel_orig_{cid:03x}.json")


def read_orig(can, cid):
    return [can.accel(cid, i) for i in range(4)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--axis', required=True, choices=sorted(AXIS))
    p.add_argument('--interface', default='can2')
    p.add_argument('--accels', default='')
    p.add_argument('--a-mm', type=float)
    p.add_argument('--b-mm', type=float)
    p.add_argument('--speed-mm-s', type=float)
    p.add_argument('--limit-a', type=float,
                   help='이 전류를 넘으면 중단하고 되돌린다. 기본은 데이터시트 정격')
    p.add_argument('--hz', type=float, default=150.0)
    p.add_argument('--read-only', action='store_true', help='원래 값만 읽는다')
    p.add_argument('--restore', action='store_true', help='저장된 원래 값으로 되돌린다')
    a = p.parse_args()

    cid, model, gear, rated = AXIS[a.axis]
    limit = rated if a.limit_a is None else a.limit_a
    can = Can(a.interface)

    cur = read_orig(can, cid)
    if cur[0] is None:
        print(f"✗ 0x{cid:03X} 가 0x42 에 응답하지 않습니다.")
        return 1
    print(f"축 {a.axis}  0x{cid:03X}  {model}  감속비 {gear}  정격 {rated}A")
    print("현재 가감속 (로터축 / 출력축):")
    for i, v in enumerate(cur):
        print(f"  {ACCEL_IDX[i]:6s} {v:>6} → {v / gear:7.0f} dps/s")

    os.makedirs(SAVE_DIR, exist_ok=True)
    sp = save_path(cid)
    if a.read_only:
        if not os.path.exists(sp):
            json.dump({'motor': cid, 'accel': cur, 'at': time.time()},
                      open(sp, 'w'), indent=2)
            print(f"\n원래 값을 처음 기록했습니다: {sp}")
        return 0

    if a.restore:
        if not os.path.exists(sp):
            print(f"✗ 저장된 원래 값이 없습니다: {sp}")
            return 1
        orig = json.load(open(sp))['accel']
        print(f"\n되돌리기 → {orig}")
        print("  ⚠ 쓰기는 ROS 쪽에서 합니다:")
        print(f"     ros2 topic pub --once /motor_accel std_msgs/String "
              f"\"{{data: '0x{cid:03x} {orig[0]}'}}\"")
        return 0

    if a.limit_a is not None and a.limit_a > rated:
        print(f"\n✗ --limit-a {a.limit_a}A 가 정격 {rated}A 보다 큽니다. 거부합니다.")
        print("   임계를 올려서 헛트립을 없애는 것이 2차년도에 모터를 태운 설정입니다.")
        return 1
    if not all(v is not None for v in (a.a_mm, a.b_mm, a.speed_mm_s)) or not a.accels:
        print("\n--a-mm --b-mm --speed-mm-s --accels 가 모두 필요합니다.")
        return 1

    if not os.path.exists(sp):
        json.dump({'motor': cid, 'accel': cur, 'at': time.time()},
                  open(sp, 'w'), indent=2)
        print(f"\n원래 값 기록: {sp}")

    return run_sweep(can, a, cid, gear, limit)


def run_sweep(can, a, cid, gear, limit):
    """rclpy 는 여기서만 쓴다 — 읽기 전용 경로는 ROS 없이도 돌게 하려고."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Empty
    from geometry_msgs.msg import Point

    rclpy.init()
    node = Node('accel_sweep')
    g_pub = node.create_publisher(Point, '/stage/goal', 10)
    s_pub = node.create_publisher(Empty, '/stage/stop', 10)
    a_pub = node.create_publisher(String, '/motor_accel', 10)
    state = {'st': None}
    node.create_subscription(
        String, '/stage/status',
        lambda m: state.update(st=json.loads(m.data)), 10)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def abort(cur_a):
        s_pub.publish(Empty())
        spin(0.3)
        node.get_logger().error(f"전류 {cur_a:.2f}A > {limit}A — 중단")

    spin(1.5)
    st = state['st']
    if st is None:
        print("✗ /stage/status 가 오지 않습니다. 서비스가 떠 있는지 확인하세요.")
        return 1
    if a.axis not in (st.get('homed') or []):
        print(f"✗ {a.axis} 축이 호밍되지 않았습니다 (homed={st.get('homed')}).")
        print("   mm 목표는 원점 없이는 거부됩니다 — 먼저 /homing_cmd 로 호밍하세요.")
        return 1

    def goto(mm, timeout=30.0):
        pt = Point()
        for k in ('x', 'y', 'z'):
            setattr(pt, k, mm if k == a.axis else float('nan'))
        g_pub.publish(pt)
        end = time.time() + timeout
        while time.time() < end:
            spin(0.1)
            st = state['st'] or {}
            now = (st.get('current_mm') or {}).get(a.axis)
            if not st.get('moving') and now is not None and abs(now - mm) < 2.0:
                return True
        return False

    # ⚠ 속도는 **토픽이 아니라 stage_node 파라미터**다 (`move_speed_mm_s_x`).
    #   2026-10-06 에 `/stage/speed` 로 보내다가 조용한 no-op 이었다 — 그 토픽은
    #   없고, 없는 토픽에 쏘면 오류 없이 무시되므로 측정이 엉뚱한 속도로 돈다.
    par = f"move_speed_mm_s_{a.axis}"
    r = subprocess.run(['ros2', 'param', 'set', '/stage_node', par,
                        str(float(a.speed_mm_s))], capture_output=True, text=True)
    if 'Set parameter successful' not in r.stdout:
        print(f"✗ 속도 파라미터 설정 실패: {r.stdout.strip()} {r.stderr.strip()}")
        return 1
    print(f"속도 {par} = {a.speed_mm_s} mm/s")
    results = []
    try:
        for val in [int(x) for x in a.accels.split(',')]:
            a_pub.publish(String(data=f"0x{cid:03x} {val}"))
            spin(1.0)
            got = can.accel(cid, 0)
            if got != val:
                print(f"\n✗ 가감속 쓰기 확인 실패: 요청 {val}, 읽은 값 {got}. 중단합니다.")
                break
            if not goto(a.a_mm):
                print(f"\n✗ 시작점 {a.a_mm}mm 도달 실패. 중단합니다.")
                break
            smp = Sampler(can, cid, a.hz, limit, abort)
            smp.start()
            t0 = time.time()
            ok = goto(a.b_mm)
            dt = time.time() - t0
            smp.stop_flag = True
            smp.join(1.0)
            ph = phases(smp.rows)
            row = {'accel': val, 'out_dpss': val / gear, 'sec': dt,
                   'peak_all': peak(smp.rows),
                   'peak_accel': peak(ph['가속']), 'peak_cruise': peak(ph['순항']),
                   'peak_decel': peak(ph['감속']),
                   'temp': max((r[3] for r in smp.rows), default=0),
                   'n': len(smp.rows), 'ok': ok, 'tripped': smp.tripped}
            results.append(row)
            print(f"\n가감속 {val} (출력축 {val / gear:.0f} dps/s)  {dt:.2f}초  "
                  f"샘플 {row['n']}")
            print(f"  피크  전체 {row['peak_all']:.2f}A  가속 {row['peak_accel']:.2f}A"
                  f"  순항 {row['peak_cruise']:.2f}A  **감속 {row['peak_decel']:.2f}A**"
                  f"  온도 {row['temp']}°C")
            if smp.tripped:
                print(f"  ✗ {smp.tripped[0]:.2f}A 로 한계 초과 — 되돌립니다")
                orig = json.load(open(save_path(cid)))['accel']
                a_pub.publish(String(data=f"0x{cid:03x} {orig[0]}"))
                spin(1.0)
                return 2
    finally:
        rclpy.shutdown()

    print("\n" + "=" * 62)
    print(f"{'가감속':>8s} {'출력축':>9s} {'시간':>6s} {'감속피크':>8s} {'전체피크':>8s}")
    for r in results:
        print(f"{r['accel']:>8} {r['out_dpss']:>7.0f}dps/s {r['sec']:>5.2f}s "
              f"{r['peak_decel']:>7.2f}A {r['peak_all']:>7.2f}A")
    print("되돌리기: --restore")
    return 0


if __name__ == '__main__':
    sys.exit(main())
