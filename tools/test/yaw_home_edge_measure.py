#!/usr/bin/env python3
"""yaw_home 감지 구간을 **단회전값**으로 다시 잰다.

## 왜 단회전인가

멀티턴은 전원을 내리면 날아가고 단회전만 복원된다. 그래서 전원을 넘겨 쓸 수 있는
기준은 단회전값뿐이다. `0x92` 위치 토픽은 **영점 오프셋이 적용된 값**이라 영점을
바꾸면 같이 움직인다 — 기준으로 쓰면 안 된다. 여기서는 `0x61`(생값)만 쓴다.

## 왜 다시 재는가

감지판을 옮기면 에지 위치가 바뀐다. 그러면 `axes.yaml` 의 `edge_single_from_high`,
`noon_single`, 자세 오프셋이 전부 무효가 되고, **탐색 방향 판정까지 틀어진다**
(에지가 자세 범위 안쪽에 있어서 자세에 따라 방향이 ± 두 가지다).

## 쓰는 법

    python3 tools/test/yaw_home_edge_measure.py

띄워 둔 채로 yaw 를 **손으로 천천히** 감지 구간을 지나 양방향으로 왕복시킨다.
켜짐/꺼짐 전이마다 단회전값과 진행 방향을 찍는다. Ctrl-C 로 끝내면 접근 방향별
에지와 구간 폭을 낸다.

⚠ yaw 는 끝 리미트가 없다. 기구 끝에 부딪히기 전에 사람이 멈춰야 한다.
⚠ 출력이 꺼져 있어야(`0x78`) 손으로 돌아간다. 켜져 있으면 서보가 되돌린다.
"""

import os
import socket
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

CPR = 262144
GEAR = 12.5
CPG = (CPR / 360.0) * GEAR          # counts / 건 1도
IFACE = os.environ.get('CAN_IFACE', 'can2')
YAW_ID = 0x148                      # 읽기 전용 질의만 보낸다


def wrap(d):
    return (d + CPR // 2) % CPR - CPR // 2


class EdgeMeasure(Node):
    def __init__(self):
        super().__init__('yaw_home_edge_measure')
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((IFACE,))
        self.sock.settimeout(0.15)
        self.home = None
        self.single = None
        self.prev = None
        self.events = []            # (단회전, 방향, 켜짐여부)
        self.samples = []
        self.create_subscription(Bool, '/limit_sensors/yaw_home', self._on_home, 10)
        self.create_timer(0.05, self.tick)
        self._warm()
        print(f"인터페이스 {IFACE}, yaw 0x{YAW_ID:03X} — 손으로 천천히 왕복시키세요")
        print(f"{'단회전':>9} {'이동(건°)':>10} {'센서':>5}")

    def _warm(self):
        """조용한 버스의 첫 프레임은 유실된다 — 버리는 셈으로 한 번 보낸다."""
        for _ in range(2):
            self._ask()
            time.sleep(0.05)

    def _ask(self):
        try:
            self.sock.send(struct.pack("IB3x8s", YAW_ID, 8,
                                       bytes([0x61, 0, 0, 0, 0, 0, 0, 0])))
        except OSError:
            return None
        end = time.time() + 0.1
        while time.time() < end:
            try:
                raw = self.sock.recv(16)
            except (socket.timeout, BlockingIOError):
                break
            cid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            d = raw[8:16]
            if cid == YAW_ID + 0x100 and d[0] == 0x61:
                return struct.unpack('<i', d[4:8])[0] % CPR
        return None

    def _on_home(self, msg):
        was, self.home = self.home, bool(msg.data)
        if was is None or was == self.home or self.single is None:
            return
        d = 0 if self.prev is None else wrap(self.single - self.prev)
        arrow = '증가' if d > 0 else ('감소' if d < 0 else '?')
        self.events.append((self.single, arrow, self.home))
        print(f"  ★ 센서 {'켜짐' if self.home else '꺼짐'} — 단회전 {self.single} "
              f"(진행 {arrow})")

    def tick(self):
        v = self._ask()
        if v is None:
            return
        self.prev, self.single = self.single, v
        self.samples.append(v)
        if self.prev is not None and abs(wrap(v - self.prev)) > 0.02 * CPG:
            moved = wrap(v - self.samples[0]) / CPG
            print(f"{v:>9} {moved:>+9.2f}° {'ON' if self.home else 'off':>5}")

    def report(self):
        print()
        print("=" * 64)
        if not self.events:
            print("전이가 없었습니다 — 감지 구간을 지나지 않았거나 센서 토픽이 없습니다.")
            print(f"  센서 현재 상태: {self.home}")
            return
        print("=== 전이 기록 ===")
        for v, arrow, on in self.events:
            print(f"  단회전 {v:>7}  진행 {arrow}  →  {'켜짐' if on else '꺼짐'}")
        ons = [v for v, _, on in self.events if on]
        offs = [v for v, _, on in self.events if not on]
        if ons and offs:
            width = abs(wrap(max(ons + offs) - min(ons + offs)))
            print()
            print(f"=== 감지 구간 ===")
            print(f"  켜짐 에지: {ons}")
            print(f"  꺼짐 에지: {offs}")
            print(f"  폭(관측)  {width} counts = 건 {width/CPG:.2f}°")
        print()
        print("=== axes.yaml 에 넣을 값 ===")
        inc = [v for v, a, on in self.events if on and a == '증가']
        dec = [v for v, a, on in self.events if on and a == '감소']
        if dec:
            print(f"  edge_single_from_high: {dec[-1]}    # counts 감소 방향 접근")
        if inc:
            print(f"  edge_single_from_low:  {inc[-1]}    # counts 증가 방향 접근")
        if not (inc and dec):
            print("  ⚠ 한쪽 방향만 봤다 — 양방향으로 구간을 **완전히 통과**해야")
            print("     구간 폭과 양쪽 에지가 나온다. 더 넓게 왕복할 것.")
        print("  ⚠ 12시·자세별 단회전값은 따로 재야 한다 (건을 그 자세에 놓고 0x61 읽기)")
        print("=" * 64)


def main():
    rclpy.init()
    n = EdgeMeasure()
    # SIGTERM(timeout·kill) 에도 보고서를 내야 한다. 안 그러면 백그라운드로
    # 띄웠을 때 측정값을 전부 잃는다.
    import signal

    def _bye(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _bye)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception) as e:
        if not isinstance(e, KeyboardInterrupt):
            print(f"(종료: {type(e).__name__})")
    finally:
        n.report()
        n.sock.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
