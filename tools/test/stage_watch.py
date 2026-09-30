#!/usr/bin/env python3
"""상부 스테이지 감시 — 리미트 7개 + 축 위치·명령을 한 화면에서 본다. 읽기 전용.

호밍(S3) 작업용이다. 확인해야 하는 것이 세 가지다:
  · 축이 움직일 때 리미트가 실제로 물리는가
  · 어느 방향이 min 이고 어느 방향이 max 인가 (배선이 반대일 수 있다)
  · 리미트에 닿는 순간의 위치값 (호밍 레퍼런스가 된다)

토픽마다 `ros2 topic echo --once` 를 따로 돌리면 초당 몇 번밖에 못 읽어 순간을 놓친다.
여기서는 한 노드가 전부 구독한다.

사용:
    python3 stage_watch.py            # 화면 갱신
    python3 stage_watch.py --log      # 변화만 한 줄씩 (배경 실행용)
    python3 stage_watch.py --sec 120
"""

import argparse
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64

LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')
# axes.yaml 기준: joint_3=0x145 X, joint_4=0x146 Y, joint_5=0x147 Z, joint_6=0x148 Yaw
AXES = (('X', 'joint_3', '0x145'), ('Y', 'joint_4', '0x146'),
        ('Z', 'joint_5', '0x147'), ('Yaw', 'joint_6', '0x148'))

GREEN, YELLOW, RED, CYAN, DIM = '\033[92m', '\033[93m', '\033[91m', '\033[96m', '\033[2m'
BOLD, RESET, CLEAR = '\033[1m', '\033[0m', '\033[H\033[2J'


class StageWatch(Node):
    def __init__(self, log_only):
        super().__init__('stage_watch')
        self.log_only = log_only
        self.limit = {n: None for n in LIMITS}
        self.cmd = {a[0]: 0.0 for a in AXES}
        self.pos = {a[0]: None for a in AXES}
        self.events = []

        for n in LIMITS:
            self.create_subscription(Bool, f'/limit_sensors/{n}',
                                     lambda m, k=n: self._on_limit(k, m), 10)
        for name, joint, mid in AXES:
            self.create_subscription(Float32, f'/{joint}/speed',
                                     lambda m, k=name: self.cmd.__setitem__(k, m.data), 10)
            # position_control_node 가 발행하는 모터별 위치
            self.create_subscription(Float64, f'/motor_{mid}_position',
                                     lambda m, k=name: self.pos.__setitem__(k, m.data), 10)

    def _on_limit(self, name, msg):
        prev = self.limit[name]
        self.limit[name] = msg.data
        if prev is not None and prev != msg.data:
            pos = ", ".join(f"{k}={v:.1f}°" for k, v in self.pos.items() if v is not None)
            line = (f"{datetime.now():%H:%M:%S.%f}"[:-3]
                    + f"  {name} {'도달' if msg.data else '해제'}"
                    + (f"   위치: {pos}" if pos else ""))
            self.events.append(line)
            if self.log_only:
                print(line, flush=True)

    def render(self):
        out = [CLEAR + f"{BOLD}상부 스테이지 감시{RESET}  {datetime.now():%H:%M:%S}"
               f"   {DIM}Ctrl+C 종료{RESET}", ""]
        out.append(f"{BOLD}리미트{RESET}")
        cells = []
        for n in LIMITS:
            v = self.limit[n]
            if v is None:
                cells.append(f"{DIM}{n} ?{RESET}")
            elif v:
                cells.append(f"{RED}{BOLD}{n} 도달{RESET}")
            else:
                cells.append(f"{GREEN}{n} ·{RESET}")
        out.append("   " + "   ".join(cells))
        out.append("")
        out.append(f"{BOLD}축{RESET}    {'명령(dps)':>12} {'위치(°)':>12}")
        for name, joint, mid in AXES:
            c = self.cmd[name]
            p = self.pos[name]
            cc = f"{YELLOW}{c:+9.1f}{RESET}" if abs(c) > 0.1 else f"{DIM}{c:+9.1f}{RESET}"
            pp = f"{p:+10.1f}" if p is not None else f"{DIM}{'?':>10}{RESET}"
            out.append(f"  {name:<4} {mid}  {cc}   {pp}")
        out.append("")
        out.append(f"{BOLD}리미트 변화 기록{RESET} (최근 12건)")
        out += self.events[-12:] or [f"{DIM}  아직 없음{RESET}"]
        sys.stdout.write("\n".join(out) + "\n")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', action='store_true', help='변화만 출력 (배경 실행용)')
    ap.add_argument('--sec', type=float, default=0.0, help='이 시간 뒤 종료 (0=무한)')
    ap.add_argument('--hz', type=float, default=5.0, help='화면 갱신')
    a = ap.parse_args()

    rclpy.init()
    node = StageWatch(a.log)
    t_end = time.time() + a.sec if a.sec > 0 else None
    next_draw = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if t_end and time.time() > t_end:
                break
            if not a.log and time.time() >= next_draw:
                next_draw = time.time() + 1.0 / a.hz
                node.render()
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n리미트 변화 {len(node.events)}건")
        for line in node.events:
            print("  " + line)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
