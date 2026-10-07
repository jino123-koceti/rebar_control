#!/usr/bin/env python3
"""결속건 액추에이터를 방향키로 찍어보는 도구.

Pololu Simple Motor Controller 18v7 은 **위치 피드백이 없는 개루프 속도
컨트롤러**다. 그래서 "어디까지 가라" 는 명령이 없고, 이동량은 **시간으로만**
정해진다. 이 도구가 한 번에 "정해진 속도로 정해진 시간" 을 보내는 이유다.

방향 규약 (2026-10-07 실장비 확인)
  · 수축 = 트리거 **당김(발사)**       → 음수 (-1.0)
  · 팽창 = 트리거 **놓음(원복)**       → 양수 (+1.0)
  ⚠ 2차년도 코드는 `trigger_speed: +1.0` 을 발사로 쓴다 — 지금 배선과 **반대**다.
    그래서 `tying_sequence` 의 `gun_speed` 는 **음수**여야 한다.

속도를 바꿀 수 있게 둔 이유: 이 컨트롤러는 **모터 전류를 보고하는 변수가 아예
없다**(데이터시트 0J44 §6.4). 그래서 마찰이 늘었는지 줄었는지 직접 못 잰다.
대신 **"움직이기 시작하는 최소 듀티"** 를 대용 지표로 쓴다. 기준점은
2026-10-07 커버 볼팅 작업 **전**에 0.2 에서 안 움직이고 1.0 에서 움직인 것이다.

조작
  ←       팽창(놓음)        →       수축(당김)
  space   즉시 정지         q/Ctrl-C  종료 (정지 후)
  , .     시간 ∓0.1초       - =       속도 ∓0.1
  s       현재 설정 표시
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

TOPIC = '/motor_0/vel'
KEY_LEFT = '\x1b[D'
KEY_RIGHT = '\x1b[C'


class GunTeleop(Node):
    def __init__(self):
        super().__init__('gun_teleop')
        self.pub = self.create_publisher(Float32, TOPIC, 10)
        self.speed = 1.0
        self.sec = 1.0

    def wait_subscriber(self, timeout=5.0):
        """구독자가 붙기 전에 보내면 **첫 메시지가 조용히 버려진다.**

        눌렀는데 아무 일도 안 일어나면 하드웨어를 의심하게 되므로 기다린다.
        """
        end = time.time() + timeout
        while self.pub.get_subscription_count() < 1 and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.pub.get_subscription_count() >= 1

    def _send(self, v):
        self.pub.publish(Float32(data=float(v)))
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.005)

    def stop(self):
        self._send(0.0)

    def pulse(self, sign, label):
        v = sign * self.speed
        print(f"  {label}  vel={v:+.2f}  {self.sec:.1f}s", flush=True)
        self._send(v)
        end = time.time() + self.sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop()


def read_key(timeout=0.1):
    """방향키는 `ESC [ D` 3바이트로 들어온다.

    ⚠ 시간 조절 키를 `[`·`]` 로 두면 안 된다 (2026-10-07 에 그랬다). 연타하면
    시퀀스가 쪼개져 남은 `[` 가 "시간 감소" 로 먹혀 **1.0초가 0.1초로 깎인다.**
    사용자는 1초를 보내는 줄 알고 0.1초를 보내게 된다. 그래서 `,`·`.` 를 쓴다.
    남은 두 바이트를 기다리는 창도 0.02 → 0.08초로 늘렸다 — 터미널이 세 바이트를
    쪼개 보내면 짧은 창으로는 놓친다.
    """
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch != '\x1b':
        return ch
    seq = ch
    end = time.time() + 0.08
    while len(seq) < 3 and time.time() < end:
        if select.select([sys.stdin], [], [], max(0.0, end - time.time()))[0]:
            seq += sys.stdin.read(1)
    return seq


def drain_stdin():
    """이동 중에 눌린 키를 버린다.

    1초 펄스 동안 눌린 키가 쌓이면 손을 뗀 뒤에도 액추에이터가 계속 움직인다.
    개루프라 '어디까지 갔는지' 를 알 수 없으니 쌓인 입력은 버리는 쪽이 안전하다.
    """
    # ⚠ 한 번만 훑으면 **시퀀스 중간에서 끊겨** 남은 `[D` 가 다음 키로 읽힌다.
    #   조용해질 때까지 비워야 한다.
    end = time.time() + 0.15
    while time.time() < end:
        if select.select([sys.stdin], [], [], 0.03)[0]:
            sys.stdin.read(1)
            end = time.time() + 0.15


def main():
    rclpy.init()
    node = GunTeleop()
    if not node.wait_subscriber():
        print(f"✗ {TOPIC} 에 구독자가 없습니다 — pololu_node 가 떠 있는지 확인하세요.")
        print("  ros2 run pololu_ros2 pololu_node --ros-args "
              "-p serial_port:=/dev/pololu_trigger -p motor_ids:=[0] "
              "-p motor_topics:=['motor_0/vel']")
        rclpy.shutdown()
        return 1

    print(__doc__)
    print(f"구독자 {node.pub.get_subscription_count()}개 — 준비됨 "
          f"(속도 {node.speed:.1f}, 시간 {node.sec:.1f}s)")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            k = read_key()
            if k is None:
                continue
            if k == KEY_LEFT:
                node.pulse(+1, '팽창(놓음)')
                drain_stdin()
            elif k == KEY_RIGHT:
                node.pulse(-1, '수축(당김)')
                drain_stdin()
            elif k == ' ':
                node.stop()
                print("  정지", flush=True)
            elif k in ('q', '\x03'):
                break
            elif k in (',', '.'):
                node.sec = max(0.1, min(10.0, node.sec + (0.1 if k == '.' else -0.1)))
                print(f"  시간 {node.sec:.1f}s", flush=True)
            elif k in ('-', '='):
                node.speed = max(0.1, min(1.0, node.speed + (0.1 if k == '=' else -0.1)))
                print(f"  속도 {node.speed:.1f}  (= 듀티 {int(node.speed * 3200)}/3200)",
                      flush=True)
            elif k == 's':
                print(f"  속도 {node.speed:.1f} (듀티 {int(node.speed * 3200)})  "
                      f"시간 {node.sec:.1f}s", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.stop()
        print("\n정지 — 종료")
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
