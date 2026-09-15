#!/usr/bin/env python3
"""
키보드 텔레옵 — 리모콘(can3) 미연결 상태의 간이 조작

  전후진 / 좌우 제자리선회 : /cmd_vel  → position_control_node (0x141 우, 0x142 좌)
  좌우 횡이동              : /lateral/step → lateral_node (0x143, 0x144)

키:
   w / s      전진 / 후진
   a / d      좌선회 / 우선회 (제자리)
   q / e      좌측 횡이동 / 우측 횡이동 (1회전 = 50mm)
   space      즉시 정지
   z / x      선속도 -/+        c / v   각속도 -/+
   ?          도움말            Ctrl+C  종료

키를 떼면 정지한다 (누르고 있는 동안만 주행). 터미널 포커스가 이 창에 있어야 한다.
"""

import sys
import select
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

HELP = __doc__

LIN_STEP = 0.05
ANG_STEP = 0.1
# 노드(position_control_node)의 max_linear_vel / max_angular_vel 과 맞춘다.
# 더 크게 두면 노드가 잘라내므로 눌러도 안 빨라져 튜닝 중 혼선이 생긴다.
# 0.25 m/s = 바퀴 492 dps = RMD-X4-36 정격 498 dps 의 99%.
LIN_MAX = 0.25
ANG_MAX = 0.5
IDLE_STOP_SEC = 0.3      # 이 시간 동안 주행키 입력이 없으면 정지
# 횡이동 완료 신호가 이 시간 안에 안 오면 래치를 푼다.
# lateral_node 가 죽었거나 명령이 유실되면 완료가 영영 안 오는데,
# 그러면 q/e 가 영구히 "진행 중"으로 무시된다 (2026-09-08 실제 발생).
LAT_TIMEOUT_SEC = 25.0


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.declare_parameter('linear_vel', 0.10)
        self.declare_parameter('angular_vel', 0.15)  # 2026-09-08 선회 공진 회피
        self.lin = float(self.get_parameter('linear_vel').value)
        self.ang = float(self.get_parameter('angular_vel').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lat_pub = self.create_publisher(Int32, '/lateral/step', 10)
        self.create_subscription(String, '/lateral/complete', self.on_lat_done, 10)
        self.create_subscription(String, '/lateral/state', self.on_lat_state, 10)

        self.cur = (0.0, 0.0)     # (linear, angular)
        self.last_drive_key = 0.0
        self.lat_state = ""
        self.lat_busy = False
        self.lat_started = 0.0
        self.lat_warned_nosub = False
        self._lock = threading.Lock()
        self.create_timer(0.1, self.tick)

    # ---- 콜백 -------------------------------------------------------------
    def on_lat_done(self, msg: String):
        self.lat_busy = False
        self.lat_started = 0.0
        print(f"\r  횡이동 {msg.data}" + " " * 40)
        self.print_status()

    def on_lat_state(self, msg: String):
        self.lat_state = msg.data

    # ---- 주기 처리 --------------------------------------------------------
    def tick(self):
        """주행키가 끊기면 자동 정지 (데드맨) + 횡이동 래치 타임아웃"""
        if self.lat_busy and self.lat_started and \
                time.time() - self.lat_started > LAT_TIMEOUT_SEC:
            self.lat_busy = False
            self.lat_started = 0.0
            print(f"\r  ⚠️ 횡이동 응답 없음 {LAT_TIMEOUT_SEC:.0f}s — 래치 해제. "
                  f"lateral_node 가 떠 있는지 확인하세요" + " " * 10)
            self.print_status()
        with self._lock:
            lin, ang = self.cur
            if (lin or ang) and time.time() - self.last_drive_key > IDLE_STOP_SEC:
                self.cur = (0.0, 0.0)
                lin, ang = 0.0, 0.0
                self.publish(0.0, 0.0)
                self.print_status()
                return
        self.publish(lin, ang)

    def publish(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def print_status(self):
        lin, ang = self.cur
        sys.stdout.write(
            f"\r  주행 lin={lin:+.2f} ang={ang:+.2f} | "
            f"설정 {self.lin:.2f} m/s, {self.ang:.2f} rad/s"
            f"{' | 횡이동 중' if self.lat_busy else ''}   ")
        sys.stdout.flush()

    # ---- 키 처리 ----------------------------------------------------------
    def on_key(self, k):
        drive = {'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1)}
        if k in drive:
            sl, sa = drive[k]
            with self._lock:
                self.cur = (sl * self.lin, sa * self.ang)
                self.last_drive_key = time.time()
            self.print_status()
            return True
        if k == ' ':
            with self._lock:
                self.cur = (0.0, 0.0)
            self.publish(0.0, 0.0)
            print("\r  정지" + " " * 50)
            return True
        if k in ('q', 'e'):
            if self.lat_busy:
                print("\r  횡이동 진행 중 — 무시" + " " * 25)
                return True
            turns = 1 if k == 'q' else -1
            if self.lat_pub.get_subscription_count() == 0:
                print("\r  ✗ /lateral/step 구독자 없음 — lateral_node 가 안 떠 있습니다"
                      + " " * 15)
                return True
            self.lat_busy = True
            self.lat_started = time.time()
            self.lat_pub.publish(Int32(data=turns))
            print(f"\r  횡이동 {'좌측' if turns > 0 else '우측'} 50mm 명령" + " " * 20)
            return True
        if k in ('z', 'x'):
            self.lin = max(LIN_STEP, min(LIN_MAX, self.lin + (LIN_STEP if k == 'x' else -LIN_STEP)))
            self.print_status()
            return True
        if k in ('c', 'v'):
            self.ang = max(ANG_STEP, min(ANG_MAX, self.ang + (ANG_STEP if k == 'v' else -ANG_STEP)))
            self.print_status()
            return True
        if k == '?':
            print("\n" + HELP)
            if self.lat_state:
                print(f"  횡이동 상태: {self.lat_state}")
            return True
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    node.print_status()
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                k = sys.stdin.read(1)
                if k == '\x03':          # Ctrl+C
                    break
                node.on_key(k)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            node.publish(0.0, 0.0)
            time.sleep(0.2)
        except Exception:
            pass
        print("\n정지 후 종료합니다.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
