#!/usr/bin/env python3
"""
리모콘(can3) → 3차년도 장비 제어 변환 노드

can3 프레임을 직접 읽어 teleop_keyboard 와 같은 토픽으로 발행한다.
따라서 잘 동작하는 position_control_node / lateral_node 를 그대로 쓴다.

can_parser(rebar_base_control) 를 쓰지 않고 can3 를 직접 읽는 이유:
can_parser 는 can2(모터)까지 같이 잡는데, can2 는 position_control_node 가
점유하고 있어 같이 띄우면 한 버스에 두 노드가 붙는다.

──────────────────────────────────────────────────────────────────────────
조작 (2차년도 iron_md_teleop_node 방식 + S13/S14 만 변경, 2026-09-29 확정)

  AN3   하부체 전후진      AN3− = 전진, AN3+ = 후진   → /cmd_vel linear.x
  AN4   하부체 좌우회전    AN4+ = CCW,  AN4− = CW     → /cmd_vel angular.z
  AN1   상부체 X축 속도                              → /joint_3/speed (0x145)
  AN2   상부체 Y축 속도                              → /joint_4/speed (0x146)

  AN3/AN4 의 부호가 뒤집힌 것은 좌우 주행모터가 180도 반대로 설치되어서다
  (2차년도와 동일 — 3차년도에서도 같은 것으로 확인).

  S19   Remote 모드 (이 노드가 조작을 통과시킴)
  S20   Auto 모드   (스틱 입력 무시, 정지 유지)
  S17   횡이동 +1스텭 (좌측 50mm)  → /lateral/step
  S18   횡이동 −1스텭 (우측 50mm)  → /lateral/step
  S13   누르고 있는 동안 Z축 상승  → /joint_5/speed (0x147)   ← 2차년도는 브레이크 해제였음
  S14   누르고 있는 동안 Z축 하강  → /joint_5/speed (0x147)   ← 2차년도는 위치 리셋이었음
  S16   START / HORN (겸용). 이 노드는 쓰지 않는다
  비상정지 → 모든 출력 0, 해제될 때까지 유지

  미구현: S21/S22(작업 시퀀스), S23/S24(Yaw ±5°)
    Yaw(0x148)는 position_control_node 의 motor_ids 에 없어 토픽이 아예 없다.
    쓰려면 robot_control.yaml 의 motor_ids 에 0x148 을 추가해야 한다.
──────────────────────────────────────────────────────────────────────────

프레임 매핑은 memory/rebar-remote-can3-protocol.md 와 같다:
  0x1E4 DATA[0]=AN1 [1]=AN2 [2]=AN3 [3]=AN4, 중립 128, 진폭 ±122
  0x2E4 DATA[0] bit7=비상정지Active bit6=Release bit2=S13 bit3=S14 bit0=S16
        DATA[3] bit0..7 = S23 S24 S21 S22 S19 S20 S17 S18
        DATA[4] = 롤링 카운터 (무시)
"""

import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, String

CAN_ID_ANALOG = 0x1E4
CAN_ID_SWITCH = 0x2E4
CAN_ID_HEARTBEAT = 0x764

NEUTRAL = 128
SPAN = 122                  # 실측 진폭 (6~250)

# 0x2E4 DATA[3] 토글 비트
BIT_S23, BIT_S24, BIT_S21, BIT_S22, BIT_S19, BIT_S20, BIT_S17, BIT_S18 = range(8)
# 0x2E4 DATA[0] 비트
BIT_S16, BIT_S13, BIT_S14, BIT_ESTOP_RELEASE, BIT_ESTOP_ACTIVE = 0, 2, 3, 6, 7


class RemoteTeleop(Node):
    def __init__(self):
        super().__init__('remote_teleop_node')

        self.declare_parameter('can_interface', 'can3')
        # position_control_node 의 상한과 맞춘다. 더 크게 줘도 노드가 잘라낸다.
        self.declare_parameter('max_linear_vel', 0.25)    # m/s
        self.declare_parameter('max_angular_vel', 0.5)    # rad/s
        self.declare_parameter('xy_max_dps', 50.0)        # X/Y 최대 속도
        self.declare_parameter('z_dps', 50.0)             # Z축 속도 (S13/S14)
        self.declare_parameter('deadzone', 0.08)          # 정규화 기준 불감대
        self.declare_parameter('publish_rate', 20.0)
        # 이 시간 동안 0x2E4 가 안 오면 정지시킨다 (수신기/버스 단절)
        self.declare_parameter('frame_timeout', 0.3)
        self.declare_parameter('lateral_timeout', 25.0)

        self.iface = self.get_parameter('can_interface').value
        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)
        self.xy_max = float(self.get_parameter('xy_max_dps').value)
        self.z_dps = float(self.get_parameter('z_dps').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.frame_timeout = float(self.get_parameter('frame_timeout').value)
        self.lateral_timeout = float(self.get_parameter('lateral_timeout').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x_pub = self.create_publisher(Float32, '/joint_3/speed', 10)   # 0x145 X
        self.y_pub = self.create_publisher(Float32, '/joint_4/speed', 10)   # 0x146 Y
        self.z_pub = self.create_publisher(Float32, '/joint_5/speed', 10)   # 0x147 Z
        self.lat_pub = self.create_publisher(Int32, '/lateral/step', 10)
        self.create_subscription(String, '/lateral/complete', self._on_lat_done, 10)

        self._lock = threading.Lock()
        self.analog = [NEUTRAL] * 4
        self.sw0 = 0x62          # 평상시값
        self.sw3 = 0x00
        self.last_frame = 0.0
        self.prev_s17 = self.prev_s18 = False
        self.lat_busy = False
        self.lat_started = 0.0
        self._warned = set()
        self._last_state = None

        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.iface,))
        self.sock.settimeout(0.2)
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.timer = self.create_timer(1.0 / float(self.get_parameter('publish_rate').value),
                                       self.tick)
        self.get_logger().info(
            f"리모콘 텔레옵 시작 — {self.iface} 수신, "
            f"주행 {self.max_lin} m/s · {self.max_ang} rad/s, XY {self.xy_max} dps, Z {self.z_dps} dps")
        self.get_logger().info("S19=Remote 모드에서만 조작이 통과합니다 (S20=Auto 면 정지 유지)")

    # ---- CAN 수신 -----------------------------------------------------------
    def _rx_loop(self):
        while self.running:
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                continue
            except OSError:
                break
            cid, dlc = struct.unpack("=IB3x", frame[:8])
            cid &= socket.CAN_EFF_MASK
            data = frame[8:8 + dlc]
            with self._lock:
                if cid == CAN_ID_ANALOG and len(data) >= 4:
                    self.analog = list(data[:4])
                    self.last_frame = time.time()
                elif cid == CAN_ID_SWITCH and len(data) >= 4:
                    self.sw0, self.sw3 = data[0], data[3]
                    self.last_frame = time.time()

    def _on_lat_done(self, msg):
        self.lat_busy = False
        self.lat_started = 0.0
        self.get_logger().info(f"횡이동 완료: {msg.data}")

    # ---- 변환 ---------------------------------------------------------------
    def _norm(self, raw):
        """0~255 → -1.0~1.0, 불감대 적용."""
        v = max(-1.0, min(1.0, (raw - NEUTRAL) / float(SPAN)))
        return 0.0 if abs(v) < self.deadzone else v

    def tick(self):
        with self._lock:
            an = list(self.analog)
            sw0, sw3 = self.sw0, self.sw3
            age = time.time() - self.last_frame if self.last_frame else 999.0

        # 1) 프레임이 끊기면 정지 (수신기 분리·버스 단절)
        if age > self.frame_timeout:
            self._stop("리모콘 프레임 끊김")
            return

        # 2) 비상정지 — Release(bit6) 가 0 인 동안 유지한다.
        #    Active(bit7) 만 보면 해제 과정의 0x00 구간에서 정지가 풀린다 (실측).
        estop = bool(sw0 >> BIT_ESTOP_ACTIVE & 1) or not (sw0 >> BIT_ESTOP_RELEASE & 1)
        if estop:
            self._stop("비상정지")
            return

        # 3) 모드 — S19(Remote) 일 때만 조작을 통과시킨다
        s19 = bool(sw3 >> BIT_S19 & 1)
        s20 = bool(sw3 >> BIT_S20 & 1)
        if not s19:
            self._stop("Auto 모드(S20)" if s20 else "모드 스위치 중립")
            return

        # 4) 스틱 → 주행 / XY
        #    AN3− 가 전진이므로 부호를 뒤집는다 (좌우 모터 180도 반대 설치)
        an1, an2, an3, an4 = (self._norm(an[0]), self._norm(an[1]),
                              self._norm(an[2]), self._norm(an[3]))
        lin = -an3 * self.max_lin
        ang = an4 * self.max_ang          # AN4+ = CCW
        self._publish_drive(lin, ang)
        self.x_pub.publish(Float32(data=float(an1 * self.xy_max)))
        self.y_pub.publish(Float32(data=float(an2 * self.xy_max)))

        # 5) S13/S14 → Z축 (누르고 있는 동안만). 둘 다 눌리면 정지.
        s13 = bool(sw0 >> BIT_S13 & 1)
        s14 = bool(sw0 >> BIT_S14 & 1)
        z = 0.0 if s13 == s14 else (self.z_dps if s13 else -self.z_dps)
        self.z_pub.publish(Float32(data=float(z)))

        # 6) S17/S18 → 횡이동 1스텭 (누른 순간에만, 완료까지 래치)
        s17 = bool(sw3 >> BIT_S17 & 1)
        s18 = bool(sw3 >> BIT_S18 & 1)
        if self.lat_busy and self.lat_started and \
                time.time() - self.lat_started > self.lateral_timeout:
            self.lat_busy = False
            self.get_logger().warning(
                f"횡이동 완료 신호가 {self.lateral_timeout:.0f}초간 없어 래치를 풉니다 "
                "— lateral_node 확인 필요")
        for pressed, prev, turns, name in ((s17, self.prev_s17, 1, 'S17 좌측'),
                                           (s18, self.prev_s18, -1, 'S18 우측')):
            if pressed and not prev:
                if self.lat_busy:
                    self.get_logger().info("횡이동 진행 중 — 무시")
                elif self.lat_pub.get_subscription_count() == 0:
                    self.get_logger().warning("/lateral/step 구독자 없음 — lateral_node 확인")
                else:
                    self.lat_busy = True
                    self.lat_started = time.time()
                    self.lat_pub.publish(Int32(data=turns))
                    self.get_logger().info(f"횡이동 {name} 50mm")
        self.prev_s17, self.prev_s18 = s17, s18

        # 7) 미구현 토글은 한 번만 알린다
        for bit, name in ((BIT_S21, 'S21'), (BIT_S22, 'S22'),
                          (BIT_S23, 'S23'), (BIT_S24, 'S24')):
            if sw3 >> bit & 1 and name not in self._warned:
                self._warned.add(name)
                extra = " (Yaw 0x148 은 motor_ids 에 없어 토픽이 없습니다)" \
                    if name in ('S23', 'S24') else " (작업 시퀀스 미구현)"
                self.get_logger().warning(f"{name} 는 아직 구현되지 않았습니다{extra}")

        self._note(f"Remote  주행 {lin:+.2f}/{ang:+.2f}  XY {an1*self.xy_max:+.0f}/"
                   f"{an2*self.xy_max:+.0f}  Z {z:+.0f}")

    def _publish_drive(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def _stop(self, reason):
        self._publish_drive(0.0, 0.0)
        for pub in (self.x_pub, self.y_pub, self.z_pub):
            pub.publish(Float32(data=0.0))
        self._note(f"정지 — {reason}")

    def _note(self, msg):
        """상태가 바뀔 때만 로그를 남긴다 (20Hz 로 찍으면 로그가 못 쓰게 된다)."""
        if msg != self._last_state:
            self._last_state = msg
            self.get_logger().info(msg)

    def destroy_node(self):
        self.running = False
        try:
            self._stop("노드 종료")
            time.sleep(0.1)
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RemoteTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print(f"CAN 인터페이스 오류: {e}\n  ip link show can3 으로 상태를 확인하세요")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
