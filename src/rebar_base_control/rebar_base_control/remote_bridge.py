#!/usr/bin/env python3
"""L1 — can3 리모콘 수신기 소유자. 프레임을 /remote_control 로만 내보낸다.

이 노드는 **can3 를 여는 유일한 곳**이다 (YEAR3_ARCHITECTURE.md §3).
조작 노드(L3)가 can3 를 직접 읽으면 계층이 무너진다 — 하드웨어를 겸하게 되고,
같은 버스를 두 곳에서 열 수 있게 된다.

can_parser 를 그대로 쓰지 않는 이유: can_parser 는 can2(모터)까지 같이 잡는데
can2 는 motor_bridge 가 소유한다. 한 버스에 두 소유자가 생긴다.

## 프레임 (2026-09-29 실측 확정, memory/rebar-remote-can3-protocol.md)

  0x1E4  60ms  아날로그 4축.  DATA[0]=AN1 [1]=AN2 [2]=AN3 [3]=AN4
                중립 0x80(128), 최대 250, 최소 6 → 진폭 ±122 대칭
                DATA[4..7] 은 0x80 고정 (미배선)
  0x2E4  60ms  스위치/상태
                DATA[0] 평상시 0x62.  bit7 비상정지 Active / bit6 Release
                        bit0 S16(START·HORN 겸용) / bit2 S13 / bit3 S14
                DATA[3] 토글 8개.  bit0 S23 bit1 S24 bit2 S21 bit3 S22
                        bit4 S19 bit5 S20 bit6 S17 bit7 S18  (중립이면 0)
                DATA[4] 하위 4비트 롤링 카운터 (버튼이 아니다)
  0x764  300ms 1바이트 하트비트

## 주의 두 가지

**1. 비상정지는 bit7 만 보면 안 된다.** 해제가 `0x80 → 0x00 → 0x63 → 0x62` 로 지나가서
   bit7 만 보면 0x00 구간(실측 1.6~2.0초)과 누름 시작 0x22 에서 정지가 풀린다.
   Release(bit6)==0 인 동안을 정지로 본다. 캡처 재생으로 3회 확인했다.

**2. 하트비트로 송신기 연결을 판단할 수 없다.** 수신기는 송신기가 꺼져 있어도
   프레임을 계속 낸다 (3분간 모든 바이트 고정인데 통신은 정상이었다).
   → 값이 변하지 않는 것으로는 판단하지 말고, 조작이 필요한 쪽에서 데드맨을 둔다.

## can3 재연결 (2차년도 실측 교훈)

수신기가 USB 허브 경유라, 허브가 EMI 로 포트를 껐다 켜면(2026-09-22 트리거 USB 에서
7회 실측) 장치는 can3 로 다시 잡히는데 **옛 소켓은 영원히 오류**가 된다. 예전엔 쉬지도
않고 오류 로그만 반복하며 서비스 재시작 전까지 리모콘이 먹통이었다.
→ 수신 오류가 나면 1초 쉬고 소켓을 다시 연다.
"""

import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rebar_base_interfaces.msg import RemoteControl

CAN_ID_ANALOG = 0x1E4
CAN_ID_SWITCH = 0x2E4
CAN_ID_HEARTBEAT = 0x764

NEUTRAL = 128      # 0x80
SPAN = 122         # 실측 진폭 (6~250)

# 0x2E4 DATA[0]
BIT_S16, BIT_S13, BIT_S14, BIT_ESTOP_RELEASE, BIT_ESTOP_ACTIVE = 0, 2, 3, 6, 7
# 0x2E4 DATA[3] — 비트 순서대로 토글 이름
TOGGLES = ('S23', 'S24', 'S21', 'S22', 'S19', 'S20', 'S17', 'S18')
# RemoteControl.buttons 순서 (2차년도와 동일하게 유지 — 구독자 호환)
BUTTON_ORDER = ('S13', 'S14', 'S17', 'S18', 'S21', 'S22', 'S23', 'S24')

RECONNECT_SEC = 1.0


class RemoteBridge(Node):
    def __init__(self):
        super().__init__('remote_bridge')

        self.declare_parameter('can_interface', 'can3')
        self.declare_parameter('publish_rate', 20.0)
        # 이 시간 동안 프레임이 없으면 통신 끊김으로 본다 (수신기·버스 단절)
        self.declare_parameter('frame_timeout', 0.3)

        self.iface = self.get_parameter('can_interface').value
        self.frame_timeout = float(self.get_parameter('frame_timeout').value)

        self.pub = self.create_publisher(RemoteControl, '/remote_control', 10)

        self._lock = threading.Lock()
        self.analog = [NEUTRAL] * 4
        self.sw0 = 0x62
        self.sw3 = 0x00
        self.last_frame = 0.0
        self.hb_count = 0
        self.sock = None
        self._warned_stale = False

        self.running = True
        self._open()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_rate').value), self.publish_tick)
        self.get_logger().info(f"리모콘 브릿지 시작 — {self.iface} → /remote_control")

    # ---- CAN ---------------------------------------------------------------
    def _open(self):
        try:
            if self.sock is not None:
                self.sock.close()
        except OSError:
            pass
        try:
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((self.iface,))
            s.settimeout(0.2)
            self.sock = s
            return True
        except OSError as e:
            self.sock = None
            self.get_logger().error(f"{self.iface} 열기 실패: {e}", throttle_duration_sec=5.0)
            return False

    def _rx_loop(self):
        while self.running:
            if self.sock is None:
                time.sleep(RECONNECT_SEC)
                self._open()
                continue
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                continue
            except OSError as e:
                # 허브가 포트를 껐다 켜면 옛 소켓은 영구 오류가 된다 → 다시 연다
                self.get_logger().error(
                    f"{self.iface} 수신 오류: {e} → {RECONNECT_SEC:.0f}초 후 재연결",
                    throttle_duration_sec=5.0)
                time.sleep(RECONNECT_SEC)
                self._open()
                continue
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
                elif cid == CAN_ID_HEARTBEAT:
                    self.hb_count += 1

    # ---- 발행 --------------------------------------------------------------
    @staticmethod
    def _norm(raw):
        """0~255 → -1.0~1.0. 중립 128, 진폭 122."""
        return float(max(-1.0, min(1.0, (raw - NEUTRAL) / float(SPAN))))

    def publish_tick(self):
        with self._lock:
            an = list(self.analog)
            sw0, sw3 = self.sw0, self.sw3
            age = time.time() - self.last_frame if self.last_frame else None

        if age is None or age > self.frame_timeout:
            # 프레임이 끊겼다. 안전한 값(중립 + 비상정지)으로 내보낸다.
            if not self._warned_stale:
                self._warned_stale = True
                self.get_logger().warning(
                    f"{self.iface} 프레임 끊김 — 비상정지로 발행합니다")
            msg = RemoteControl()
            msg.emergency_stop = True
            msg.joysticks = [0.0, 0.0, 0.0, 0.0]
            msg.buttons = [0] * len(BUTTON_ORDER)
            self.pub.publish(msg)
            return
        if self._warned_stale:
            self._warned_stale = False
            self.get_logger().info(f"{self.iface} 프레임 복구")

        toggles = {name: bool(sw3 >> i & 1) for i, name in enumerate(TOGGLES)}
        s13 = bool(sw0 >> BIT_S13 & 1)
        s14 = bool(sw0 >> BIT_S14 & 1)

        msg = RemoteControl()
        # Release(bit6)==0 도 정지로 본다 — bit7 만 보면 해제 과정에서 정지가 풀린다
        msg.emergency_stop = bool(sw0 >> BIT_ESTOP_ACTIVE & 1) \
            or not (sw0 >> BIT_ESTOP_RELEASE & 1)
        msg.switch_s10 = toggles['S19']      # Manual (2차년도 필드명 유지)
        msg.switch_s20 = toggles['S20']      # Auto
        msg.joysticks = [self._norm(an[0]), self._norm(an[1]),
                         self._norm(an[2]), self._norm(an[3])]
        flags = {'S13': s13, 'S14': s14}
        flags.update(toggles)
        msg.buttons = [1 if flags[n] else 0 for n in BUTTON_ORDER]
        msg.an_command = 1 if bool(sw0 >> BIT_S16 & 1) else 0   # S16 START/HORN
        self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        try:
            if self.sock is not None:
                self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RemoteBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
