#!/usr/bin/env python3
"""EZI-IO 노드 — 상부 리미트 + 하부 범퍼·조작스위치를 ROS 토픽으로 발행한다.

3차년도는 보드가 **2개**다. 둘 다 이 노드 하나가 읽는다 (board_id 로 구분).

  상부 192.168.0.6 board_id=0  Ethernet-**I8O8** (입력 8 / 출력 8)
       스테이지 리미트 7개. **전부 a접점** (평상시 OFF, 트리거되면 ON)
       IN00 x_min  IN01 x_max  IN02 z_min  IN03 z_max  IN04 y_min  IN05 yaw_home  IN06 y_max
       IN07 미사용

  하부 192.168.0.5 board_id=1  Ethernet-**IN16** (입력 전용, 출력 없음)
       범퍼 4개 + 조작 스위치 2개. **극성이 섞여 있다** — 아래 표 참고
       IN00 후방 START(a)  IN01 STOP(a)  IN05 후방범퍼(a)  IN07 전방범퍼(a)
       IN10 우측범퍼(b)    IN13 좌측범퍼(b)
       IN02 는 평상시 ON 이나 용도 미확인, IN03/IN04 근접센서는 미검증

모든 매핑은 2026-09-29 실측이다. 센서를 하나씩 순서대로 만져 확인했고,
2차년도 설정값과는 상부·하부 모두 전부 달랐다.

## 발행 토픽

  /limit_sensors/{x_min,x_max,y_min,y_max,z_min,z_max,yaw_home}  Bool  (True = 리미트 도달)
  /bumpers/{front,rear,left,right}                               Bool  (True = 눌림)
  /switches/{start_rear,stop}                                    Bool  (True = 눌림)
  /ezi_io/inputs_upper, /ezi_io/inputs_lower                     Int32 (원시 비트, 진단용)
  /ezi_io/diagnostics                                            DiagnosticArray

**극성은 이 노드에서 흡수한다.** 구독자는 항상 "True = 동작(도달·눌림)" 으로만 보면 된다.
b접점(평상시 ON)을 그대로 넘기면 받는 쪽마다 반전을 기억해야 하고, 한 곳만 빠뜨려도
범퍼가 거꾸로 동작한다. 2차년도 bumper_node 는 4개가 전부 b접점이라는 가정으로 쓰였다.

## 통신

Modbus TCP 502 는 이 보드가 **거부한다** (yaml 의 port 는 무시). FASTECH Plus-E
라이브러리의 UDP(FAS_Connect)로 붙는다. TCP(FAS_ConnectTCP)는 꺼진 보드에서 3초씩
막혀 다른 보드 갱신까지 멈춘다. 보드가 없으면 주기적으로 재연결을 시도한다.
"""

import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

sys.path.append(os.environ.get('FASTECH_LIBRARY_PATH',
                               os.path.expanduser('~/python/PE/Library')))
try:
    from FAS_EziMOTIONPlusE import (FAS_Connect, FAS_Close, FAS_GetInput,
                                    FAS_GetSlaveInfo)
    from ReturnCodes_Define import FMM_OK
except ImportError as e:  # pragma: no cover
    print(f"FASTECH Plus-E 라이브러리를 찾을 수 없습니다: {e}")
    print("  FASTECH_LIBRARY_PATH 또는 ~/python/PE/Library 확인")
    sys.exit(1)

# 장치타입 → (이름, 입력 점수)
BOARD_TYPES = {150: ("Ethernet-IN16", 16), 151: ("Ethernet-IN32", 32),
               155: ("Ethernet-I8O8", 8), 156: ("Ethernet-I16O16", 16)}

RECONNECT_SEC = 3.0


class Board:
    """EZIO 보드 하나. 연결·재연결과 입력 읽기만 담당한다."""

    def __init__(self, node, ip, bid, label):
        self.node, self.ip, self.bid, self.label = node, ip, bid, label
        self.connected = False
        self.n_in = 16
        self.type_name = "?"
        self.inputs = 0
        self.last_try = 0.0

    def _now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def connect(self):
        self.last_try = self._now()
        octets = [int(x) for x in self.ip.split('.')]
        if not FAS_Connect(*octets, self.bid):
            return False
        self.connected = True
        r, dtype, desc = FAS_GetSlaveInfo(self.bid)
        if r == FMM_OK:
            self.type_name, self.n_in = BOARD_TYPES.get(dtype, (f"알수없음({dtype})", 16))
        self.node.get_logger().info(
            f"{self.label} 연결: {self.ip} board_id={self.bid} {self.type_name} 입력 {self.n_in}점")
        return True

    def close(self):
        if self.connected:
            FAS_Close(self.bid)
            self.connected = False

    def read(self):
        """입력 비트를 읽어 True/False 를 갱신한다. 실패 시 재연결."""
        if not self.connected:
            if self._now() - self.last_try > RECONNECT_SEC:
                self.connect()
            return False
        r, inp, _latch = FAS_GetInput(self.bid)
        if r != FMM_OK:
            self.node.get_logger().warning(
                f"{self.label} 입력 읽기 실패 (코드 {r}) — 재연결", throttle_duration_sec=5.0)
            self.close()
            return False
        self.inputs = inp & ((1 << self.n_in) - 1)
        return True

    def bit(self, ch, normally_closed=False):
        """채널 상태를 '동작 중인가' 로 바꿔 돌려준다 (극성 흡수)."""
        raw = bool(self.inputs >> ch & 1)
        return (not raw) if normally_closed else raw


class EziIoNode(Node):
    def __init__(self):
        super().__init__('ezi_io_node')

        self.declare_parameter('upper_ip', '192.168.0.6')
        self.declare_parameter('upper_board_id', 0)
        self.declare_parameter('lower_ip', '192.168.0.5')
        self.declare_parameter('lower_board_id', 1)
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('enable_upper', True)
        self.declare_parameter('enable_lower', True)

        # 상부 리미트 (전부 a접점)
        self.declare_parameter('limit_x_min_channel', 0)
        self.declare_parameter('limit_x_max_channel', 1)
        self.declare_parameter('limit_z_min_channel', 2)
        self.declare_parameter('limit_z_max_channel', 3)
        self.declare_parameter('limit_y_min_channel', 4)
        self.declare_parameter('limit_yaw_home_channel', 5)
        self.declare_parameter('limit_y_max_channel', 6)
        self.declare_parameter('limits_normally_closed', False)

        # 하부 범퍼 — 전방·후방은 a접점, 좌·우는 b접점
        self.declare_parameter('bumper_front_channel', 7)
        self.declare_parameter('bumper_rear_channel', 5)
        self.declare_parameter('bumper_left_channel', 13)
        self.declare_parameter('bumper_right_channel', 10)
        self.declare_parameter('bumper_front_normally_closed', False)
        self.declare_parameter('bumper_rear_normally_closed', False)
        self.declare_parameter('bumper_left_normally_closed', True)
        self.declare_parameter('bumper_right_normally_closed', True)

        # 하부 조작 스위치 (a접점)
        self.declare_parameter('start_switch_rear_channel', 0)
        self.declare_parameter('stop_switch_channel', 1)
        self.declare_parameter('switches_normally_closed', False)

        p = self.get_parameter
        lim_nc = bool(p('limits_normally_closed').value)
        self.limit_map = {                       # 토픽 이름 → (채널, b접점 여부)
            'x_min':    (int(p('limit_x_min_channel').value), lim_nc),
            'x_max':    (int(p('limit_x_max_channel').value), lim_nc),
            'y_min':    (int(p('limit_y_min_channel').value), lim_nc),
            'y_max':    (int(p('limit_y_max_channel').value), lim_nc),
            'z_min':    (int(p('limit_z_min_channel').value), lim_nc),
            'z_max':    (int(p('limit_z_max_channel').value), lim_nc),
            'yaw_home': (int(p('limit_yaw_home_channel').value), lim_nc),
        }
        self.bumper_map = {
            'front': (int(p('bumper_front_channel').value),
                      bool(p('bumper_front_normally_closed').value)),
            'rear':  (int(p('bumper_rear_channel').value),
                      bool(p('bumper_rear_normally_closed').value)),
            'left':  (int(p('bumper_left_channel').value),
                      bool(p('bumper_left_normally_closed').value)),
            'right': (int(p('bumper_right_channel').value),
                      bool(p('bumper_right_normally_closed').value)),
        }
        sw_nc = bool(p('switches_normally_closed').value)
        self.switch_map = {
            'start_rear': (int(p('start_switch_rear_channel').value), sw_nc),
            'stop':       (int(p('stop_switch_channel').value), sw_nc),
        }

        self.limit_pubs = {n: self.create_publisher(Bool, f'/limit_sensors/{n}', 10)
                           for n in self.limit_map}
        self.bumper_pubs = {n: self.create_publisher(Bool, f'/bumpers/{n}', 10)
                            for n in self.bumper_map}
        self.switch_pubs = {n: self.create_publisher(Bool, f'/switches/{n}', 10)
                            for n in self.switch_map}
        self.raw_upper_pub = self.create_publisher(Int32, '/ezi_io/inputs_upper', 10)
        self.raw_lower_pub = self.create_publisher(Int32, '/ezi_io/inputs_lower', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/ezi_io/diagnostics', 10)

        self.upper = Board(self, p('upper_ip').value, int(p('upper_board_id').value), '상부') \
            if p('enable_upper').value else None
        self.lower = Board(self, p('lower_ip').value, int(p('lower_board_id').value), '하부') \
            if p('enable_lower').value else None
        for b in (self.upper, self.lower):
            if b and not b.connect():
                self.get_logger().warning(
                    f"{b.label} 보드({b.ip}) 연결 실패 — {RECONNECT_SEC:.0f}초마다 재시도")

        self._prev = {}
        self.timer = self.create_timer(1.0 / float(p('update_rate').value), self.tick)
        self.get_logger().info("EZI-IO 노드 시작 (극성은 이 노드에서 흡수, True = 동작)")

    # ---- 주기 처리 ----------------------------------------------------------
    def tick(self):
        if self.upper and self.upper.read():
            self.raw_upper_pub.publish(Int32(data=int(self.upper.inputs)))
            for name, (ch, nc) in self.limit_map.items():
                self._pub(self.limit_pubs[name], f'limit/{name}', self.upper.bit(ch, nc))
        if self.lower and self.lower.read():
            self.raw_lower_pub.publish(Int32(data=int(self.lower.inputs)))
            for name, (ch, nc) in self.bumper_map.items():
                self._pub(self.bumper_pubs[name], f'bumper/{name}', self.lower.bit(ch, nc))
            for name, (ch, nc) in self.switch_map.items():
                self._pub(self.switch_pubs[name], f'switch/{name}', self.lower.bit(ch, nc))
        self._publish_diagnostics()

    def _pub(self, pub, key, value):
        pub.publish(Bool(data=bool(value)))
        if self._prev.get(key) != value:      # 상태가 바뀔 때만 로그
            self._prev[key] = value
            if value:
                self.get_logger().info(f"▶ {key} 동작")
            else:
                self.get_logger().info(f"  {key} 해제")

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for b in (self.upper, self.lower):
            if not b:
                continue
            st = DiagnosticStatus()
            st.name = f'ezi_io/{b.label}'
            st.hardware_id = f'{b.ip}:{b.bid}'
            if b.connected:
                st.level = DiagnosticStatus.OK
                st.message = f'{b.type_name} 정상'
                st.values = [KeyValue(key='inputs', value=f'0x{b.inputs:X}')]
            else:
                st.level = DiagnosticStatus.ERROR
                st.message = '연결 안 됨'
            arr.status.append(st)
        self.diag_pub.publish(arr)

    def destroy_node(self):
        for b in (self.upper, self.lower):
            if b:
                b.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EziIoNode()
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
