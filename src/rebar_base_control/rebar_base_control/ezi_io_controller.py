#!/usr/bin/env python3
"""
EZI-IO Controller Node
FASTECH EZI-IO-EN-L16O16N-T I/O 모듈에서 리미트 센서 읽기

rebar_base_control 패키지로 통합
- 원본: ezi_io_ros2/ezi_io_node.py
"""

import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# FASTECH Plus-E 라이브러리 경로 설정
library_path = os.environ.get('FASTECH_LIBRARY_PATH', '/home/koceti/python/PE/Library')

# 라이브러리 경로 추가
if os.path.exists(library_path):
    sys.path.append(library_path)
    _FASTECH_AVAILABLE = True
else:
    _FASTECH_AVAILABLE = False

# FASTECH 라이브러리 임포트 시도
try:
    if _FASTECH_AVAILABLE:
        from FAS_EziMOTIONPlusE import *
        from MOTION_DEFINE import *
        from ReturnCodes_Define import *
except ImportError:
    _FASTECH_AVAILABLE = False


class EziIoController(Node):
    """EZI-IO 리미트 센서 모니터링 노드"""

    def __init__(self):
        super().__init__('ezi_io_controller')

        # FASTECH 라이브러리 체크
        if not _FASTECH_AVAILABLE:
            self.get_logger().error(
                f"FASTECH 라이브러리를 찾을 수 없습니다. "
                f"경로: {library_path}"
            )
            self.get_logger().error(
                "환경 변수 설정: export FASTECH_LIBRARY_PATH=/path/to/PE/Library"
            )
            self.connected = False
            return

        # 파라미터 선언
        self.declare_parameter('ip_address', '192.168.0.3')
        self.declare_parameter('board_id', 0)
        self.declare_parameter('update_rate', 20.0)  # Hz

        # 리미트 센서 매핑 (입력 채널 번호)
        self.declare_parameter('limit_x_min_channel', 2)   # IN02
        self.declare_parameter('limit_x_max_channel', 3)   # IN03
        self.declare_parameter('limit_y_min_channel', 1)   # IN01
        self.declare_parameter('limit_y_max_channel', 0)   # IN00
        self.declare_parameter('limit_z_min_channel', 5)   # IN05
        self.declare_parameter('limit_z_max_channel', 6)   # IN06
        self.declare_parameter('limit_yaw_home_channel', 4)  # IN04 (Yaw 원점)

        # 파라미터 가져오기
        ip_str = self.get_parameter('ip_address').value
        self.board_id = self.get_parameter('board_id').value
        self.update_rate = self.get_parameter('update_rate').value

        # IP 주소를 4개의 숫자로 변환
        ip_parts = ip_str.split('.')
        self.ip_address = [int(x) for x in ip_parts]

        # 리미트 채널 매핑
        self.limit_channels = {
            'x_min': self.get_parameter('limit_x_min_channel').value,
            'x_max': self.get_parameter('limit_x_max_channel').value,
            'y_min': self.get_parameter('limit_y_min_channel').value,
            'y_max': self.get_parameter('limit_y_max_channel').value,
            'z_min': self.get_parameter('limit_z_min_channel').value,
            'z_max': self.get_parameter('limit_z_max_channel').value,
            'yaw_home': self.get_parameter('limit_yaw_home_channel').value,
        }

        # FASTECH Plus-E 연결
        self.connected = False
        self.connect_device()

        # 발행자 생성 - 개별 리미트 센서
        self.limit_publishers = {}
        for name in self.limit_channels.keys():
            topic = f'/limit_sensors/{name}'
            self.limit_publishers[name] = self.create_publisher(Bool, topic, 10)

        # 전체 입력 상태 발행 (진단용)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/ezi_io/diagnostics',
            10
        )

        # 타이머 - 주기적으로 입력 읽기
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.read_inputs_callback)

        # 이전 상태 저장 (변화 감지용)
        self.previous_limits = {}

        # ⚠️ [2026-08-19] 읽기 실패 시 **자동 재연결**.
        #   사고: 12:16 연결이 끊긴 뒤 `Failed to read inputs`를 **27,538회** 찍으며
        #   재연결을 한 번도 시도하지 않았다. 노드는 살아 있고 토픽도 등록돼 있어
        #   겉보기엔 정상이었으나 **리미트 데이터가 전혀 발행되지 않았고**,
        #   그 상태에서 호밍이 x_min을 향해 30초간 계속 이동해 **리미트를 뚫었다.**
        #   보드는 내내 정상이었다(ping·TCP 2001 모두 응답).
        #   → 연속 실패가 임계를 넘으면 연결을 닫고 다시 맺는다.
        self._read_fail = 0
        self._reconnect_after = 5        # 연속 실패 이 횟수 넘으면 재연결
        self._reconnect_backoff = 1.0    # 재연결 최소 간격(초) — 폭주 방지
        self._last_reconnect = 0.0
        self._down_since = None          # 통신 두절이 시작된 시각
        # ★ [2026-09-02] 일정 시간 복구 못 하면 **스스로 죽는다**.
        #   실측: 보드는 멀쩡한데(새 프로세스로는 즉시 연결·읽기 성공) 이 프로세스
        #   안에서는 FAS_Close→재연결로 복구가 안 됐다. FASTECH 라이브러리의
        #   프로세스 전역 상태가 꼬이면 **프로세스를 새로 띄우는 것 말고 방법이 없다.**
        #   그런데 노드는 살아 있어 launch가 되살리지도 않는다 →
        #   **39시간을 좀비로 버티며 284만 회 읽기 실패**했다(2026-09-02 실측).
        #   그동안 리미트 토픽이 전혀 안 나가 호밍이 계속 거부됐다.
        #   장비 전원 OFF는 상시 상황이라 너무 짧으면 재시작만 반복한다 → 120초.
        #   0이면 기능 끔(= 옛 동작, 무한 좀비).
        self.declare_parameter('exit_after_down_sec', 120.0)
        self._exit_after = float(self.get_parameter('exit_after_down_sec').value)

        ip_str_display = '.'.join(map(str, self.ip_address))
        self.get_logger().info(f'EZI-IO Controller started: {ip_str_display}')
        self.get_logger().info(f'Limit channels: {self.limit_channels}')

    def connect_device(self):
        """FASTECH Plus-E 연결"""
        if not _FASTECH_AVAILABLE:
            return False

        try:
            # TCP 연결 시도
            result = FAS_ConnectTCP(
                self.ip_address[0],
                self.ip_address[1],
                self.ip_address[2],
                self.ip_address[3],
                self.board_id
            )

            if result == 0:
                self.get_logger().error('Failed to connect to EZI-IO',
                                        throttle_duration_sec=5.0)
                return False

            ip_str = '.'.join(map(str, self.ip_address))
            self.get_logger().info(f'✅ Connected to EZI-IO at {ip_str}')
            self.connected = True
            return True

        except Exception as e:
            self.get_logger().error(f'EZI-IO connection error: {e}')
            return False

    def _try_reconnect(self):
        """읽기가 연속 실패하면 연결을 닫고 다시 맺는다.

        ⚠ 재연결 폭주를 막으려고 최소 간격(_reconnect_backoff)을 둔다 —
          보드가 완전히 죽은 경우 매 주기 접속을 시도하면 로그와 CPU만 낭비한다.
        """
        import time as _t
        now = _t.time()
        if now - self._last_reconnect < self._reconnect_backoff:
            return
        self._last_reconnect = now
        self.get_logger().warn(
            f'입력 읽기 {self._read_fail}회 연속 실패 → EZI-IO 재연결 시도')
        try:
            FAS_Close(self.board_id)                 # 남은 세션 정리 (실패해도 무시)
        except Exception:
            pass
        self.connected = False
        if self.connect_device():
            self.get_logger().info('✅ EZI-IO 재연결 성공')
            self._read_fail = 0
            self._down_since = None
        else:
            if self._down_since is None:
                self._down_since = now
            down = now - self._down_since
            self.get_logger().error(
                f'❌ EZI-IO 재연결 실패 ({down:.0f}s 두절) — 다음 주기에 재시도',
                throttle_duration_sec=5.0)
            if self._exit_after > 0 and down > self._exit_after:
                self.get_logger().fatal(
                    f'🛑 {down:.0f}s 복구 실패 → **노드 종료.** 프로세스를 새로 '
                    f'띄워야만 복구되는 경우가 있다(FASTECH 전역상태). launch가 되살린다.')
                raise SystemExit(1)

    def read_inputs_callback(self):
        """입력 상태 읽기 및 발행"""
        if not _FASTECH_AVAILABLE:
            return

        if not self.connected:
            # ⚠ [2026-09-02] 여기서 매 주기(20Hz) 무제한 재접속을 하고 있었다.
            #   보드가 죽으면 30초에 600줄이 쌓여 **진짜 오류를 파묻는다**.
            #   backoff를 가진 _try_reconnect로 일원화한다.
            self._read_fail += 1
            self._try_reconnect()
            return

        try:
            # FAS_GetInput으로 입력 읽기 (모터 드라이버/IO 공용)
            status_result, input_status, latch_status = FAS_GetInput(self.board_id)

            if status_result != FMM_OK:
                self._read_fail += 1
                self.get_logger().warn(
                    f'Failed to read inputs (error: {status_result}) '
                    f'연속 {self._read_fail}회',
                    throttle_duration_sec=2.0)
                if self._read_fail >= self._reconnect_after:
                    self._try_reconnect()
                return

            if self._read_fail:                      # 정상 복귀
                self.get_logger().info(
                    f'✅ 입력 읽기 복구됨 (실패 {self._read_fail}회 후)')
                self._read_fail = 0

            # 리미트 센서 상태 발행
            for name, channel in self.limit_channels.items():
                if channel < 16:  # 16 inputs total
                    # 비트 체크
                    state = bool(input_status & (1 << channel))

                    # 상태 변화시에만 로그 (TRIGGERED만 info, CLEAR는 debug)
                    if name not in self.previous_limits or self.previous_limits[name] != state:
                        if state:  # TRIGGERED
                            self.get_logger().info(f'⚠️ Limit {name} (IN{channel:02d}): TRIGGERED')
                        else:  # CLEAR
                            self.get_logger().debug(f'Limit {name} (IN{channel:02d}): CLEAR')
                        self.previous_limits[name] = state

                    # 발행
                    msg = Bool()
                    msg.data = state
                    self.limit_publishers[name].publish(msg)

            # 진단 메시지 발행
            self.publish_diagnostics(input_status)

        except Exception as e:
            self.get_logger().error(f'Error reading inputs: {e}',
                                    throttle_duration_sec=5.0)
            self.connected = False
            self._read_fail += 1
            self._try_reconnect()          # backoff 있는 경로로 일원화

    def publish_diagnostics(self, input_status):
        """진단 정보 발행"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "EZI-IO Limit Sensors"
        ip_str = '.'.join(map(str, self.ip_address))
        status.hardware_id = ip_str

        # 리미트 센서 상태
        any_triggered = False
        for name, channel in self.limit_channels.items():
            if channel < 16:
                state = bool(input_status & (1 << channel))
                status.values.append(KeyValue(
                    key=name,
                    value="TRIGGERED" if state else "CLEAR"
                ))
                if state:
                    any_triggered = True

        # 전체 상태 레벨
        if any_triggered:
            status.level = DiagnosticStatus.WARN
            status.message = "Some limits triggered"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "All limits clear"

        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)

    def destroy_node(self):
        """노드 종료"""
        if _FASTECH_AVAILABLE and self.connected:
            FAS_Close(self.board_id)
            self.get_logger().info('EZI-IO connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = EziIoController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
