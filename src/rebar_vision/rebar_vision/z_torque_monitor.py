#!/usr/bin/env python3
"""
Z축 토크 모니터링 모듈

Z 하강 시 0x9C 폴링으로 전류/속도를 모니터링하여 충돌(과부하)을 감지.
2단계 감지: 가속 구간(고임계값) + 감속 구간(저임계값, 연속 N회).
"""

from rebar_base_interfaces.msg import JointControl


class ZTorqueMonitor:
    """Z축 하강 토크 모니터링.

    사용법:
        monitor = ZTorqueMonitor(node, params)
        # motor_feedback 콜백에서:
        monitor.update_feedback(current_mA, speed_dps)
        # Z 하강 시작 시:
        monitor.start(z_wait_time, speed_percent, effective_z_speed)
        # WAITING_Z 상태에서 매 tick:
        result = monitor.check_overload(elapsed)
        if result:  # 'OVERLOAD_ACCEL' or 'OVERLOAD_DECEL'
            monitor.log_summary(result)
            monitor.stop()
        # Z 하강 정상 완료 시:
        monitor.log_summary('OK')
        monitor.stop()
    """

    def __init__(self, node, params, encoder_request_pub, joint_pub):
        """
        Args:
            node: ROS2 노드 (get_clock, create_timer, get_logger, flog)
            params: dict with keys:
                z_torque_base_mA, z_torque_scale_mA, z_torque_delay_ratio,
                z_torque_poll_hz, z_torque_decel_mA, z_torque_decel_ratio,
                z_torque_decel_count
            encoder_request_pub: /encoder_request 퍼블리셔
            joint_pub: /joint_control 퍼블리셔 (Z정지 명령용)
        """
        self._node = node
        self._encoder_request_pub = encoder_request_pub
        self._joint_pub = joint_pub

        # 파라미터
        self._base_mA = params['z_torque_base_mA']
        self._scale_mA = params['z_torque_scale_mA']
        self._delay_ratio = params['z_torque_delay_ratio']
        self._poll_hz = params['z_torque_poll_hz']
        self._decel_mA = params['z_torque_decel_mA']
        self._decel_ratio = params['z_torque_decel_ratio']
        self._decel_count = params['z_torque_decel_count']

        # 상태
        self.z_descending = False
        self.current_mA = 0
        self.speed_dps = 0
        self._poll_timer = None
        self._descent_start_time = None
        self._samples = []
        self._decel_hit = 0

        # 계산값 (start 시 설정)
        self.accel_limit_mA = 0
        self.check_delay = 0.0
        self.z_wait_time = 0.0

    def update_feedback(self, current_mA, speed_dps):
        """motor_feedback 콜백에서 0x9C 응답 수신 시 호출."""
        self.current_mA = abs(current_mA)
        self.speed_dps = abs(int(speed_dps))

    def start(self, z_wait_time, speed_percent, effective_z_speed):
        """Z 하강 시작 시 모니터링 시작."""
        self.z_descending = True
        self.current_mA = 0
        self._decel_hit = 0
        self._samples = []
        self._descent_start_time = self._node.get_clock().now()
        self.z_wait_time = z_wait_time

        # 임계값 계산
        self.accel_limit_mA = int(self._base_mA + speed_percent * self._scale_mA)
        self.check_delay = z_wait_time * self._delay_ratio

        if self._poll_timer is None:
            period = 1.0 / self._poll_hz
            self._poll_timer = self._node.create_timer(period, self._poll)
            decel_start = z_wait_time * self._decel_ratio
            self._node.get_logger().info(
                f'    Z축 토크 모니터링 시작 ({self._poll_hz}Hz, '
                f'가속임계={self.accel_limit_mA}mA, '
                f'감속임계={self._decel_mA}mA@{decel_start:.2f}s, '
                f'연속{self._decel_count}회 @{speed_percent:.0f}%)')
            self._node.flog(
                f"Z_DOWN_START: speed%={speed_percent:.0f} "
                f"eff_speed={effective_z_speed:.0f}dps "
                f"accel_limit={self.accel_limit_mA}mA "
                f"decel_limit={self._decel_mA}mA "
                f"decel_start={decel_start:.3f}s "
                f"decel_count={self._decel_count} "
                f"z_wait={z_wait_time:.3f}s "
                f"poll_hz={self._poll_hz}")

    def stop(self):
        """모니터링 정지."""
        self.z_descending = False
        if self._poll_timer is not None:
            self._poll_timer.cancel()
            self._node.destroy_timer(self._poll_timer)
            self._poll_timer = None

    def check_overload(self, elapsed):
        """WAITING_Z에서 매 tick 호출. 과부하 시 결과 문자열 반환, 정상이면 None.

        Returns:
            None: 정상
            'OVERLOAD_ACCEL': 가속 구간 과부하
            'OVERLOAD_DECEL': 감속 구간 과부하
        """
        if not self.z_descending or elapsed < self.check_delay:
            return None

        # 1) 가속 구간: 기존 고임계값
        if self.current_mA > self.accel_limit_mA:
            self._node.get_logger().warn(
                f'    ⚠️ Z축 토크 과부하 감지! {self.current_mA}mA > '
                f'{self.accel_limit_mA}mA → 하강 정지, 트리거 스킵')
            return 'OVERLOAD_ACCEL'

        # 2) 감속 구간: 저임계값 + 연속 초과
        decel_start = self.z_wait_time * self._decel_ratio
        if elapsed >= decel_start:
            if self.current_mA > self._decel_mA:
                self._decel_hit += 1
                if self._decel_hit >= self._decel_count:
                    self._node.get_logger().warn(
                        f'    ⚠️ Z축 감속구간 과부하! {self.current_mA}mA > '
                        f'{self._decel_mA}mA (연속 {self._decel_hit}회) '
                        f'→ 하강 정지, 트리거 스킵')
                    return 'OVERLOAD_DECEL'
            else:
                self._decel_hit = 0

        return None

    def log_summary(self, result):
        """하강 완료/과부하 시 통계 요약을 파일 로그에 기록."""
        if not self._samples:
            self._node.flog(f"Z_DOWN_END: result={result} samples=0")
            return
        currents = [s[1] for s in self._samples]
        speeds = [s[2] for s in self._samples]
        duration_ms = self._samples[-1][0] - self._samples[0][0]
        self._node.flog(
            f"Z_DOWN_END: result={result} "
            f"samples={len(self._samples)} duration_ms={duration_ms:.0f} "
            f"current_min={min(currents)} current_max={max(currents)} "
            f"current_avg={sum(currents)/len(currents):.0f} "
            f"speed_min={min(speeds)} speed_max={max(speeds)} "
            f"speed_avg={sum(speeds)/len(speeds):.0f} "
            f"final_current={currents[-1]} final_speed={speeds[-1]}")

    def stop_z_motor(self):
        """Z축 즉시 정지 (속도 0 명령)."""
        stop_msg = JointControl()
        stop_msg.joint_id = 0x146
        stop_msg.position = 0.0
        stop_msg.velocity = 0.0
        stop_msg.control_mode = JointControl.MODE_SPEED
        self._joint_pub.publish(stop_msg)

    def _poll(self):
        """0x9C 명령으로 Z축 모터 상태 요청."""
        msg = JointControl()
        msg.joint_id = 0x146
        msg.control_mode = 0x9C
        msg.position = 0.0
        msg.velocity = 0.0
        self._encoder_request_pub.publish(msg)

        elapsed_ms = (self._node.get_clock().now() - self._descent_start_time).nanoseconds / 1e6
        self._node.get_logger().info(
            f'    [토크폴링] Z전류={self.current_mA}mA 속도={self.speed_dps}dps')
        self._node.flog(
            f"Z_TORQUE: t_ms={elapsed_ms:.0f} "
            f"current_mA={self.current_mA} "
            f"speed_dps={self.speed_dps} "
            f"limit_mA={self.accel_limit_mA}")
        self._samples.append((elapsed_ms, self.current_mA, self.speed_dps))
