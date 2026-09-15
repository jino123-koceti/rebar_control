#!/usr/bin/env python3
"""
횡이동 2축 ROS2 노드

구독:
  /lateral/step   (std_msgs/Int32)   +N = 좌측 N회전(50mm), -N = 우측
발행:
  /lateral/complete (std_msgs/String) "COMPLETE" / "FAILED"
  /lateral/state    (std_msgs/String) 상태 요약

CAN 소켓을 직접 소유한다 (raw 부호 규약). position_control_node 와
동시에 떠 있어도 되지만, /joint_1/position, /joint_2/position 으로
0x143/0x144 를 동시에 명령하지는 말 것 — 부호 규약이 반대라 충돌한다.
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from rmd_robot_control.lateral_axes import LateralAxes, LateralAxis, MM_PER_TURN

DEFAULT_HOME_JSON = "/home/koceti/ros2_ws/src/rebar_control/tools/motor/lateral_home.json"


class LateralNode(Node):
    def __init__(self):
        super().__init__('lateral_node')

        self.declare_parameter('can_interface', 'can2')
        self.declare_parameter('home_json', DEFAULT_HOME_JSON)
        self.declare_parameter('speed_dps', 50)
        self.declare_parameter('max_turns', 4)
        # 하드 차단(0x80) 임계 [A]. 데이터시트 피크 21.5A(rms) 를 넘기지 말 것.
        # 18.0 은 리프팅 기동 서지(실측 18.0~18.2A, 0.1~0.2초)에 걸려 매번 차단됐다.
        self.declare_parameter('i_hard_a', 20.0)
        # 전류 프로파일 CSV 저장 위치. '' 이면 기록하지 않는다.
        self.declare_parameter('record_dir', '/home/koceti/lateral_logs')
        # 0xA2/0xA4 DATA[1] maxTorque. 0 = 힘 제어 비활성. 자세한 배경은 lateral_axes 주석.
        self.declare_parameter('max_torque', 255)

        iface = self.get_parameter('can_interface').value
        home_json = self.get_parameter('home_json').value
        speed = int(self.get_parameter('speed_dps').value)
        self.max_turns = int(self.get_parameter('max_turns').value)
        i_hard_a = float(self.get_parameter('i_hard_a').value)
        record_dir = str(self.get_parameter('record_dir').value or '')
        max_torque = int(self.get_parameter('max_torque').value)

        axes = self._load_axes(home_json)
        if axes is None:
            raise RuntimeError("횡이동 홈 설정을 읽지 못했습니다")

        self.ctl = LateralAxes(axes, interface=iface, speed_dps=speed,
                               logger=self.get_logger(), i_hard_a=i_hard_a,
                               max_torque=max_torque)

        self._rec_f = None
        self._rec_move = 0
        self._rec_n = 0
        if record_dir:
            self._open_recorder(record_dir)

        self.complete_pub = self.create_publisher(String, '/lateral/complete', 10)
        self.state_pub = self.create_publisher(String, '/lateral/state', 10)
        self.create_subscription(Int32, '/lateral/step', self.on_step, 10)

        self._busy = threading.Lock()
        self.create_timer(2.0, self.publish_state)

        from rmd_robot_control.lateral_axes import (
            I_RATED_A, I_HARD_A, I2T_BUDGET, TEMP_STOP_C)
        self.get_logger().info(
            f"횡이동 노드 시작: {iface}, {speed} dps, 최대 {self.max_turns}회전/명령")
        self.get_logger().info(
            f"  maxTorque(DATA[1]) = {self.ctl.max_torque}")
        self.get_logger().info(
            f"  보호: 정격 {I_RATED_A}A / 즉시차단 {self.ctl.i_hard_a}A / "
            f"I²t {I2T_BUDGET:.0f} A²s / 온도 {TEMP_STOP_C}°C → 0x80 차단")
        for ax in self.ctl.axes:
            self.get_logger().info(
                f"  0x{ax.motor_id:03X} {ax.name}: 부호 {ax.sign:+d}, "
                f"12시 엔코더 {ax.home_enc_single}")

    # ---- 전류 프로파일 기록 ------------------------------------------------
    def _open_recorder(self, record_dir):
        """감시 루프의 모든 샘플을 CSV 로 남긴다 (약 620 Hz x 축수).

        보호 임계를 근거 있게 정하려면 최대값 몇 개가 아니라 파형이 필요하다.
        행정 위치별 전류를 봐야 '어디서 얼마나 더 필요한가' 를 외삽할 수 있다.
        """
        try:
            os.makedirs(record_dir, exist_ok=True)
            path = os.path.join(
                record_dir, time.strftime('lateral_%Y%m%d_%H%M%S.csv'))
            self._rec_f = open(path, 'w', buffering=1 << 16)
            self._rec_f.write('move,motor_id,t_s,current_a,speed_dps,temp_c,i2t,angle_raw\n')
            self.get_logger().info(f"  전류 프로파일 기록: {path}")
        except Exception as e:
            self._rec_f = None
            self.get_logger().warning(f"  전류 프로파일 기록 실패 (계속 진행): {e}")
            return
        self.ctl.recorder = self._record

    def _record(self, mid, t, cur, spd, temp, i2t, ang):
        f = self._rec_f
        if f is None:
            return
        try:
            f.write(f"{self._rec_move},0x{mid:03X},{t:.4f},{cur:.2f},"
                    f"{spd},{temp},{i2t:.1f},{ang}\n")
            self._rec_n += 1
        except Exception:
            pass

    def _load_axes(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"홈 설정 파일 없음: {path}")
            return None
        try:
            cfg = json.load(open(path))
        except Exception as e:
            self.get_logger().error(f"홈 설정 파싱 실패: {e}")
            return None
        axes = []
        for key, mid in (("0x143", 0x143), ("0x144", 0x144)):
            c = cfg.get(key)
            if not c:
                self.get_logger().error(f"{key} 설정 없음")
                return None
            if c.get("home_enc_single") is None:
                self.get_logger().error(
                    f"{key} home_enc_single 미설정 — "
                    f"tools/motor/lateral_dual.py set-home 을 먼저 실행하세요")
                return None
            axes.append(LateralAxis(mid, int(c.get("sign", 1)),
                                    int(c["home_enc_single"]),
                                    c.get("name", key)))
        return axes

    def on_step(self, msg: Int32):
        turns = int(msg.data)
        if turns == 0:
            return
        if abs(turns) > self.max_turns:
            self.get_logger().warning(
                f"횡이동 거부: {turns}회전은 상한 {self.max_turns}회전 초과")
            self.complete_pub.publish(String(data="FAILED"))
            return
        if not self._busy.acquire(blocking=False):
            self.get_logger().warning("횡이동 진행 중 — 명령 무시")
            return
        threading.Thread(target=self._run, args=(turns,), daemon=True).start()

    def _run(self, turns):
        self._rec_move += 1
        self._rec_n = 0
        try:
            ok, _ = self.ctl.move_turns(turns)
            self.complete_pub.publish(String(data="COMPLETE" if ok else "FAILED"))
        except Exception as e:
            self.get_logger().error(f"횡이동 예외: {e}")
            self.complete_pub.publish(String(data="FAILED"))
        finally:
            if self._rec_f is not None:
                try:
                    self._rec_f.flush()
                    self.get_logger().info(
                        f"  프로파일 기록: move {self._rec_move}, {self._rec_n} 샘플")
                except Exception:
                    pass
            self._busy.release()

    def publish_state(self):
        if self._busy.locked():
            return
        parts = []
        for ax in self.ctl.axes:
            off = self.ctl.home_offset_deg(ax)
            st = self.ctl.status(ax.motor_id)
            if off is None or st is None:
                parts.append(f"0x{ax.motor_id:03X}=응답없음")
                continue
            parts.append(f"0x{ax.motor_id:03X} 12시까지 {off:+.1f}° "
                         f"({off/7.2:+.1f}mm) {st['temp']}°C err=0x{st['err']:04X}")
        self.state_pub.publish(String(data=" | ".join(parts)))

    def destroy_node(self):
        try:
            self.ctl.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LateralNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"횡이동 노드 시작 실패: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
