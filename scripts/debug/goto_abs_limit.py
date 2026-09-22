#!/usr/bin/env python3
"""goto_abs_limit.py — 멀티턴 절대 엔코더 리미트값 검증 이동 도구

목적:
  검증 때 기록한 각 축의 min/max 리미트 0x92 절대각도가 "지금도" 유효한지,
  실제로 해당 절대값으로 이동시키면 물리 리미트에 도달하는지 확인.

흐름:
  1) 축 선택 (X / Y / Z / Yaw)
  2) 그 축의 현재 0x92 절대 엔코더 값 출력
  3) min / max 선택 → 기록된 절대각도로 이동
  4) 이동 중 리미트 센서 모니터링 → 닿으면 즉시 정지 (하드스톱 방지)

⚠️ robot-control 서비스가 떠 있는 상태에서 실행 (ROS 토픽 사용, can_sender 재사용).
   raw CAN 직접 접근 아님 → 서비스 중지 불필요.

기록값 출처: docs/design/workpart_workspace_analysis.md (2026-05-29 측정)
  ⚠️ 0x64 ROM 리셋/커플링 슬립 후엔 값이 달라질 수 있음.

사용:
  ros2 run 없이 직접:  python3 scripts/debug/goto_abs_limit.py [x|y|z|yaw] [min|max]
  (인자 생략 시 대화형 선택)
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rebar_base_interfaces.msg import JointControl, MotorFeedback


# 축 정의: 기록된 리미트 0x92 절대각(deg). 출처: workpart_workspace_analysis.md (2026-05-29)
AXES = {
    'x': {
        'can_id': 0x144, 'mb_id': 0x44, 'label': 'X (0x144)',
        'step_deg': 4.497, 'stroke_deg': 1842.0,
        'ref': {'min': 1818.69, 'max': -23.53},   # min=xMin/home, max=xMax
        # 표시용 mm 환산: x_mm = (1818.69 - cur) / 4.497
        'to_mm': lambda c: (1818.69 - c) / 4.497,
        'default_speed': 20.0,
    },
    'y': {
        'can_id': 0x145, 'mb_id': 0x45, 'label': 'Y (0x145)',
        'step_deg': 4.462, 'stroke_deg': 1316.0,
        'ref': {'min': -989.36, 'max': 326.19},   # min=yMin/home, max=yMax
        'to_mm': lambda c: (c + 989.36) / 4.462,
        'default_speed': 20.0,
    },
    'z': {
        'can_id': 0x146, 'mb_id': 0x46, 'label': 'Z (0x146)',
        'step_deg': 13.45, 'stroke_deg': None,
        'ref': {'min': None, 'max': None},        # ⚠️ 미측정 (수동 입력 필요)
        'to_mm': None,
        'default_speed': 20.0,
    },
    'yaw': {
        'can_id': 0x147, 'mb_id': 0x47, 'label': 'Yaw (0x147)',
        'step_deg': None, 'stroke_deg': 396.0,
        'ref': {'min': -360.13, 'max': 36.10},    # min=home/min_limit, max=max_limit
        'to_mm': None,
        'default_speed': 15.0,
    },
}

# 축별 리미트 센서 토픽 (min/max → /limit_sensors/<name>)
LIMIT_TOPIC = {
    'x': {'min': 'x_min', 'max': 'x_max'},
    'y': {'min': 'y_min', 'max': 'y_max'},
    'z': {'min': 'z_min', 'max': 'z_max'},
    'yaw': {'min': 'yaw_home', 'max': None},   # yaw는 home 센서만 존재 (max 리미트 센서 없음)
}

# --to-limit 모드: 해당 리미트로 향하는 속도 부호 (모터 극성 기준, 메모리/호밍 근거)
#   X: +속도=xMin 방향 / Y: +속도=yMax 방향 / Z·Yaw: -속도=min(home) 방향(호밍 근거)
LIMIT_DIR = {
    'x': {'min': +1, 'max': -1},
    'y': {'min': -1, 'max': +1},
    'z': {'min': -1, 'max': +1},
    'yaw': {'min': -1, 'max': +1},
}

REACH_TOL_DEG = 2.0
ENC_REQ_PERIOD = 0.15   # 0x92 재요청 주기 (s)


class GotoAbsLimit(Node):
    def __init__(self, axis_key):
        super().__init__('goto_abs_limit')
        self.axis_key = axis_key
        self.ax = AXES[axis_key]
        self.mb_id = self.ax['mb_id']

        self.cur_deg = None          # 최신 0x92 절대각
        self.cur_deg_time = 0.0
        self.limit_state = {'min': None, 'max': None}

        self.joint_pub = self.create_publisher(JointControl, '/joint_control', 10)
        self.enc_pub = self.create_publisher(JointControl, '/encoder_request', 10)

        self.create_subscription(MotorFeedback, '/motor_feedback', self._on_feedback, 50)

        for side in ('min', 'max'):
            name = LIMIT_TOPIC[self.axis_key][side]
            if name:
                self.create_subscription(
                    Bool, f'/limit_sensors/{name}',
                    lambda m, s=side: self._on_limit(s, m), 10)

    # ---- callbacks ----
    def _on_feedback(self, msg: MotorFeedback):
        if msg.motor_id == self.mb_id and msg.status == 0x92:
            self.cur_deg = float(msg.current_position)
            self.cur_deg_time = time.monotonic()

    def _on_limit(self, side, msg: Bool):
        self.limit_state[side] = bool(msg.data)

    # ---- helpers ----
    def request_encoder(self):
        """0x92 멀티턴 절대각 읽기 요청"""
        m = JointControl()
        m.joint_id = self.ax['can_id']
        m.control_mode = 0x92
        m.position = 0.0
        m.velocity = 0.0
        self.enc_pub.publish(m)

    def read_current(self, timeout=3.0):
        """현재 0x92 절대각 취득 (요청 후 응답 대기)"""
        self.cur_deg = None
        t0 = time.monotonic()
        last_req = 0.0
        while time.monotonic() - t0 < timeout:
            if time.monotonic() - last_req > ENC_REQ_PERIOD:
                self.request_encoder()
                last_req = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.cur_deg is not None:
                return self.cur_deg
        return None

    def send_abs(self, target_deg, speed_dps):
        m = JointControl()
        m.joint_id = self.ax['can_id']
        m.control_mode = JointControl.MODE_ABSOLUTE
        m.position = float(target_deg)
        m.velocity = float(speed_dps)
        self.joint_pub.publish(m)

    def send_speed(self, speed_dps):
        """MODE_SPEED → 속도명령 (to-limit용). 모터는 다음 명령까지 속도 유지."""
        m = JointControl()
        m.joint_id = self.ax['can_id']
        m.control_mode = JointControl.MODE_SPEED
        m.position = float(speed_dps)
        m.velocity = 0.0
        self.joint_pub.publish(m)

    def send_stop(self):
        """MODE_SPEED 0 → can_sender가 모터정지(0x81) 전송"""
        m = JointControl()
        m.joint_id = self.ax['can_id']
        m.control_mode = JointControl.MODE_SPEED
        m.position = 0.0
        m.velocity = 0.0
        for _ in range(3):
            self.joint_pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def fmt(self, deg):
        s = f"{deg:.2f}°"
        if self.ax['to_mm'] is not None:
            s += f"  (≈{self.ax['to_mm'](deg):.1f}mm from home)"
        return s


def run_to_limit(node, axis_key, side):
    """속도명령으로 리미트까지 이동 → 센서로 정지 → 그때의 0x92(현재 프레임 기준점) 측정."""
    ax = node.ax
    tgt_limit = LIMIT_TOPIC[axis_key][side]
    if not tgt_limit:
        print(f"❌ {ax['label']} {side}에는 리미트 센서가 없어 to-limit 불가 (정지 불능). 중단.")
        return
    sign = LIMIT_DIR[axis_key][side]

    sp_in = input(f"to-limit 속도(dps), 기본 {ax['default_speed']:.0f}: ").strip()
    speed = float(sp_in) if sp_in else ax['default_speed']
    other = 'max' if side == 'min' else 'min'

    # 이미 리미트 ON?
    rclpy.spin_once(node, timeout_sec=0.3)
    if node.limit_state.get(side) is True:
        cur = node.read_current()
        print(f"ℹ️ {tgt_limit} 이미 ON. 현재 0x92 = {cur:.2f}° (이게 현재 프레임의 {side} 기준점)")
        return

    if ask(f"\n>>> {ax['label']} 를 {tgt_limit} 방향으로 {speed:.0f}dps 이동(센서 정지). 진행? [y/n]: ",
           ['y', 'n']) != 'y':
        print("취소."); return

    print(f"\n{tgt_limit} 방향 이동 시작 (Ctrl-C 정지)\n")
    stroke = ax['stroke_deg'] or 2000.0
    timeout = max(20.0, stroke / max(speed, 1.0) + 15.0)
    t0 = time.monotonic()
    last_req = last_spd = last_print = 0.0
    result = None
    other_cleared = False   # 시작 시 붙어있던 반대편 리미트를 벗어났는지 (벗어나기 전엔 무시)
    try:
        while time.monotonic() - t0 < timeout:
            now = time.monotonic()
            if now - last_spd > 0.2:                 # 속도 명령 주기 갱신
                node.send_speed(sign * speed)
                last_spd = now
            if now - last_req > ENC_REQ_PERIOD:
                node.request_encoder()
                last_req = now
            rclpy.spin_once(node, timeout_sec=0.05)

            if node.limit_state.get(side) is True:
                node.send_stop()
                result = ('LIMIT', node.cur_deg)
                break
            # 반대편 리미트: 시작 시 붙어있던 경우는 벗어날 때까지 무시,
            # 한 번 off된(=벗어난) 뒤 다시 ON이면 진짜 역방향 → 정지
            if LIMIT_TOPIC[axis_key][other]:
                if node.limit_state.get(other) is False:
                    other_cleared = True
                elif node.limit_state.get(other) is True and other_cleared:
                    node.send_stop()
                    result = ('WRONG_LIMIT', node.cur_deg)
                    break
            if now - last_print > 0.5 and node.cur_deg is not None:
                print(f"  현재 {node.cur_deg:8.2f}°   {tgt_limit}="
                      f"{'ON' if node.limit_state.get(side) else 'off'}")
                last_print = now
        else:
            node.send_stop()
            result = ('TIMEOUT', node.cur_deg)
    except KeyboardInterrupt:
        node.send_stop()
        result = ('ABORT', node.cur_deg)

    node.send_stop()
    # 정지 후 정착값 재측정
    time.sleep(0.3)
    settled = node.read_current()
    print("\n" + "=" * 60)
    kind, at = result
    if kind == 'LIMIT':
        print(f"✅ {tgt_limit} 도달 정지. 현재 프레임의 {side} 기준점:")
        print(f"   0x92 = {settled:.2f}°   (정지순간 {at:.2f}°)")
        rec = ax['ref'].get(side)
        if rec is not None:
            print(f"   2026-05-29 기록값 {rec:.2f}° 대비 차이 {settled - rec:+.2f}°")
        print(f"\n   → config persistent_ref_{axis_key} 후보값: {settled:.2f}")
    elif kind == 'WRONG_LIMIT':
        print(f"⚠️ 반대편({other}) 리미트로 정지 @ {settled:.2f}° → LIMIT_DIR 부호 점검 필요")
    elif kind == 'TIMEOUT':
        print(f"⏱️ 타임아웃 @ {settled:.2f}° (리미트 미도달)")
    else:
        print(f"⏹️ 중단 @ {settled:.2f}°")
    print("=" * 60)


def ask(prompt, choices):
    while True:
        v = input(prompt).strip().lower()
        if v in choices:
            return v
        print(f"  → {choices} 중에서 입력하세요.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('axis', nargs='?', choices=list(AXES.keys()))
    ap.add_argument('side', nargs='?', choices=['min', 'max'])
    ap.add_argument('--to-limit', action='store_true',
                    help='기록값 무시. 속도명령으로 리미트까지 가서 센서로 멈추고 현재 0x92 측정 '
                         '(현재 프레임 기준점 재측정용, 호밍 방식)')
    args = ap.parse_args()

    rclpy.init()

    # 1) 축 선택
    axis_key = args.axis or ask("축 선택 [x/y/z/yaw]: ", list(AXES.keys()))
    node = GotoAbsLimit(axis_key)
    ax = node.ax
    print(f"\n=== {ax['label']} 선택 ===")

    # 2) 현재 절대 엔코더 출력
    print("현재 0x92 절대 엔코더 읽는 중...")
    cur = node.read_current()
    if cur is None:
        print("❌ 현재 엔코더 값 수신 실패 (robot-control 동작/CAN 확인). 종료.")
        node.send_stop()
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    print(f"  현재 위치: {node.fmt(cur)}")

    # 3) min/max 선택
    side = args.side or ask("이동 목표 [min/max]: ", ['min', 'max'])

    # --to-limit: 기록값 무시, 속도명령으로 리미트까지 가서 현재 프레임 기준점 측정
    if args.to_limit:
        run_to_limit(node, axis_key, side)
        node.send_stop()
        node.destroy_node(); rclpy.shutdown(); return

    target = ax['ref'][side]

    if target is None:
        print(f"\n⚠️ {ax['label']}의 {side} 기록값이 없습니다 (미측정).")
        manual = input("   목표 0x92 각도를 직접 입력(deg), 취소는 Enter: ").strip()
        if not manual:
            print("취소.")
            node.destroy_node(); rclpy.shutdown(); return
        try:
            target = float(manual)
        except ValueError:
            print("숫자 아님. 취소."); node.destroy_node(); rclpy.shutdown(); return

    delta = target - cur
    print(f"\n  목표({side}): {node.fmt(target)}")
    print(f"  이동량: {delta:+.2f}°  방향: {'증가(+)' if delta > 0 else '감소(-)'}")

    # 안전: 스트로크 초과 이동 차단
    stroke = ax['stroke_deg']
    if stroke and abs(delta) > stroke * 1.15:
        print(f"❌ 이동량 {abs(delta):.0f}° 이 스트로크 {stroke:.0f}°의 115%를 초과 → "
              f"기록값 불일치 의심. 안전상 중단.")
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    # 목표 방향의 리미트 이미 ON?
    tgt_limit = LIMIT_TOPIC[axis_key][side]
    rclpy.spin_once(node, timeout_sec=0.3)
    if tgt_limit and node.limit_state.get(side) is True:
        print(f"  ℹ️ {tgt_limit} 리미트가 이미 ON 상태 (이미 {side} 부근).")

    # 속도
    sp_in = input(f"이동 속도(dps), 기본 {ax['default_speed']:.0f}: ").strip()
    speed = float(sp_in) if sp_in else ax['default_speed']

    if ask(f"\n>>> {ax['label']} 를 {node.fmt(target)} 로 {speed:.0f}dps 이동. 진행? [y/n]: ",
           ['y', 'n']) != 'y':
        print("취소."); node.destroy_node(); rclpy.shutdown(); return

    # 4) 이동 + 리미트 모니터링
    print("\n이동 시작. (Ctrl-C 정지)\n")
    node.send_abs(target, speed)

    est = abs(delta) / max(speed, 1.0) + 10.0
    timeout = max(15.0, min(est, 120.0))
    t0 = time.monotonic()
    last_req = 0.0
    last_print = 0.0
    result = None
    try:
        while time.monotonic() - t0 < timeout:
            now = time.monotonic()
            if now - last_req > ENC_REQ_PERIOD:
                node.request_encoder()
                last_req = now
            rclpy.spin_once(node, timeout_sec=0.05)

            # 목표 리미트 도달 → 정지 (검증 성공 핵심)
            if tgt_limit and node.limit_state.get(side) is True:
                node.send_stop()
                result = ('LIMIT', node.cur_deg)
                break
            # 반대편 리미트 도달 → 안전 정지 (방향 이상)
            other = 'max' if side == 'min' else 'min'
            if LIMIT_TOPIC[axis_key][other] and node.limit_state.get(other) is True:
                node.send_stop()
                result = ('WRONG_LIMIT', node.cur_deg)
                break
            # 목표각 도달
            if node.cur_deg is not None and abs(node.cur_deg - target) <= REACH_TOL_DEG:
                node.send_stop()
                result = ('TARGET', node.cur_deg)
                break

            if now - last_print > 0.5 and node.cur_deg is not None:
                rem = target - node.cur_deg
                lm = node.limit_state.get(side)
                print(f"  현재 {node.cur_deg:8.2f}°  남음 {rem:+7.2f}°  "
                      f"{tgt_limit}={'ON' if lm else 'off'}")
                last_print = now
        else:
            node.send_stop()
            result = ('TIMEOUT', node.cur_deg)
    except KeyboardInterrupt:
        node.send_stop()
        result = ('ABORT', node.cur_deg)

    # 결과 보고
    print("\n" + "=" * 60)
    kind, at = result
    at_s = f"{at:.2f}°" if at is not None else "?"
    if kind == 'LIMIT':
        err = (at - target) if at is not None else None
        print(f"✅ 리미트 도달 정지: {tgt_limit} ON @ 0x92={at_s}")
        if err is not None:
            print(f"   기록값 {target:.2f}° 대비 오차 {err:+.2f}°  "
                  f"→ {'일치(검증 OK)' if abs(err) <= 5 else '⚠️ 불일치(재측정 권장)'}")
    elif kind == 'TARGET':
        lm = node.limit_state.get(side)
        print(f"✅ 목표 절대각 도달: 0x92={at_s} (기록값 {target:.2f}°)")
        print(f"   목표 리미트 {tgt_limit}: {'ON (검증 OK)' if lm else 'off ⚠️ (리미트 미도달=값 어긋남 가능)'}")
    elif kind == 'WRONG_LIMIT':
        print(f"⚠️ 반대편 리미트 트리거로 정지 @ 0x92={at_s} → 방향/부호 점검 필요")
    elif kind == 'TIMEOUT':
        print(f"⏱️ 타임아웃 정지 @ 0x92={at_s} (목표 {target:.2f}°)")
    else:
        print(f"⏹️ 사용자 중단 @ 0x92={at_s}")
    print("=" * 60)

    node.send_stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
