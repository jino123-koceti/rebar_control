#!/usr/bin/env python3
"""데크끝 감지 기반 전진↔후진 자율주행 테스트 (리모콘 연동).

판정: **철근 배근(rebar_h/v)이 이어진 곳만 주행가능** (무한궤도가 철근 위를 밟고 감).
  방수포(background)·floor·wall·obstacle 은 전부 주행불가 → 배근 끊기면 감속·정지.
  rebar_frac(배근이 이어진 세로범위) 실측: 정상 0.55~0.68 / 데크끝 0.15~0.45
  → front/back **공통 임계**(stop 0.45 / slow 0.55). 구 deck_frac은 카메라별 임계가
    필요했으나(방수포 포함이라 baseline이 갈림) 철근기준은 카메라 무관.

리모콘 체계:
  S20(auto mode) 유지 + S23 → 전진 시작 → 전면 데크끝 감지 시 감속→정지
    → 후진 시작 → 후면 데크끝 감지 시 감속→정지 → 완료
  S24 → 즉시 정지(abort).  리모콘 emergency_stop 버튼은 drive_controller가 독립 처리.

⚠️ 안전:
  - 기본 **dry-run**(모션 없음, 판정+의도 cmd_vel만 로그). 실제 구동은 `--arm`.
  - **A단계 권장**: 궤도 들고(공중) --arm 로 상태전이 검증 후에만 지면(B).
  - 저속(--speed 0.05), staleness 워치독(seg 끊기면 정지), max-sec 타임아웃.
  - drive_controller가 cmd_vel 0.5s 끊기면 자동정지(이 노드 죽어도 안전).
  - 임계는 rebar_frac 기준 공통 0.45/0.55. 물리 정지거리는 아직 미환산 → depth(2단계) 권장.

  python3 tools/drive/deck_edge_drive_test.py            # dry-run (모션X)
  python3 tools/drive/deck_edge_drive_test.py --arm      # 실제 구동 (궤도 공중부터!)
    [--speed 0.05] [--front-stop 0.45 --front-slow 0.55] [--back-stop 0.45 --back-slow 0.55]
    [--settle 1.5] [--seg-hz 6] [--max-sec 40]
"""
import os
import sys
import time
import threading
import argparse
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
from rebar_base_interfaces.msg import RemoteControl

ROOT = '/home/koceti/ros2_ws'
# ⚠ 이 스크립트는 노드화 이전의 개발용 시제품이다. 실주행 정본은
#   `rebar_vision/rebar_drive_node.py`(주행) + `deck_edge_node.py`(판정).
#   여기 남겨둔 이유는 ROS 노드 없이 단독 실행해 판정을 빠르게 확인하기 위함.
sys.path.insert(0, os.path.join(ROOT, 'src/rebar_vision'))
from rebar_vision.rebar_seg import RebarSegmenter                    # noqa: E402
from rebar_vision.deck_edge import rebar_edge, band_features, VerdictFSM  # noqa: E402

FRONT_TOPIC = '/zedxmini2/zed_node/rgb/color/rect/image/compressed'
BACK_TOPIC = '/zedxmini1/zed_node/rgb/color/rect/image/compressed'
WEIGHTS = os.path.join(ROOT, 'src/rebar_vision/model/retrain_best_260804.pt')
# 리모콘 buttons 인덱스 (can_parser): [s13,s14,s17,s18,s21,s22,s23,s24]
S23_I, S24_I = 6, 7


class DeckEdgeDriveTest(Node):
    def __init__(self, a):
        super().__init__('deck_edge_drive_test')
        self.a = a
        self.dry = not a.arm
        # 상태: IDLE→FWD→FWD_SETTLE→REV→REV_SETTLE→DONE / ABORT
        self.state = 'IDLE'
        self.state_t = time.time()
        self.msg = {'front': None, 'back': None}     # 최신 compressed
        self.seg = RebarSegmenter(WEIGHTS, a.device, a.imgsz)
        self.fsm = {'front': VerdictFSM(a.front_stop, a.front_slow, a.smooth),
                    'back': VerdictFSM(a.back_stop, a.back_slow, a.smooth)}
        self.verdict = 'GO'
        self.rebar_frac = 1.0
        self.hard_why = ''
        self.last_seg_t = 0.0
        # 리모콘 상태
        self.s20 = False
        self.s24 = False
        self.estop = False
        self.prev_s23 = 0
        self.s23_edge = False
        self.br = None

        self.create_subscription(CompressedImage, FRONT_TOPIC,
                                 lambda m: self._store('front', m), qos_profile_sensor_data)
        self.create_subscription(CompressedImage, BACK_TOPIC,
                                 lambda m: self._store('back', m), qos_profile_sensor_data)
        self.create_subscription(RemoteControl, '/remote_control',
                                 self._remote_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        self.timer = self.create_timer(1.0 / a.rate, self._tick)   # 제어 15Hz
        self.get_logger().warn(
            f"데크끝 주행 테스트 {'[DRY-RUN 모션X]' if self.dry else '[ARM 실구동]'} "
            f"speed={a.speed} front(stop{a.front_stop}/slow{a.front_slow}) "
            f"back(stop{a.back_stop}/slow{a.back_slow})")
        self.get_logger().warn(
            f"S20(auto)+S23=시작({a.start}), S24=정지, 리모콘 estop=하드정지")
        if a.speed > 0.25:
            self.get_logger().error(
                f"⚠ 고속 {a.speed}m/s: 데크끝 반응거리 부족 위험(정지거리 미환산) → "
                f"물리 백스톱 필수 + S24 대기. 후진(back margin 큼)/여유공간부터 권장.")
        # seg 별도 스레드 (제어 타이머가 seg에 안 막혀 cmd_vel 15Hz 유지 → 매끄러운 주행)
        self._stop = False
        self._seg_thread = threading.Thread(target=self._seg_worker, daemon=True)
        self._seg_thread.start()

    def _begin(self):
        """S23 시작 → args.start 방향으로 진입 (fwd=전진, rev=후진 단독)."""
        # staleness 워치독 기준시각을 지금으로 리셋 → 첫 프레임에 stale_sec 유예.
        # (안 하면 last_seg_t=0이라 첫 tick에 즉시 오판 abort)
        self.last_seg_t = time.time()
        self.verdict = 'GO'
        if self.a.start == 'rev':
            self.fsm['back'] = VerdictFSM(self.a.back_stop, self.a.back_slow, self.a.smooth)
            self.get_logger().warn('🟢 S23 → 후진 시작 (start=rev)')
            self._set_state('REV')
        else:
            self.fsm['front'] = VerdictFSM(self.a.front_stop, self.a.front_slow, self.a.smooth)
            self.get_logger().warn('🟢 S23 → 전진 시작')
            self._set_state('FWD')

    def _store(self, cam, m):
        self.msg[cam] = m

    def _remote_cb(self, m):
        self.s20 = bool(m.switch_s20)
        self.estop = bool(m.emergency_stop)
        b = list(m.buttons)
        s23 = b[S23_I] if len(b) > S23_I else 0
        self.s24 = bool(b[S24_I]) if len(b) > S24_I else False
        if self.prev_s23 == 0 and s23 == 1:
            self.s23_edge = True                     # rising edge 래치
        self.prev_s23 = s23

    def _decode(self, cam):
        m = self.msg[cam]
        if m is None:
            return None
        arr = np.frombuffer(m.data, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _run_seg(self, cam):
        img = self._decode(cam)
        if img is None:
            return None
        mask = self.seg.predict(img)
        _, rebar_frac, on_rebar = rebar_edge(mask)          # 철근배근 기반 판정
        obs = band_features(mask)['obs_nearmid']
        hard = (not on_rebar) or obs > self.a.obs           # 배근이탈/장애물 = 즉시정지
        vd, sm, _ = self.fsm[cam].step(rebar_frac, hard)
        self.last_seg_t = time.time()
        self.hard_why = ('배근이탈' if not on_rebar else
                         (f'장애물 {obs:.2f}' if hard else ''))
        return vd, sm

    def _seg_worker(self):
        """별도 스레드: 활성 카메라 seg → verdict 갱신. 제어 타이머(cmd_vel 15Hz)를 안 막음."""
        while not self._stop:
            if self.state in ('FWD', 'REV'):
                cam = 'front' if self.state == 'FWD' else 'back'
                try:
                    r = self._run_seg(cam)
                    if r is not None:
                        self.verdict, self.rebar_frac = r
                except Exception as e:
                    self.get_logger().error(f'seg 오류: {e}')
            time.sleep(1.0 / self.a.seg_hz)

    def _set_state(self, s):
        if s != self.state:
            self.get_logger().warn(f"상태: {self.state} → {s}")
        self.state = s
        self.state_t = time.time()

    def _abort(self, why):
        self.get_logger().error(f"⛔ ABORT: {why} → 정지")
        self._publish(0.0)
        self._set_state('ABORT')

    def _publish(self, vx):
        """cmd_vel 발행 (dry-run이면 0). auto 모드 유지."""
        if not self.dry and self.state in ('FWD', 'REV', 'FWD_SETTLE', 'REV_SETTLE'):
            mm = String(); mm.data = 'auto'; self.mode_pub.publish(mm)
        t = Twist()
        t.linear.x = 0.0 if self.dry else float(vx)
        self.cmd_pub.publish(t)
        if self.dry and abs(vx) > 1e-6:
            pass  # dry-run: 의도속도는 로그로만

    def _vel_for(self, vd):
        base = self.a.speed
        if vd == 'STOP':
            return 0.0
        return base * (self.a.slow_scale if vd == 'SLOW' else 1.0)

    def _tick(self):
        now = time.time()
        # --- 공통 안전 게이트 ---
        if self.estop:
            if self.state not in ('IDLE', 'ABORT', 'DONE'):
                self._abort('리모콘 emergency_stop')
            self._publish(0.0); return
        if self.s24 and self.state not in ('IDLE', 'ABORT', 'DONE'):
            self._abort('S24'); return
        if self.state in ('FWD', 'REV', 'FWD_SETTLE', 'REV_SETTLE'):
            if not self.s20:
                self._abort('S20 해제(auto 이탈)'); return
            if now - self.state_t > self.a.max_sec:
                self._abort(f'max-sec {self.a.max_sec}s 초과'); return

        # --- 상태머신 ---
        if self.state == 'IDLE':
            self._publish(0.0)
            if self.s23_edge and self.s20:
                self.s23_edge = False
                self._begin()
            else:
                self.s23_edge = False
            return

        if self.state in ('FWD', 'REV'):
            cam = 'front' if self.state == 'FWD' else 'back'
            direction = 1.0 if self.state == 'FWD' else -1.0
            # seg는 별도 스레드(_seg_worker)가 self.verdict 갱신 → 제어 타이머는 cmd_vel만
            # 15Hz로 끊김없이 발행(seg에 안 막힘 = drive_controller 워치독 안 걸림 = 매끄러운 주행).
            # staleness 워치독 (worker가 last_seg_t 갱신; seg 끊기면 정지)
            if now - self.last_seg_t > self.a.stale_sec:
                self._abort(f'{cam} seg staleness {self.a.stale_sec}s'); return
            vx = direction * self._vel_for(self.verdict)
            self._publish(vx)
            self.get_logger().info(
                f"[{self.state}] {cam} {self.verdict} rebar_frac={self.rebar_frac:.2f} "
                f"{self.hard_why} vx={0.0 if self.dry else vx:+.3f}"
                f"{' (dry)' if self.dry else ''}",
                throttle_duration_sec=0.5)
            if self.verdict == 'STOP':
                self.get_logger().warn(
                    f'🛑 {cam} 배근 끊김(데크끝) 정지 {self.hard_why}')
                self._set_state('FWD_SETTLE' if self.state == 'FWD' else 'REV_SETTLE')
            return

        if self.state == 'FWD_SETTLE':
            self._publish(0.0)
            if now - self.state_t >= self.a.settle:
                self.fsm['back'] = VerdictFSM(self.a.back_stop, self.a.back_slow, self.a.smooth)
                self.get_logger().warn('🔄 후진 시작')
                self._set_state('REV')
            return

        if self.state == 'REV_SETTLE':
            self._publish(0.0)
            if now - self.state_t >= self.a.settle:
                self.get_logger().warn('✅ 시퀀스 완료')
                self._set_state('DONE')
            return

        # DONE / ABORT: 정지 유지 + auto 이탈(idle)
        self._publish(0.0)
        mm = String(); mm.data = 'idle'; self.mode_pub.publish(mm)
        # S23 재입력 시 재시작 허용 (args.start 방향)
        if self.s23_edge and self.s20 and not self.estop and not self.s24:
            self.s23_edge = False
            self.get_logger().warn('🟢 S23 재입력 → 재시작')
            self._begin()
        else:
            self.s23_edge = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', action='store_true', help='실제 구동(기본 dry-run 모션X)')
    ap.add_argument('--speed', type=float, default=0.20, help='전/후진 속도 m/s (느리면 상향)')
    ap.add_argument('--start', choices=['fwd', 'rev'], default='fwd',
                    help='S23 시작 방향: fwd=전진부터(전체), rev=후진 단독 테스트')
    ap.add_argument('--slow-scale', type=float, default=0.4, help='SLOW 시 속도배율')
    # rebar_frac 임계 — 철근기준이라 front/back 공통 (실측 정상 0.55~0.68 / 데크끝 0.15~0.45)
    ap.add_argument('--front-stop', type=float, default=0.45)
    ap.add_argument('--front-slow', type=float, default=0.55)
    ap.add_argument('--back-stop', type=float, default=0.45)
    ap.add_argument('--back-slow', type=float, default=0.55)
    ap.add_argument('--obs', type=float, default=0.06,
                    help='전방밴드 obstacle/human 비율>이 값 → 즉시 STOP')
    ap.add_argument('--smooth', type=int, default=5)
    ap.add_argument('--seg-hz', type=float, default=8.0, help='seg 추론 주기(고속일수록↑ 반응)')
    ap.add_argument('--stale-sec', type=float, default=1.0, help='seg 끊김 정지 임계')
    ap.add_argument('--settle', type=float, default=1.5, help='정지 후 대기')
    ap.add_argument('--max-sec', type=float, default=180.0,
                    help='한 방향 최대 주행시간(안전 백스톱). 데크 통과시간보다 길게')
    ap.add_argument('--rate', type=float, default=15.0, help='제어 루프 Hz')
    ap.add_argument('--imgsz', type=int, default=512)
    ap.add_argument('--device', default=None)
    args = ap.parse_args()
    import torch
    args.device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')

    try:
        rclpy.init()
    except Exception:
        pass
    from rclpy.executors import ExternalShutdownException
    node = DeckEdgeDriveTest(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 종료 시 확실히 정지 + seg 스레드 종료
        node._stop = True
        try:
            t = Twist(); node.cmd_pub.publish(t)
            mm = String(); mm.data = 'idle'; node.mode_pub.publish(mm)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
