#!/usr/bin/env python3
"""Z 에러를 감시해 **자동으로 리셋하고 원점까지 복원**한다.

## 무엇을 푸는가

Z 축(모터 ID 는 `axes.yaml`)이 `errorState=0x4000`(엔코더 데이터 오류)에 빠지면 **브레이크가 안 풀리고
위치 명령에 반응하지 않는다.** 2026-10-07 에 반복해서 났고 증상이 멀다 —
결속 9점이 `z_down 타임아웃 45s` 로 전부 실패하고, 다음 호밍은 시작하자마자
`브레이크가 6.0초 동안 풀리지 않았다` 로 죽는다. 온도는 25°C 로 멀쩡하다.

복구는 **`0x76`(System Reset)** 이다 — `0x9B` 는 V4.3 에 없는 명령이라 효과가 없다.
사람이 매번 명령을 치는 대신 이 노드가 감시하다 자동으로 복구한다.

## 원점을 잃지 않는 법

`0x76` 은 멀티턴을 날린다(리셋 뒤 `multi == single` 이 된다). 원점은 모터 각도로
들고 있으므로 그냥 리셋하면 **`mm` 환산이 조용히 틀어진다** — 그게 에러보다 위험하다.

그런데 리셋 중에는 브레이크가 잠겨 있어 **축이 움직이지 않는다.** 그러면 리셋
전후 멀티턴 차이가 곧 "좌표 프레임이 얼마나 밀렸는가" 이고, 원점에 그만큼 더하면
환산이 보존된다:

    new_ref = old_ref + (multi_after − multi_before)

⚠⚠ **그 전제를 반드시 검증한다.** 단회전은 절대 엔코더라 리셋에 영향받지 않는다 —
  전후가 같으면 "안 움직였다" 가 증명된다. 다르면 **복원을 포기하고 재호밍을
  요구한다.** 틀린 원점으로 Z 를 내리면 철근을 찍는다.

⚠ 전체 1회전(32.6mm)의 정확한 배수만큼 움직인 경우는 단회전으로 못 잡는다.
  브레이크가 잠긴 1.5초 동안 그럴 일은 없다고 본다 — 다만 기록해 둔다.

## 왜 별도 노드인가

`position_control_node` 안에 넣으면 그 노드가 더 커지고(이미 한도의 4배다) 복구
정책이 하드웨어 계층에 섞인다. 리허설 동안 쓰는 보조 장치이므로 떼었다 붙이기
쉬운 쪽이 낫다 (`use_z_watchdog:=false`).

구독  /motor_error              String(JSON)  {id, state, motor_c, mos_c, volt}
      /motor_0x<Z>/encoder_single · _multi     Int32  (ID 는 axes.yaml)
발행  /brake_cmd                String        'recover z'
      /homing/ref_shift         String(JSON)  {axis, delta_deg, why}
      /z_watchdog/status        String(JSON)
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

AXIS = 'z'
# ⚠ **모터 ID 를 여기 적지 않는다.** `axes.yaml` 이 유일한 출처다 — 두 곳에 적으면
#   ID 를 바꿀 때 한쪽이 남는다 (검사기 R1 이 이것을 막는다).
try:
    from .axis_config import load_axis_motor_ids
    CAN_ID = int(load_axis_motor_ids([AXIS])[AXIS], 16)
except Exception:                       # 설정을 못 읽으면 감시를 포기한다
    CAN_ID = None
CPR = 262144
CPD = CPR / 360.0              # counts per 모터축 1도


class ZWatchdog(Node):
    def __init__(self):
        super().__init__('z_watchdog')
        # 복구를 시도할 에러 비트. 0 이면 **모든** 에러에 반응한다.
        #   기본은 0x4000(엔코더 데이터 오류) 만 — 과온도·과전류에 리셋을 거는 것은
        #   원인을 덮는 짓이다 (식혀야 하거나 부하를 줄여야 한다).
        self.declare_parameter('error_mask', 0x4000)
        # 연속 복구 사이 최소 간격. 짧게 두면 못 낫는 고장에 리셋을 난사한다.
        self.declare_parameter('cooldown_sec', 15.0)
        # 몇 번까지 시도하고 포기하나. 포기 뒤에는 사람이 봐야 한다.
        self.declare_parameter('max_tries', 5)
        # 리셋 뒤 재부팅·재열거를 기다리는 시간 (`recover` 가 내부에서 1초 쉰다)
        self.declare_parameter('settle_sec', 2.5)
        # 단회전이 이보다 더 움직였으면 "축이 돌았다" 로 보고 복원을 포기한다.
        #   50 counts = 모터축 0.07° = Z 0.006mm.
        self.declare_parameter('single_tol_counts', 50)

        g = self.get_parameter
        self.mask = int(g('error_mask').value)
        self.cooldown = float(g('cooldown_sec').value)
        self.max_tries = int(g('max_tries').value)
        self.settle = float(g('settle_sec').value)
        self.tol = int(g('single_tol_counts').value)

        if CAN_ID is None:
            # 설정을 못 읽으면 감시를 켜지 않는다 — 엉뚱한 ID 로 리셋을 쏘는 것이
            # 감시를 못 하는 것보다 나쁘다
            self.get_logger().error(
                'axes.yaml 에서 Z 모터 ID 를 못 읽었다 — 감시를 켜지 않는다')
            return
        self.single = None
        self.multi = None
        self.state = None
        self.tries = 0
        self.last_try = 0.0
        self.pending = None        # 리셋 직후 검증 대기 (s0, m0, t)
        self.detail = '대기'

        self.brake_pub = self.create_publisher(String, '/brake_cmd', 10)
        self.shift_pub = self.create_publisher(String, '/homing/ref_shift', 10)
        self.status_pub = self.create_publisher(String, '/z_watchdog/status', 10)
        self.create_subscription(String, '/motor_error', self._on_err, 10)
        self.create_subscription(
            Int32, f'/motor_0x{CAN_ID:03x}/encoder_single',
            lambda m: setattr(self, 'single', int(m.data)), 10)
        self.create_subscription(
            Int32, f'/motor_0x{CAN_ID:03x}/encoder_multi',
            lambda m: setattr(self, 'multi', int(m.data)), 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f"Z 감시 시작 — 0x{CAN_ID:03X} 에러 0x{self.mask:04X} 를 보면 "
            f"0x76 리셋하고 원점을 복원한다 (최대 {self.max_tries}회, "
            f"간격 {self.cooldown:.0f}초)")

    # ---- 입력 --------------------------------------------------------------
    def _on_err(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if int(d.get('id', 0)) != CAN_ID:
            return
        self.state = int(d.get('state', 0))
        if self.state:
            self.get_logger().warning(
                f"Z 에러 0x{self.state:04X} — 모터 {d.get('motor_c')}°C / "
                f"MOS {d.get('mos_c')}°C / {d.get('volt')}V")

    # ---- 동작 --------------------------------------------------------------
    def _tick(self):
        if self.pending is not None:
            self._verify()
            return
        hit = bool(self.state) and (not self.mask or (self.state & self.mask))
        if not hit:
            if self.state == 0 and self.tries:
                self.get_logger().info('Z 에러 해소 — 시도 횟수를 초기화한다')
                self.tries = 0
                self.detail = '정상'
            self._publish()
            return
        now = time.time()
        if now - self.last_try < self.cooldown:
            return
        if self.tries >= self.max_tries:
            self.detail = (f'포기 — {self.tries}회 복구했는데 에러가 돌아온다. '
                           f'사람이 Z 엔코더 배선을 봐야 한다')
            self._publish()
            return
        if self.single is None or self.multi is None:
            self.detail = '엔코더 값을 못 받고 있다 — 복구를 미룬다'
            self._publish()
            return
        self.tries += 1
        self.last_try = now
        self.pending = (self.single, self.multi, now)
        self.detail = f'복구 시도 {self.tries}/{self.max_tries} — 0x76 리셋'
        self.get_logger().warning(
            f"{self.detail} (단회전 {self.single}, 멀티턴 {self.multi})")
        self.brake_pub.publish(String(data=f'recover {AXIS}'))
        self._publish()

    def _verify(self):
        s0, m0, t0 = self.pending
        if time.time() - t0 < self.settle:
            return
        self.pending = None
        if self.single is None or self.multi is None:
            self.detail = '리셋 뒤 엔코더 값이 없다 — 원점 복원 못 함, 재호밍 필요'
            self.get_logger().error(self.detail)
            return self._publish()
        # ⚠ 단회전이 그대로여야 "안 움직였다" 가 증명된다
        ds = (self.single - s0 + CPR // 2) % CPR - CPR // 2
        if abs(ds) > self.tol:
            self.detail = (f'⚠ 리셋 중 축이 움직였다 (단회전 {ds:+d} counts = '
                           f'모터축 {ds / CPD:+.2f}°) — 원점 복원을 포기한다. '
                           f'**전체 호밍 필요**')
            self.get_logger().error(self.detail)
            return self._publish()
        delta = (self.multi - m0) / CPD
        if abs(delta) < 1e-6:
            self.detail = '복구 완료 — 멀티턴 프레임이 그대로라 원점도 그대로'
            self.get_logger().info(self.detail)
            return self._publish()
        self.shift_pub.publish(String(data=json.dumps({
            'axis': AXIS, 'delta_deg': delta,
            'why': f'0x76 리셋 (단회전 {ds:+d} counts 로 무이동 확인)',
        })))
        self.detail = (f'복구 완료 — 원점을 모터축 {delta:+.2f}° 보정했다 '
                       f'(단회전 {ds:+d} counts). 재호밍 불필요')
        self.get_logger().warning(self.detail)
        self._publish()

    def _publish(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'tries': self.tries,
            'max_tries': self.max_tries, 'detail': self.detail,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    n = ZWatchdog()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
