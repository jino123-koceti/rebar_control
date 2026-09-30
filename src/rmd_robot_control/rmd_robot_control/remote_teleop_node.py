#!/usr/bin/env python3
"""L3 — 리모콘 입력을 축 명령으로 바꾼다.

`/remote_control`(L1 `remote_bridge` 발행)만 구독한다. **CAN 을 직접 읽지 않는다** —
조작 노드가 하드웨어를 겸하면 계층이 무너지고 같은 버스를 두 곳에서 열 수 있게 된다
(YEAR3_ARCHITECTURE.md §3). 2026-09-29 이전 구현은 can3 를 직접 읽었고 그걸 걷어냈다.

──────────────────────────────────────────────────────────────────────────
조작 (2차년도 iron_md_teleop 방식 + S13/S14·S23/S24 만 3차년도에서 변경)

  AN3   하부체 전후진      AN3− = 전진, AN3+ = 후진   → /cmd_vel linear.x
  AN4   하부체 좌우회전    AN4+ = CCW,  AN4− = CW     → /cmd_vel angular.z
  AN1   상부체 X축 속도                              → /joint_3/speed (0x145)
  AN2   상부체 Y축 속도                              → /joint_4/speed (0x146)

  AN3/AN4 의 부호가 뒤집힌 것은 좌우 주행모터가 180도 반대로 설치되어서다
  (2차년도와 동일 — 3차년도도 같은 것으로 확인).

  S19   Remote 모드 — 이 노드가 조작을 통과시킨다
  S20   Auto 모드   — 스틱 입력 무시, 정지 유지
  S17   횡이동 +1스텝 (좌측 50mm)   → /lateral/step
  S18   횡이동 −1스텝 (우측 50mm)   → /lateral/step
  S13   누르고 있는 동안 Z축 상승   → /joint_5/speed (0x147)
  S14   누르고 있는 동안 Z축 하강   → /joint_5/speed (0x147)

  비상정지 → 전 축 정지. `remote_bridge` 가 판정해서 넘겨준다.
  송신기 꺼짐·START 전(DATA[0]=0x00)도 비상정지로 들어온다.

  미구현: S21/S22(작업 시퀀스), S23/S24(자율주행 시작·정지 — L4 가 받는다)
──────────────────────────────────────────────────────────────────────────

Z축 주의: 리프팅축이라 브레이크를 풀면 자중으로 내려앉을 수 있다. 브레이크 해제는
이 노드가 하지 않는다 (axes.yaml 의 never_auto_release).
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, String
from rebar_base_interfaces.msg import RemoteControl

# RemoteControl.buttons 순서 — remote_bridge 와 같아야 한다
BUTTON_ORDER = ('S13', 'S14', 'S17', 'S18', 'S21', 'S22', 'S23', 'S24')


class RemoteTeleop(Node):
    def __init__(self):
        super().__init__('remote_teleop_node')

        # position_control_node 의 상한과 맞춘다. 더 크게 줘도 노드가 잘라낸다.
        self.declare_parameter('max_linear_vel', 0.25)     # m/s
        self.declare_parameter('max_angular_vel', 0.5)     # rad/s
        self.declare_parameter('xy_max_dps', 50.0)
        self.declare_parameter('z_dps', 50.0)
        self.declare_parameter('deadzone', 0.08)
        # /remote_control 을 받는 즉시 처리한다. 타이머는 **끊김 감시용**이다
        # (타이머 발행만 쓰면 브릿지 지연에 또 50ms 가 얹힌다 — 2026-09-30 실측)
        self.declare_parameter('watchdog_rate', 20.0)
        # /remote_control 이 이 시간 이상 끊기면 정지한다 (브릿지 사망·노드 분리)
        self.declare_parameter('remote_timeout', 0.5)
        self.declare_parameter('lateral_timeout', 25.0)

        g = self.get_parameter
        self.max_lin = float(g('max_linear_vel').value)
        self.max_ang = float(g('max_angular_vel').value)
        self.xy_max = float(g('xy_max_dps').value)
        self.z_dps = float(g('z_dps').value)
        self.deadzone = float(g('deadzone').value)
        self.remote_timeout = float(g('remote_timeout').value)
        self.lateral_timeout = float(g('lateral_timeout').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.x_pub = self.create_publisher(Float32, '/joint_3/speed', 10)   # 0x145
        self.y_pub = self.create_publisher(Float32, '/joint_4/speed', 10)   # 0x146
        self.z_pub = self.create_publisher(Float32, '/joint_5/speed', 10)   # 0x147
        self.lat_pub = self.create_publisher(Int32, '/lateral/step', 10)

        self.create_subscription(RemoteControl, '/remote_control', self._on_remote, 10)
        self.create_subscription(String, '/lateral/complete', self._on_lat_done, 10)

        self.remote = None
        self.last_remote = 0.0
        self.prev = {n: False for n in BUTTON_ORDER}
        self.lat_busy = False
        self.lat_started = 0.0
        self._warned = set()
        self._last_note = None
        self._pub_state = {}        # 토픽별 (마지막 발행값, 시각)
        # 공백 진단 (remote_bridge 와 같은 목적)
        self._gap_warn_sec = 0.15
        self._last_rx_log = 0.0

        # 타이머는 **감시만** 한다. tick() 을 여기서도 부르면 콜백과 합쳐 56 Hz 로
        # 이중 발행되어 백엔드 큐를 밀어낸다 (2026-09-30 실측).
        self.timer = self.create_timer(1.0 / float(g('watchdog_rate').value), self.watchdog)
        self.get_logger().info(
            f"리모콘 텔레옵 시작 — /remote_control 구독, "
            f"주행 {self.max_lin} m/s · {self.max_ang} rad/s, XY {self.xy_max} dps, Z {self.z_dps} dps")
        self.get_logger().info("S19=Remote 에서만 조작이 통과합니다 (S20=Auto 면 정지 유지)")

    # ---- 수신 --------------------------------------------------------------
    def _on_remote(self, msg):
        self.remote = msg
        now = time.time()
        if self._last_rx_log and now - self._last_rx_log > self._gap_warn_sec:
            self.get_logger().warning(
                f"/remote_control 수신 공백 {(now - self._last_rx_log)*1000:.0f}ms")
        self._last_rx_log = now
        self.last_remote = now
        self.tick()          # 받는 즉시 반영 — 타이머를 기다리지 않는다

    def _on_lat_done(self, msg):
        self.lat_busy = False
        self.lat_started = 0.0
        self.get_logger().info(f"횡이동 완료: {msg.data}")

    def _pub_if_changed(self, pub, key, value, eps, refresh=0.1):
        """값이 바뀔 때만(또는 refresh 주기마다) 발행한다.

        ⚠ 2026-09-30 실측: 모든 토픽을 36Hz 로 계속 쏘면 position_control_node 의
        단일 스레드 실행기에서 cmd_vel 콜백이 자리를 차지해 **상부 축 콜백이 3.2Hz 로
        굶었다**(주행은 36.6Hz). 그래서 상부 X/Y 만 조작이 0.3초 늦게 반응했다.
        제어값은 바뀔 때만 보내면 충분하고, 유지 구간은 낮은 주기로 새로 고친다.
        (cmd_vel 은 백엔드 워치독이 0.5초로 보고 있어 새로 고침이 필요하다)
        """
        now = time.time()
        prev = self._pub_state.get(key)
        if prev is not None and abs(prev[0] - value) <= eps and now - prev[1] < refresh:
            return False
        self._pub_state[key] = (value, now)
        if isinstance(value, float) and pub is self.cmd_pub:
            return True          # cmd_vel 은 호출부에서 Twist 로 만든다
        pub.publish(Float32(data=float(value)))
        return True

    def watchdog(self):
        """리모콘이 끊겼을 때만 개입한다. 평소 발행은 _on_remote 가 한다."""
        if self.remote is None or time.time() - self.last_remote > self.remote_timeout:
            self._stop("/remote_control 끊김")

    # ---- 주기 처리 ----------------------------------------------------------
    def _dead(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def tick(self):
        r = self.remote
        if r is None or time.time() - self.last_remote > self.remote_timeout:
            self._stop("/remote_control 끊김")
            return
        if r.emergency_stop:
            self._stop("비상정지 (또는 송신기 꺼짐·START 전)")
            return

        s19, s20 = r.switch_s10, r.switch_s20          # switch_s10 = S19 (2차년도 필드명)
        if not s19:
            self._stop("Auto 모드(S20)" if s20 else "모드 스위치 중립")
            return

        js = list(r.joysticks) + [0.0] * (4 - len(r.joysticks))
        an1, an2, an3, an4 = (self._dead(js[0]), self._dead(js[1]),
                              self._dead(js[2]), self._dead(js[3]))
        btn = {n: bool(v) for n, v in zip(BUTTON_ORDER, list(r.buttons) + [0] * 8)}

        # 주행 — AN3− 가 전진이므로 부호를 뒤집는다
        lin = float(-an3 * self.max_lin)
        ang = float(an4 * self.max_ang)                 # AN4+ = CCW
        now = time.time()
        prev = self._pub_state.get('cmd_vel')
        if (prev is None or abs(prev[0][0] - lin) > 0.004 or abs(prev[0][1] - ang) > 0.008
                or now - prev[1] > 0.1):
            self._pub_state['cmd_vel'] = ((lin, ang), now)
            t = Twist()
            t.linear.x = lin
            t.angular.z = ang
            self.cmd_pub.publish(t)

        # 상부 X/Y
        self._pub_if_changed(self.x_pub, 'x', float(an1 * self.xy_max), 0.5)
        self._pub_if_changed(self.y_pub, 'y', float(an2 * self.xy_max), 0.5)

        # Z축 — 누르고 있는 동안만. 둘 다 눌리면 정지
        z = 0.0 if btn['S13'] == btn['S14'] else (self.z_dps if btn['S13'] else -self.z_dps)
        self._pub_if_changed(self.z_pub, 'z', float(z), 0.5)

        # 횡이동 — 누른 순간에만, 완료까지 래치
        if self.lat_busy and self.lat_started and \
                time.time() - self.lat_started > self.lateral_timeout:
            self.lat_busy = False
            self.get_logger().warning(
                f"횡이동 완료 신호가 {self.lateral_timeout:.0f}초간 없어 래치를 풉니다 "
                "— lateral_node 확인 필요")
        for name, turns, label in (('S17', 1, '좌측'), ('S18', -1, '우측')):
            if btn[name] and not self.prev[name]:
                if self.lat_busy:
                    self.get_logger().info("횡이동 진행 중 — 무시")
                elif self.lat_pub.get_subscription_count() == 0:
                    self.get_logger().warning("/lateral/step 구독자 없음 — lateral_node 확인")
                else:
                    self.lat_busy = True
                    self.lat_started = time.time()
                    self.lat_pub.publish(Int32(data=turns))
                    self.get_logger().info(f"횡이동 {label} 50mm ({name})")

        # 미구현 토글은 한 번만 알린다
        for name, why in (('S21', '작업 시퀀스 미구현'), ('S22', '작업 시퀀스 미구현'),
                          ('S23', '자율주행 시작은 L4 담당'), ('S24', '자율주행 정지는 L4 담당')):
            if btn[name] and name not in self._warned:
                self._warned.add(name)
                self.get_logger().warning(f"{name}: {why}")

        self.prev = btn
        self._note(f"Remote  주행 {lin:+.2f}/{ang:+.2f}  "
                   f"XY {an1*self.xy_max:+.0f}/{an2*self.xy_max:+.0f}  Z {z:+.0f}")

    def _stop(self, reason):
        # 정지는 변화 기반을 거치지 않고 항상 보낸다 — 늦거나 빠지면 안 된다
        self.cmd_pub.publish(Twist())
        for pub in (self.x_pub, self.y_pub, self.z_pub):
            pub.publish(Float32(data=0.0))
        self._pub_state.clear()
        self._note(f"정지 — {reason}")

    def _note(self, msg):
        """상태가 바뀔 때만 로그를 남긴다 (20Hz 로 찍으면 로그가 못 쓰게 된다)."""
        if msg != self._last_note:
            self._last_note = msg
            self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RemoteTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node._stop("노드 종료")
                time.sleep(0.1)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
