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
        ⚠ X·Y 는 **리미트에 닿은 방향으로 안 나간다** (아래 "리미트 차단" 참고)

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

## 리미트 차단 (X·Y)

리모콘 조작은 `stage_node` 를 경유하지 않으므로 자세별 가동범위 검사를 못 받는다.
최소한 **리미트 센서에 닿은 방향으로는 더 안 나가게** 한다.

센서를 직접 읽지 않는다 — `safety_node` 가 이미 리미트를 축 방향으로 환산해
`/safety/state` 의 `blocked_axes` 에 담아 발행한다(`x_min` → `'x-'`). 같은 판정을
여기서 또 만들면 두 곳이 어긋난다.

⚠ **부호 규약이 둘이라 섞이기 쉽다.** `blocked_axes` 는 **mm 증감** 기준이고
(`'x+'` = mm 증가 = 원점에서 멀어짐), `/joint_N/speed` 는 **모터 dps** 다.
`axes.yaml` 의 `home_dir` 가 "어느 부호가 원점 방향인가" 를 주므로 그것으로 환산한다:

    mm 방향 = −sign(dps) × home_dir

X·Y 는 `home_dir` 가 +1 이라 **양수 dps 가 원점(x_min/y_min) 방향**이다.

리미트 토픽을 한 번도 못 받았으면(ezi_io 미기동) **막지 않고 한 번 경고**한다.
조작 자체를 못 하게 만드는 쪽이 더 위험하다.

Z 는 아직 이 차단에 넣지 않았다 — `blocked_axes` 에 `z±` 도 들어오므로 같은 방식으로
한 줄이면 되지만, 요청 범위가 X·Y 였다. [미적용]
"""

import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, String
from rebar_base_interfaces.msg import RemoteControl, SafetyState

from .axis_config import load_stage_axes

# RemoteControl.buttons 순서 — remote_bridge 와 같아야 한다
BUTTON_ORDER = ('S13', 'S14', 'S17', 'S18', 'S21', 'S22', 'S23', 'S24')


class RemoteTeleop(Node):
    def __init__(self):
        super().__init__('remote_teleop_node')

        # position_control_node 의 상한과 맞춘다. 더 크게 줘도 노드가 잘라낸다.
        self.declare_parameter('max_linear_vel', 0.25)     # m/s
        self.declare_parameter('max_angular_vel', 0.5)     # rad/s
        # [2026-10-03] 50 → 100. 사용자 요청으로 2배. 상부 X·Y 는 리드스크류라
        # 100dps 도 출력축으로는 X 29mm/s · Y 22mm/s 다 (mm_per_deg 0.2906/0.2227).
        self.declare_parameter('xy_max_dps', 100.0)
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
        # 제어 권한. **내 모드가 아니면 축·주행 명령을 내지 않는다.**
        # 2026-09-30: 호밍이 30dps 를 보내는 동안 이 노드가 같은 토픽에 "정지(0)" 를
        # 0.1초마다 새로 보내서, 모터가 30/0 을 번갈아 받아 툭툭 끊겼다 (로그에서
        # 0.0 이 102회, 30.0 이 103회로 정확히 1:1). 결국 호밍이 스톨로 실패했다.
        # ⚠ 권한을 잃으면 **정지를 한 번만** 보내고 그 뒤로는 조용히 있어야 한다.
        #   계속 0 을 보내는 것이 바로 그 문제의 원인이었다.
        self.create_subscription(String, '/control_mode', self._on_mode, 10)
        # 리미트 차단 — safety_node 가 환산해 둔 것을 쓴다 (위 "리미트 차단" 참고)
        self.ax = load_stage_axes(('x', 'y'))
        self.blocked = None           # None = 한 번도 못 받았다 (막지 않는다)
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 1)
        self._limit_noted = set()

        self.remote = None
        self.last_remote = 0.0
        self.prev = {n: False for n in BUTTON_ORDER}
        self.lat_busy = False
        self.lat_started = 0.0
        self._warned = set()
        self._last_note = None
        # 중재기가 없으면 예전처럼 동작한다 (이 변경만으로 시스템이 멈추면 안 된다)
        self._has_control = True
        self._released = False      # 권한을 잃고 정지를 이미 보냈는가
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
        self.last_remote = now
        if not self._has_control:
            return               # 권한이 없으면 발행하지 않는다 (정지는 이미 보냈다)
        if self._last_rx_log and now - self._last_rx_log > self._gap_warn_sec:
            self.get_logger().warning(
                f"/remote_control 수신 공백 {(now - self._last_rx_log)*1000:.0f}ms")
        self._last_rx_log = now
        self.last_remote = now
        self.tick()          # 받는 즉시 반영 — 타이머를 기다리지 않는다

    def _on_mode(self, msg):
        try:
            mode = (json.loads(msg.data) or {}).get('mode', 'manual')
        except ValueError:
            return
        has = (mode == 'manual')
        if has == self._has_control:
            return
        self._has_control = has
        if not has:
            self._stop(f"제어 권한 넘김 (현재 모드 {mode})")
            self._released = True
        else:
            self._released = False
            self._note("제어 권한 회복 — 리모콘 조작 가능")

    def _on_lat_done(self, msg):
        self.lat_busy = False
        self.lat_started = 0.0
        self.get_logger().info(f"횡이동 완료: {msg.data}")

    def _on_safety(self, msg):
        self.blocked = set(msg.blocked_axes)

    def _limit_clamp(self, axis, dps):
        """리미트에 닿은 방향이면 0 으로 깎는다. 부호 환산은 모듈 문서 참고."""
        if not dps:
            return dps
        if self.blocked is None:
            if 'no_safety' not in self._limit_noted:
                self._limit_noted.add('no_safety')
                self.get_logger().warning(
                    "/safety/state 를 못 받고 있다 — X·Y 리미트 차단이 동작하지 않는다 "
                    "(safety_node 확인). 조작은 그대로 통과시킨다")
            return dps
        d = int((self.ax.get(axis) or {}).get('home_dir', 1))
        mm_dir = '+' if (-1 if dps > 0 else 1) * d > 0 else '-'
        key = f'{axis}{mm_dir}'
        if key in self.blocked:
            if key not in self._limit_noted:
                self._limit_noted.add(key)
                self.get_logger().warning(
                    f"{axis.upper()} {'원점' if mm_dir == '-' else '반대'} 쪽 리미트 — "
                    f"그 방향 조작을 막는다 ({key})")
            return 0.0
        self._limit_noted.discard(key)
        return dps

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
        if not self._has_control:
            return               # 권한이 없으면 워치독 정지도 내지 않는다
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
        xd = self._limit_clamp('x', float(an1 * self.xy_max))
        yd = self._limit_clamp('y', float(an2 * self.xy_max))
        self._pub_if_changed(self.x_pub, 'x', xd, 0.5)
        self._pub_if_changed(self.y_pub, 'y', yd, 0.5)

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
                   f"XY {xd:+.0f}/{yd:+.0f}  Z {z:+.0f}"
                   + (f"  차단 {','.join(sorted(self.blocked))}" if self.blocked else ''))

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
