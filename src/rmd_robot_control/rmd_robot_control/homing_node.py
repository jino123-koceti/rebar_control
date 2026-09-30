#!/usr/bin/env python3
"""L3 — 상부 스테이지 원점복귀.

리미트 센서를 보고 축을 원점까지 보낸 뒤 레퍼런스를 기록한다.
**CAN 을 모른다.** 리미트는 `/limit_sensors/*`(L1 ezi_io_node)로 받고,
축은 `/joint_N/speed` 로 명령한다 (YEAR3_ARCHITECTURE.md §2 계층 규칙).

──────────────────────────────────────────────────────────────────────────
2차년도(`rebar_base_control/homing_controller.py`, 1,106줄)와 다른 점

  · 그 노드는 CAN ID·엔코더 상수·Yaw 자세복귀(L2R)까지 한 파일에 있었다.
    3차년도는 축 정의를 `config/axes.yaml` 에 두고, 이 노드는 순서만 관리한다.
  · **축을 하나씩 따로 호밍할 수 있다.** 3차년도는 호밍을 한 번도 돌려본 적이
    없으므로, 전체 시퀀스를 처음부터 돌리는 것은 위험하다. Z 단독 → X → Y → Yaw
    순서로 검증하고 마지막에 전체를 돌린다.
  · Yaw 자세복귀(L2R)는 아직 옮기지 않았다. 3차년도 Yaw 엔코더 기준값이 미측정이다.

## 명령 (`/homing_cmd`, String)

    "z" / "x" / "y" / "yaw"   해당 축만 원점으로
    "all"                     Z → X → Y → Yaw 순서로 전체
    "stop"                    중단 (전 축 정지)

## 상태 (`/homing_status`, String JSON)

    {"state": "...", "axis": "...", "detail": "...", "refs": {...}}

## 안전

  · 리미트 토픽이 `limit_stale_sec` 이상 갱신되지 않으면 시작하지 않고, 진행 중이면
    중단한다. 센서를 못 보는 상태로 축을 미는 것이 가장 위험하다.
  · 목표 리미트가 이미 눌려 있으면 그 축은 곧바로 BACK_OFF 로 간다.
  · 반대쪽 리미트가 눌리면 즉시 중단한다 (방향이 반대라는 뜻이다).
  · 축별 타임아웃. 넘으면 정지하고 실패로 끝낸다.
  · 비상정지(`/safety/state` 또는 `/remote_control`)는 아직 연결하지 않았다 — S2 에서
    safety_node 가 생기면 그쪽을 구독한다. 그때까지는 `stop` 명령과 타임아웃만 있다.

⚠ 브레이크: 이 노드는 브레이크를 풀지 않는다. X/Y 는 `0x77` 로 미리 풀어야 하고,
  Z 는 리프팅축이라 자중 낙하 위험이 있어 기구 상태를 보고 사람이 판단한다
  (`axes.yaml` 의 `never_auto_release`).
"""

import json
import os
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64, String


class Phase(Enum):
    IDLE = 'idle'
    SEEK = 'seek'          # 원점 리미트를 향해 이동
    BACK_OFF = 'back_off'  # 리미트에서 살짝 빠짐 (센서 해제)
    FINE = 'fine'          # 느린 속도로 재접근 → 레퍼런스 기록
    DONE = 'done'
    FAILED = 'failed'


# 축 정의 — **모터 ID 는 config/axes.yaml 에서 읽는다.** 코드에 0x14X 를 쓰지 않는다
# (tools/check 의 R1). 2차년도는 ID 가 10개 파일에 흩어져 있어 3차년도로 넘어올 때
# 주석과 실제가 어긋났다.
#   joint: 명령 토픽 번호,  home_limit: 원점 리미트,  far_limit: 반대쪽(안전 확인용)
#   dir: 원점 방향 부호 (실측으로 확정해야 한다 — 기본값은 미검증)
AXES = {
    'x':   dict(joint=3, home_limit='x_min', far_limit='x_max', dir=-1),
    'y':   dict(joint=4, home_limit='y_min', far_limit='y_max', dir=-1),
    'z':   dict(joint=5, home_limit='z_min', far_limit='z_max', dir=+1),
    'yaw': dict(joint=6, home_limit='yaw_home', far_limit=None, dir=-1),
}


def load_axis_motor_ids():
    """axes.yaml 의 stage 항목에서 축별 CAN ID 를 읽는다.

    못 읽으면 위치 토픽만 못 구독한다(레퍼런스 기록이 비게 된다). 호밍 자체는
    리미트로 동작하므로 진행은 가능하다 — 그래서 실패해도 죽이지 않는다.
    """
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('rebar_base_control'),
                         'config', 'axes.yaml')
        stage = (yaml.safe_load(open(p, encoding='utf-8')) or {}).get('stage', {})
        out = {}
        for name in AXES:
            cid = (stage.get(name) or {}).get('can_id')
            if cid is not None:
                out[name] = f"0x{int(cid):03X}".lower().replace('0X', '0x')
        return out
    except Exception:
        return {}
LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')
ALL_ORDER = ('z', 'x', 'y', 'yaw')      # 2차년도와 같은 순서 (Z 를 먼저 안전 위치로)


class HomingNode(Node):
    def __init__(self):
        super().__init__('homing_node')

        self.declare_parameter('seek_speed_dps', 30.0)
        self.declare_parameter('fine_speed_dps', 8.0)
        self.declare_parameter('back_off_sec', 0.7)
        self.declare_parameter('axis_timeout_sec', 40.0)
        self.declare_parameter('limit_stale_sec', 1.0)
        # 축별 원점 방향 부호. **실측 전이므로 기본값을 믿지 말 것.**
        for name, cfg in AXES.items():
            self.declare_parameter(f'{name}_dir', cfg['dir'])

        g = self.get_parameter
        self.seek_speed = float(g('seek_speed_dps').value)
        self.fine_speed = float(g('fine_speed_dps').value)
        self.back_off_sec = float(g('back_off_sec').value)
        self.axis_timeout = float(g('axis_timeout_sec').value)
        self.limit_stale = float(g('limit_stale_sec').value)
        for name in AXES:
            AXES[name]['dir'] = int(g(f'{name}_dir').value)

        self.speed_pubs = {n: self.create_publisher(Float32, f"/joint_{c['joint']}/speed", 10)
                           for n, c in AXES.items()}
        self.status_pub = self.create_publisher(String, '/homing_status', 10)

        self.limit = {n: None for n in LIMITS}
        self.limit_time = {n: 0.0 for n in LIMITS}
        self.pos = {}
        for n in LIMITS:
            self.create_subscription(Bool, f'/limit_sensors/{n}',
                                     lambda m, k=n: self._on_limit(k, m), 10)
        motor_ids = load_axis_motor_ids()
        if not motor_ids:
            self.get_logger().warning(
                "axes.yaml 에서 축 CAN ID 를 읽지 못했습니다 — 위치 레퍼런스는 기록되지 않습니다")
        for name, mid in motor_ids.items():
            self.create_subscription(Float64, f"/motor_{mid}_position",
                                     lambda m, k=name: self.pos.__setitem__(k, m.data), 10)
        self.create_subscription(String, '/homing_cmd', self._on_cmd, 10)

        self.phase = Phase.IDLE
        self.axis = None
        self.queue = []
        self.t_phase = 0.0
        self.refs = {}
        self.detail = ''

        self.timer = self.create_timer(0.05, self.tick)      # 20Hz
        # 상태를 주기적으로도 알린다 (전이 때만 내면 관측자가 현재 상태를 못 본다)
        self.status_timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            "호밍 노드 시작 — /homing_cmd 로 'z'|'x'|'y'|'yaw'|'all'|'stop'")
        self.get_logger().warning(
            "⚠ 브레이크는 이 노드가 풀지 않습니다. X/Y 는 미리 0x77 로 해제하세요. "
            "Z 는 낙하 위험이 있어 기구 상태 확인 후 판단하세요.")

    # ---- 입력 --------------------------------------------------------------
    def _on_limit(self, name, msg):
        self.limit[name] = msg.data
        self.limit_time[name] = time.time()

    def _on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'stop':
            self._stop_all('정지 명령')
            return
        if self.phase not in (Phase.IDLE, Phase.DONE, Phase.FAILED):
            self.get_logger().warning(f"호밍 진행 중({self.axis}) — '{cmd}' 무시")
            return
        if cmd == 'all':
            self.queue = list(ALL_ORDER)
        elif cmd in AXES:
            self.queue = [cmd]
        else:
            self.get_logger().error(f"알 수 없는 명령: '{cmd}'")
            return
        if not self._limits_fresh():
            self._fail("리미트 토픽이 갱신되지 않습니다 — ezi_io_node 확인")
            return
        self._next_axis()

    # ---- 진행 --------------------------------------------------------------
    def _limits_fresh(self):
        now = time.time()
        stale = [n for n in LIMITS
                 if self.limit[n] is None or now - self.limit_time[n] > self.limit_stale]
        if stale:
            self.detail = f"갱신 안 된 리미트: {', '.join(stale)}"
            return False
        return True

    def _next_axis(self):
        if not self.queue:
            self._enter(Phase.DONE, '완료')
            self.get_logger().info(f"호밍 완료 — 레퍼런스 {self.refs}")
            return
        self.axis = self.queue.pop(0)
        cfg = AXES[self.axis]
        if self.limit[cfg['home_limit']]:
            self.get_logger().info(
                f"{self.axis}: 이미 {cfg['home_limit']} 에 있음 → 후퇴부터")
            self._enter(Phase.BACK_OFF, '이미 원점')
        else:
            self._enter(Phase.SEEK, f"{cfg['home_limit']} 탐색")

    def _enter(self, phase, detail=''):
        self.phase = phase
        self.detail = detail
        self.t_phase = time.time()
        self._publish_status()
        if phase in (Phase.DONE, Phase.FAILED, Phase.IDLE):
            self.axis = None

    def _cmd_speed(self, axis, dps):
        self.speed_pubs[axis].publish(Float32(data=float(dps)))

    def _stop_axis(self, axis):
        self._cmd_speed(axis, 0.0)

    def _stop_all(self, reason):
        for a in AXES:
            self._cmd_speed(a, 0.0)
        self.queue = []
        self.get_logger().warning(f"호밍 중단 — {reason}")
        self._enter(Phase.IDLE, reason)

    def _fail(self, reason):
        for a in AXES:
            self._cmd_speed(a, 0.0)
        self.queue = []
        self.get_logger().error(f"호밍 실패 — {reason}")
        self._enter(Phase.FAILED, reason)

    def tick(self):
        if self.phase in (Phase.IDLE, Phase.DONE, Phase.FAILED):
            return
        if not self._limits_fresh():
            self._fail(self.detail)
            return
        cfg = AXES[self.axis]
        elapsed = time.time() - self.t_phase
        if elapsed > self.axis_timeout:
            self._fail(f"{self.axis} {self.phase.value} 타임아웃 {self.axis_timeout:.0f}s")
            return
        # 반대쪽 리미트가 눌리면 방향이 반대다 — 즉시 멈춘다
        if cfg['far_limit'] and self.limit[cfg['far_limit']]:
            self._fail(f"{self.axis}: 반대쪽 리미트({cfg['far_limit']})에 닿음 — 방향 부호 확인")
            return

        at_home = bool(self.limit[cfg['home_limit']])

        if self.phase is Phase.SEEK:
            if at_home:
                self._stop_axis(self.axis)
                self.get_logger().info(f"{self.axis}: {cfg['home_limit']} 도달 → 후퇴")
                self._enter(Phase.BACK_OFF, '리미트 도달')
            else:
                self._cmd_speed(self.axis, cfg['dir'] * self.seek_speed)

        elif self.phase is Phase.BACK_OFF:
            if elapsed < self.back_off_sec:
                self._cmd_speed(self.axis, -cfg['dir'] * self.fine_speed)
            else:
                self._stop_axis(self.axis)
                if at_home:
                    self._fail(f"{self.axis}: 후퇴했는데 {cfg['home_limit']} 가 안 풀림 "
                               "— 센서 위치·감도 확인")
                else:
                    self.get_logger().info(f"{self.axis}: 센서 해제 → 정밀 재접근")
                    self._enter(Phase.FINE, '정밀 재접근')

        elif self.phase is Phase.FINE:
            if at_home:
                self._stop_axis(self.axis)
                p = self.pos.get(self.axis)
                self.refs[self.axis] = p
                self.get_logger().info(
                    f"{self.axis}: 원점 확정" + (f" (위치 {p:.2f}°)" if p is not None else
                                              " (위치 토픽 없음)"))
                self._next_axis()
            else:
                self._cmd_speed(self.axis, cfg['dir'] * self.fine_speed)

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.phase.value,
            'axis': self.axis,
            'detail': self.detail,
            'queue': self.queue,
            'refs': {k: v for k, v in self.refs.items()},
        }, ensure_ascii=False)))

    def destroy_node(self):
        try:
            for a in AXES:
                self._cmd_speed(a, 0.0)
            time.sleep(0.1)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HomingNode()
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
