#!/usr/bin/env python3
"""L5 — 자율 주행 고리. 격자 플래너(`coverage_map`)를 실장비에 붙인다.

    보고(detect) → 계획(plan) → 미션으로 넘기고 → 다시 본다

`navigator`(L4)는 **미리 정한 순서**를 실행한다 (`['tie', 'drive 350', …]`).
없던 것은 "단계를 실행하는 층" 이 아니라 **"단계를 정하는 층"** 이다. 이
노드가 그 자리다 — 지도를 보고 다음 한 묶음을 만들어 `navigator` 에게 준다.

## 왜 직접 `/drive/step` 을 쏘지 않는가

`navigator` 가 이미 타임아웃·중단 전파·**안전상태(`/safety/state`) 연동**을
한다. 여기서 다시 구현하면 두 경로가 생기고, 안전 판단이 한쪽에만 있게 된다.
그래서 이 노드는 미션을 **만들기만** 하고 실행은 넘긴다.

정차지 **안**의 결속은 `tying_orchestrator_node`/`tying_planner` 가 한다.
이 증분에서는 주행 고리만 돈다 (`tie_at_stop` 을 켜면 미션에 `tie` 가 붙는다).

## 기본값이 드라이런이다

`dry_run: true` — 계획만 세우고 미션을 보내지 않는다. 기본값을 반대로 두면
"그냥 띄워봤다" 가 곧 주행이 된다.

## 토픽·서비스

    호출  /navigator/set_parameters (SetParameters) 미션 주입
    발행  /rebar/detect    (Empty)  검출 트리거 — `tying_planner` 와 같은 경로
          /mission/start   (Empty)  미션 실행
          /mission/abort   (Empty)  중단
          /autonomy/status (String) JSON 상태
    구독  /rebar/crossings (RebarGrid) 검출 결과
          /mission/status  (String) phase 로 완료 판정
          /autonomy/cmd    (String) "start" / "stop"

경로를 **그대로 따라간다.** 플래너가 A* 로 찾은 `path` 를 무시하고 ㄱ자로
움직이면 기둥을 관통한다 — 시뮬에서 실제로 그랬다.
"""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Empty, String

from rebar_base_interfaces.msg import RebarGrid

from rmd_robot_control.coverage_map import CoverageMap
from rmd_robot_control.coverage_reach import Reach

# 상태. 문자열로 두는 이유는 로그와 `/autonomy/status` 에 그대로 실리기 때문이다.
IDLE, OBSERVE, PLAN, SEND, RUN, DONE, FAILED = (
    'idle', 'observe', 'plan', 'send', 'run', 'done', 'failed')


class DriveAutonomyNode(Node):

    def __init__(self):
        super().__init__('drive_autonomy_node')
        cb = ReentrantCallbackGroup()

        self.declare_parameter('dry_run', True)
        self.declare_parameter('detect_timeout_s', 20.0)
        self.declare_parameter('mission_timeout_s', 300.0)
        self.declare_parameter('max_stops', 30)
        # 한 구간에 허용할 최대 전진. 플래너가 이상한 값을 내거나 부호가
        # 뒤집혀도 장비가 멀리 가버리지 못하게 막는 마지막 울타리다.
        self.declare_parameter('max_leg_mm', 600.0)
        # 정차지마다 결속까지 할지. 이 증분의 기본은 주행 고리만이다.
        self.declare_parameter('tie_at_stop', False)
        # 검출 필터는 `tying_planner` 와 **같은 값**을 쓴다. 다르면 두 층이
        # 서로 다른 점 집합을 보고, 지도에 있는 셀을 결속부가 못 가는 일이 난다.
        self.declare_parameter('min_confidence', 0.5)
        # ⚠ 깊이 필터는 선택이 아니다. 캘리 모델은 깊이를 쓰지 않으므로
        #   평면 밖 검출이 **엉뚱한 좌표로 환산된다** — 2026-10-15 실측에서
        #   깊이 744mm 검출이 (0.5, 76.9)mm 로 환산돼 교차점이 하나도 없는
        #   자리로 장비가 갔다. 신뢰도 임계가 우연히 걸러 주던 것이다.
        self.declare_parameter('depth_min_mm', 330.0)
        self.declare_parameter('depth_max_mm', 660.0)

        g = self.get_parameter
        self.dry_run = bool(g('dry_run').value)
        self.detect_timeout = float(g('detect_timeout_s').value)
        self.mission_timeout = float(g('mission_timeout_s').value)
        self.max_stops = int(g('max_stops').value)
        self.max_leg_mm = float(g('max_leg_mm').value)
        self.tie_at_stop = bool(g('tie_at_stop').value)
        self.min_conf = float(g('min_confidence').value)
        self.depth_lo = float(g('depth_min_mm').value)
        self.depth_hi = float(g('depth_max_mm').value)

        self.reach = Reach.from_files()
        self.map = CoverageMap(self.reach)

        self.state = IDLE
        self.plan = None
        self.mission = []
        self.deadline = 0.0
        self.stops = 0
        self.phase = 'idle'
        self.detail = ''

        self.grid = None            # 새 검출이 오면 채워진다
        self.detect_sent = False
        self.params = self.create_client(
            SetParameters, '/navigator/set_parameters', callback_group=cb)
        self.param_future = None

        self.pub_detect = self.create_publisher(Empty, '/rebar/detect', 10)
        self.pub_start = self.create_publisher(Empty, '/mission/start', 10)
        self.pub_abort = self.create_publisher(Empty, '/mission/abort', 10)
        self.pub_status = self.create_publisher(String, '/autonomy/status', 10)

        self.create_subscription(RebarGrid, '/rebar/crossings', self.on_grid, 10)
        self.create_subscription(String, '/mission/status', self.on_mission, 10)
        self.create_subscription(String, '/autonomy/cmd', self.on_cmd, 10)

        self.create_timer(0.2, self.tick, callback_group=cb)
        self.create_timer(1.0, self.publish_status)

        mode = '드라이런 (미션을 보내지 않는다)' if self.dry_run else '★ 실제 주행'
        self.get_logger().info(f'자율 주행 고리 — {mode}')
        self.get_logger().info(
            f'   한 구간 상한 {self.max_leg_mm:.0f}mm · 정차 상한 {self.max_stops}회'
            f" · 정차 결속 {'켬' if self.tie_at_stop else '끔'}")

    # ── 들어오는 것 ─────────────────────────────────────────────────────

    def on_grid(self, msg):
        # 트리거를 보낸 뒤에 온 것만 받는다. 켜 두면 **예전 검출**을 새 것으로
        # 착각해 움직이지도 않고 다음 정차지를 계획한다.
        if self.detect_sent:
            self.grid = msg

    def on_mission(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.phase = d.get('phase', 'idle')
        self.detail = d.get('detail', '')

    def on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            if self.state in (IDLE, DONE, FAILED):
                self.stops = 0
                self.go(OBSERVE, '시작')
            else:
                self.get_logger().warning(f'이미 도는 중이다 ({self.state})')
        elif cmd == 'stop':
            self.get_logger().warning('정지 요청')
            if not self.dry_run:
                self.pub_abort.publish(Empty())
            self.go(IDLE)

    def go(self, state, why=''):
        self.state = state
        if why:
            self.get_logger().info(f'[{state}] {why}')

    # ── 상태 기계 ───────────────────────────────────────────────────────

    def tick(self):
        try:
            {OBSERVE: self.do_observe, PLAN: self.do_plan,
             SEND: self.do_send, RUN: self.do_run}.get(
                 self.state, lambda: None)()
        except Exception as e:                      # 고리가 통째로 멈추지 않게
            self.get_logger().error(f'고리 오류: {e}')
            self.go(FAILED)

    def do_observe(self):
        """교차점을 받아 지도에 넣는다.

        검출은 서비스가 아니라 **토픽 쌍**이다 (`tying_planner` 와 같은 경로):
        `/rebar/detect` 로 트리거하고 `/rebar/crossings` 로 받는다.
        """
        if not self.detect_sent:
            self.grid = None
            self.detect_sent = True
            self.pub_detect.publish(Empty())
            self.deadline = time.time() + self.detect_timeout
            return

        if self.grid is None:
            if time.time() > self.deadline:
                self.get_logger().error('검출 결과가 오지 않았다 (시간 초과)')
                self.detect_sent = False
                self.go(FAILED)
            return

        grid, self.grid = self.grid, None
        self.detect_sent = False
        if not grid.valid:
            self.get_logger().error(f'검출 실패: {grid.error_message}')
            self.go(FAILED)
            return

        dets, drop_conf, drop_depth = self.to_canon(grid.detections)
        # ⚠ **조용히 버리지 않는다.** 왜 점이 줄었는지 로그에 남아야 사람이
        #   안다 (이 저장소는 조용한 필터로 여러 번 시간을 잃었다).
        if drop_conf or drop_depth:
            self.get_logger().warning(
                f'  제외 — 신뢰도 {drop_conf}개 · 깊이범위 밖 {drop_depth}개'
                f' (허용 {self.depth_lo:.0f}~{self.depth_hi:.0f}mm)')
        self.map.observe(dets)
        self.get_logger().info(
            f'검출 {len(grid.detections)}점 중 {len(dets)}점 사용 '
            f'→ 지도 {self.map.counts()} (정차 {self.stops}회째)')
        self.go(PLAN)

    def to_canon(self, detections):
        """검출을 **정준 좌표**로 바꾼다. 지도는 카메라 mm 를 모른다.

        카메라 좌표와 스테이지 좌표는 29° 기울기 때문에 다르다 — 그대로
        넣으면 격자가 통째로 어긋난다. `Reach` 가 들고 있는 자세별 A·b 가
        `tying_planner._to_stage` 와 **같은 모델**이라 두 층이 일치한다.
        """
        out, bad_conf, bad_depth = [], 0, 0
        for d in detections:
            if d.confidence < self.min_conf:
                bad_conf += 1
                continue
            if not (self.depth_lo <= d.depth_mm <= self.depth_hi):
                bad_depth += 1
                continue
            out.append(self.reach.canon_of_cam(d.x, d.y))
        return out, bad_conf, bad_depth

    def do_plan(self):
        """다음 정차지를 고르고 미션 단계로 바꾼다."""
        if self.stops >= self.max_stops:
            self.get_logger().warning(f'정차 상한 {self.max_stops}회 — 멈춘다')
            self.go(DONE)
            return

        plan = self.map.plan_next()
        if plan is None:
            self.get_logger().info(f'더 갈 곳이 없다 — 완료 ({self.stops}회 정차)')
            self.go(DONE)
            return

        self.plan = plan
        self.mission = self.mission_of(plan)
        kind = '결속' if plan['kind'] == 'tie' else '탐색'
        self.get_logger().info(
            f"계획[{kind}] 전진 {plan['drive_mm']:+.0f}mm · 횡 {plan['turns']:+d}회전 "
            f"· 셀 {len(plan['cells'])}개 · 비용 {plan['cost_s']:.1f}s")
        self.get_logger().info(f'  미션 {self.mission}')
        self.go(SEND)

    def mission_of(self, plan):
        """경로(포즈 목록)를 `navigator` 미션 단계로 바꾼다.

        포즈 사이에서 바뀐 축이 그 단계의 축이다. 경로를 안 쓰고 목적지로
        바로 가면 기둥을 관통한다 — 그래서 포즈를 하나씩 따라간다.
        `lateral` 단계는 **회전수**를 받는다 (mm 가 아니다).
        """
        path = plan.get('path') or []
        legs = []
        if len(path) < 2:
            # 경로가 없으면(장애물 없는 판) 계획의 총량을 그대로 쓴다
            if plan['turns']:
                legs.append(('v', plan['turns']))
            if abs(plan['drive_mm']) > 1.0:
                legs.append(('u', plan['drive_mm']))
        else:
            for (u0, v0), (u1, v1) in zip(path, path[1:]):
                du, dv = u1 - u0, v1 - v0
                if abs(dv) > 1e-6:
                    legs.append(('v', int(round(dv / self.map.step_v()))))
                if abs(du) > 1e-6:
                    legs.append(('u', du))

        out = []
        for axis, qty in legs:
            if not qty:
                continue
            if axis == 'v':
                out.append(f'lateral {int(qty):+d}')
            else:
                if abs(qty) > self.max_leg_mm:
                    raise RuntimeError(
                        f'한 구간 {qty:+.0f}mm 는 상한 '
                        f'{self.max_leg_mm:.0f}mm 를 넘는다')
                out.append(f'drive {qty:.0f}')
        if self.tie_at_stop and plan['kind'] == 'tie':
            out.append('tie')
        return out

    def do_send(self):
        """미션을 `navigator` 파라미터에 넣고 실행을 건다."""
        if not self.mission:
            self.arrive()
            return

        if self.dry_run:
            self.get_logger().info('  (드라이런 — 미션을 보내지 않는다)')
            self.arrive()
            return

        if self.param_future is None:
            if not self.params.service_is_ready():
                self.get_logger().warning(
                    '/navigator/set_parameters 가 아직 없다 — 기다린다',
                    throttle_duration_sec=5.0)
                return
            req = SetParameters.Request()
            req.parameters = [Parameter(
                name='mission',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING_ARRAY,
                    string_array_value=self.mission))]
            self.param_future = self.params.call_async(req)
            self.deadline = time.time() + 10.0
            return

        if not self.param_future.done():
            if time.time() > self.deadline:
                self.get_logger().error('미션 주입 응답이 없다')
                self.param_future = None
                self.go(FAILED)
            return

        res = self.param_future.result()
        self.param_future = None
        if res is None or not all(r.successful for r in res.results):
            self.get_logger().error('미션 주입 거부됨')
            self.go(FAILED)
            return

        self.phase = 'idle'
        self.pub_start.publish(Empty())
        self.deadline = time.time() + self.mission_timeout
        self.go(RUN, '미션 실행 요청')

    def do_run(self):
        """`navigator` 가 끝낼 때까지 기다린다."""
        if self.phase == 'done':
            self.arrive()
        elif self.phase == 'failed':
            self.get_logger().error(f'미션 실패: {self.detail}')
            self.go(FAILED)
        elif time.time() > self.deadline:
            self.get_logger().error('미션이 시간 안에 안 끝났다')
            self.pub_abort.publish(Empty())
            self.go(FAILED)

    def arrive(self):
        """도착 — 지도에 반영하고 다시 본다."""
        p = self.plan
        self.map.advance(drive_mm=p['drive_mm'], lateral_turns=p['turns'])
        self.stops += 1
        self.get_logger().info(
            f'도착 (u={self.map.ru:.0f} v={self.map.rv:.0f}) — 다시 검출한다')
        self.go(OBSERVE)

    # ── 상태 발행 ───────────────────────────────────────────────────────

    def publish_status(self):
        self.pub_status.publish(String(data=json.dumps({
            'state': self.state,
            'dry_run': self.dry_run,
            'stops': self.stops,
            'pose': {'u': round(self.map.ru, 1), 'v': round(self.map.rv, 1)},
            'cells': self.map.counts(),
            'mission': self.mission,
            'mission_phase': self.phase,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = DriveAutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
