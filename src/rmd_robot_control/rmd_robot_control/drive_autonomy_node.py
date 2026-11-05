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
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Empty, String

from rebar_base_interfaces.msg import RebarGrid

from rmd_robot_control.coverage_map import CoverageMap, FAILED as CELL_FAILED, TIED
from rmd_robot_control.coverage_reach import Reach
from rmd_robot_control.run_log import RunLog

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
        # ── 폭주 울타리 ──────────────────────────────────────────────
        # ⚠ 이것은 **안전층이 아니다.** 진짜 안전(데크 끝·장애물 정지)이 아직
        #   자율 고리에 안 붙어 있어서 그 동안 세워 둔 임시 방벽이다.
        #   `/deck_edge_block` 은 아직 발행자가 없다 (safety_node 는 구독만).
        #   그것들이 붙으면 이 값은 거의 의미가 없어진다 — "데크 끝이 보이면
        #   멈춘다" 가 거리 상한보다 훨씬 옳은 기준이다.
        #
        # 값의 근거: 플래너가 한 번에 움직이는 거리는 **결속영역 길이
        #   (U 방향 520mm)** 언저리가 자연 상한이다. 그보다 훨씬 먼 계획은
        #   진짜 이상한 것이다. 600 이면 **정상 계획은 안 걸리고 폭주는 잡는다.**
        #   ⚠ 250 으로 뒀다가 259mm 짜리 정상 계획을 9mm 차이로 거부했다
        #     (2026-11-05). 근거 없는 숫자는 오탐으로 돌아온다.
        self.declare_parameter('max_leg_mm', 600.0)
        # 횡이동에도 같은 울타리. 없으면 전진만 막고 횡은 무제한이라
        # 일관성이 없다 — 부호가 뒤집히면 옆으로 멀리 간다.
        self.declare_parameter('max_leg_turns', 6)      # 6회전 = 300mm
        # 정차지마다 결속까지 할지. 이 증분의 기본은 주행 고리만이다.
        self.declare_parameter('tie_at_stop', True)
        # 결속 결과를 셀에 되돌릴 때의 짝짓기 허용 오차. 철근 피치가 200mm
        # 안팎이므로 반 피치보다 한참 작아야 옆 셀과 헷갈리지 않는다.
        self.declare_parameter('match_tol_mm', 60.0)
        # 돌릴 때마다 CSV 로 남긴다 — 빈 문자열이면 끈다.
        # 왜 기본으로 켜 두나: 2026-10-15~16 야외 시도가 계측을 안 해
        # **쓸 수 없는 데이터**가 됐다. 기록은 켜 두는 쪽이 기본이어야 한다.
        # ⚠⚠ 끝나면 **노드를 내린다.** 떠 있으면 `/autonomy/cmd` 가
        #   브로드캐스트라 다음에 누가 start 를 보낼 때 **이 노드도 받는다** —
        #   그리고 DONE 상태의 노드는 정차 수를 0 으로 되돌리고 다시 돈다.
        #   2026-11-05: 끝난 실주행 노드를 안 치운 채 드라이런을 시험하려고
        #   start 를 쐈더니 **살아 있던 실주행 노드가 장비를 움직였다.**
        #   끝난 노드가 남아 있으면 장전된 총이다. 상주가 필요하면 끄면 된다.
        self.declare_parameter('exit_when_done', True)
        self.declare_parameter(
            'run_log_dir',
            os.path.join(os.path.expanduser('~'), 'rebar_runs'))
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
        self.max_leg_turns = int(g('max_leg_turns').value)
        self.tie_at_stop = bool(g('tie_at_stop').value)
        self.match_tol = float(g('match_tol_mm').value)
        self.exit_done = bool(g('exit_when_done').value)
        self.log_dir = str(g('run_log_dir').value or '')
        self.log = RunLog(self.log_dir, tag='dry' if self.dry_run else 'live')
        self.t_mark = {}
        self.min_conf = float(g('min_confidence').value)
        self.depth_lo = float(g('depth_min_mm').value)
        self.depth_hi = float(g('depth_max_mm').value)

        self.reach = Reach.from_files()
        self.map = CoverageMap(self.reach)

        self.state = IDLE
        self.plan = None
        self.mission = []
        self.moved = (0.0, 0)
        self.det_total = self.det_used = 0
        self.tie_matched = self.tie_ok = 0
        self.deadline = 0.0
        self.stops = 0
        self.phase = 'idle'
        self.detail = ''
        self.points = []     # /plan/status 의 점별 결속 결과
        self.path_cleared = False

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
        self.create_subscription(String, '/plan/status', self.on_plan, 10)
        self.create_subscription(String, '/mission/status', self.on_mission, 10)
        self.create_subscription(String, '/autonomy/cmd', self.on_cmd, 10)

        self.create_timer(0.2, self.tick, callback_group=cb)
        self.create_timer(1.0, self.publish_status)

        mode = '드라이런 (미션을 보내지 않는다)' if self.dry_run else '★ 실제 주행'
        self.get_logger().info(f'자율 주행 고리 — {mode}')
        self.get_logger().info(
            f'   구간 상한 전진 {self.max_leg_mm:.0f}mm / 횡 {self.max_leg_turns}회전'
            f' · 정차 상한 {self.max_stops}회'
            f" · 정차 결속 {'켬' if self.tie_at_stop else '끔'}")

    # ── 들어오는 것 ─────────────────────────────────────────────────────

    def on_grid(self, msg):
        # 트리거를 보낸 뒤에 온 것만 받는다. 켜 두면 **예전 검출**을 새 것으로
        # 착각해 움직이지도 않고 다음 정차지를 계획한다.
        if self.detect_sent:
            self.grid = msg

    def on_plan(self, msg):
        try:
            self.points = json.loads(msg.data).get('points') or []
        except ValueError:
            pass

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
        self.t_mark[state] = time.time()
        self.state = state
        if why:
            self.get_logger().info(f'[{state}] {why}')
        if state in (DONE, FAILED) and self.exit_done:
            self.get_logger().info('노드를 내린다 (exit_when_done)')
            self.log.close()
            raise SystemExit(0 if state == DONE else 1)

    # ── 상태 기계 ───────────────────────────────────────────────────────

    def tick(self):
        try:
            {OBSERVE: self.do_observe, PLAN: self.do_plan,
             SEND: self.do_send, RUN: self.do_run}.get(
                 self.state, lambda: None)()
        except SystemExit:
            raise                                   # 종료는 삼키지 않는다
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
        self.det_total, self.det_used = len(grid.detections), len(dets)
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
            self._log_done()
            self.go(DONE)
            return

        plan = self.map.plan_next()
        if plan is None:
            self.get_logger().info(f'더 갈 곳이 없다 — 완료 ({self.stops}회 정차)')
            self._log_done()
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

    def _log_done(self):
        if self.log.ok:
            self.get_logger().info('  기록: ' + ' , '.join(self.log.paths(self.log_dir)))
        elif self.log_dir:
            self.get_logger().warning(f'  기록 실패 — {self.log_dir} 에 쓸 수 없다')

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
                    # ⚠ `step_v()` 가 아니라 `lat_mm` 이다. 전자는 **배근 피치**
                    #   (≈200mm), 후자는 **횡이동 1회전**(50mm)이다. 피치로
                    #   나누면 50mm 구간이 round(0.25)=0 이 되어 **횡이동이
                    #   통째로 사라진다** — 2026-11-05 기록에서 plan_turns 1 이
                    #   매번 moved_turns 0 으로 나와 드러났다.
                    legs.append(('v', int(round(dv / self.map.lat_mm))))
                if abs(du) > 1e-6:
                    legs.append(('u', du))

        out, done_u, done_v = [], 0.0, 0
        for axis, qty in legs:
            if not qty:
                continue
            if axis == 'v':
                if abs(qty) > self.max_leg_turns:
                    raise RuntimeError(
                        f'한 구간 횡이동 {qty:+d}회전은 상한 '
                        f'{self.max_leg_turns}회전을 넘는다')
                out.append(f'lateral {int(qty):+d}')
                done_v += int(qty)
            else:
                if abs(qty) > self.max_leg_mm:
                    raise RuntimeError(
                        f'한 구간 {qty:+.0f}mm 는 상한 '
                        f'{self.max_leg_mm:.0f}mm 를 넘는다')
                out.append(f'drive {qty:.0f}')
                done_u += float(f'{qty:.0f}')      # 미션에 적힌 값 그대로
        if self.tie_at_stop and plan['kind'] == 'tie':
            out.append('tie')
        # ⚠ 지도는 **계획이 아니라 실제로 미션에 넣은 양**으로 갱신해야 한다.
        #   `plan['turns']`/`plan['drive_mm']` 는 후보를 고를 때의 총량이고,
        #   미션은 A* 경로 구간에서 만들어져 **서로 다를 수 있다.**
        #   2026-11-05 실측: 계획 '횡 +1회전' 인데 미션엔 lateral 이 없었다.
        #   그대로 두면 지도가 로봇 위치를 50mm 틀리게 알고, 그 어긋남이
        #   결속 결과 짝짓기(허용 60mm)를 반 토막 냈다 (2개 중 1개).
        self.moved = (done_u, done_v)
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
            # ⚠ `path_file` 도 **같이 비워야 한다.** navigator 는 그 값이
            #   있으면 경로 파일에서 단계를 만들고 `mission` 을 무시한다
            #   (navigator.py: `texts = _from_path(path) if path else mission`).
            #   2026-10-20 실측: 미션은 `['drive 141']` 로 들어갔는데 실행된
            #   것은 데모 경로의 5단계였고 `tie` 부터 시작해 실패했다.
            req.parameters = [
                Parameter(name='mission',
                          value=ParameterValue(
                              type=ParameterType.PARAMETER_STRING_ARRAY,
                              string_array_value=self.mission)),
                Parameter(name='path_file',
                          value=ParameterValue(
                              type=ParameterType.PARAMETER_STRING,
                              string_value=''))]
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

        if not self.path_cleared:
            self.path_cleared = True
            self.get_logger().warning(
                'navigator 의 path_file 을 비웠다 — 미션 파라미터가 우선하도록.'
                ' 데모 경로를 다시 쓰려면 그 값을 되돌려야 한다')
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
        """도착 — 결속 결과를 셀에 되돌리고 다시 본다."""
        p = self.plan
        du, dv = self.moved
        if abs(du - p['drive_mm']) > 1.0 or dv != p['turns']:
            self.get_logger().warning(
                f"  계획과 미션이 다르다 — 계획 {p['drive_mm']:+.0f}mm/{p['turns']:+d}회전"
                f" vs 미션 {du:+.0f}mm/{dv:+d}회전. **미션 쪽으로** 지도를 갱신한다")
        self.map.advance(drive_mm=du, lateral_turns=dv)
        self.stops += 1
        if self.tie_at_stop and p['kind'] == 'tie':
            self.apply_tie(p)
        self.get_logger().info(
            f'도착 (u={self.map.ru:.0f} v={self.map.rv:.0f}) — 다시 검출한다')
        c = self.map.counts()
        now = time.time()
        self.log.stop(
            stop=self.stops, pose_u=round(self.map.ru, 1),
            pose_v=round(self.map.rv, 1),
            plan_kind=p['kind'], plan_drive_mm=round(p['drive_mm'], 1),
            plan_turns=p['turns'], plan_cells=len(p['cells']),
            plan_cost_s=round(p['cost_s'], 1),
            mission=' | '.join(self.mission), moved_mm=round(du, 1),
            moved_turns=dv,
            det_total=self.det_total, det_used=self.det_used,
            tie_reported=len(self.points), tie_matched=self.tie_matched,
            tie_ok=self.tie_ok,
            cells_pending=c.get('pending', 0), cells_tied=c.get('tied', 0),
            cells_empty=c.get('empty', 0), cells_blocked=c.get('blocked', 0),
            t_observe_s=round(self.t_mark.get(PLAN, now)
                              - self.t_mark.get(OBSERVE, now), 2),
            t_plan_s=round(self.t_mark.get(SEND, now)
                           - self.t_mark.get(PLAN, now), 2),
            t_mission_s=round(now - self.t_mark.get(RUN, now), 2),
            note='dry' if self.dry_run else '')
        self.go(OBSERVE)

    def apply_tie(self, plan):
        """결속 결과를 **셀별로** 되돌린다.

        이게 없으면 셀이 pending 에서 빠지지 않아 고리가 끝나지 않는다.

        점은 `stage_mm` 과 `pose` 로 온다. 계획했던 셀을 **그 점이 쓴 자세로**
        스테이지 좌표에 올려 가장 가까운 것과 짝짓는다. 자세를 계획 쪽 값으로
        쓰면 안 된다 — `tying_planner` 는 자세 전환을 줄이려 다른 자세를 고른다.

        드라이런은 결속이 성공한 것으로 **가정한다.** 그래야 고리가 끝까지
        돌아 전체 정차 순서를 볼 수 있다 — 가정했다는 것을 로그에 남긴다.
        """
        keys = [tuple(k) for k in plan['cells']]
        if not keys:
            return
        if self.dry_run:
            for k in keys:
                self.map.mark(k, TIED, '드라이런 — 결속 성공 가정')
            self.tie_matched, self.tie_ok = len(keys), len(keys)
            self.get_logger().info(f'  (드라이런) {len(keys)}개 셀을 결속으로 가정')
            return

        hit = n_ok = 0
        if not self.points:
            self.get_logger().warning(
                '  /plan/status 에 점별 결과가 없다 — 결속 결과를 반영 못 한다')
        for pt in self.points:
            pose, sm = pt.get('pose'), pt.get('stage_mm')
            if pose is None or not sm:
                continue
            best, bd = None, 1e9
            for k in keys:
                # ⚠ `uv_of` 는 **지도 전역 좌표**인데 `stage_of_canon` 은
                #   **로봇 기준** 정준 좌표를 받는다 (`observe` 와 같은 규약).
                #   그대로 넣으면 현재 포즈만큼 통째로 어긋나 하나도 안 맞는다
                #   — 2026-11-05 첫 실주행에서 "계획 4개 중 0개 짝지음" 이 났다.
                #   결속은 `advance()` 뒤 자리에서 했으므로 지금 포즈를 뺀다.
                u, v = self.map.uv_of(*k)
                sx, sy = self.reach.stage_of_canon(
                    pose, u - self.map.ru, v - self.map.rv)
                d = math.hypot(sx - sm[0], sy - sm[1])
                if d < bd:
                    best, bd = k, d
            if best is None or bd > self.match_tol:
                continue
            ok = pt.get('state') == '완료'
            self.map.mark(best, TIED if ok else CELL_FAILED,
                          pt.get('state', ''))
            hit += 1
            n_ok += 1 if ok else 0
            # ⚠ `resid_mm` 가 이 기록의 이유다 — 플래너가 "여기 있을 것" 이라
            #   계산한 자리와 결속부가 "여기 있었다" 고 보고한 자리의 거리.
            #   쌓이면 **정차마다 재검출하는 구조가 드리프트를 흡수하는가**에
            #   답할 수 있다 (지금은 가정이다).
            pu, pv = self.map.uv_of(*best)
            px, py = self.reach.stage_of_canon(
                pose, pu - self.map.ru, pv - self.map.rv)
            self.log.point(
                stop=self.stops, pose=pose,
                rep_x=round(sm[0], 1), rep_y=round(sm[1], 1),
                pred_x=round(px, 1), pred_y=round(py, 1),
                resid_mm=round(bd, 1),
                cell_i=best[0], cell_j=best[1], state=pt.get('state', ''))
        # 짝이 안 지어진 셀은 **건드리지 않는다.** 묶였다고 단정하면 영영
        # 다시 안 가고, 실패로 단정하면 쓸데없이 재시도한다. pending 으로
        # 두면 다음에 들를 때 자연히 다시 계획된다.
        self.tie_matched, self.tie_ok = hit, n_ok
        self.get_logger().info(
            f'  결속 결과 반영 — 보고된 점 {len(self.points)}개 · '
            f'계획 {len(keys)}개 중 {hit}개 짝지음 ({n_ok}개 성공)')
        if hit < len(keys):
            self.get_logger().warning(
                f'  짝 못 지은 {len(keys) - hit}개는 pending 으로 둔다')

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
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
