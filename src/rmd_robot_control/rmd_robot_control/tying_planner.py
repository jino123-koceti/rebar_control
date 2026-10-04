#!/usr/bin/env python3
"""L5 — 검출된 교차점 전부를 순회하며 결속한다.

    /rebar/crossings (카메라 mm)  →  캘리브레이션 모델  →  자세별 스테이지 mm
      →  도달 가능한 것만 고르고 순서를 정해  →  한 점씩 tying_sequence 에

`tying_sequence` 는 **한 점**을 끝까지 수행한다. 이 노드는 그 위에서 **어느 점을
어느 자세로 어떤 순서로** 할지를 정한다. 축도 CAN 도 직접 건드리지 않는다.

## 왜 자세를 이 노드가 정하는가

캘리브레이션 모델은 자세별로 b 가 다르다 — 같은 교차점이 1번 자세에서 X 278mm
인데 3번 자세에서는 398mm 다 (오프셋 최대 122mm). 그러니 **모델을 가진 쪽만**
어느 자세의 좌표인지 안다. 사분면 규칙(`pose_select`)은 "스테이지 좌표가 주어졌을
때" 의 규칙이라 이 단계에서는 쓸 수 없다 — 좌표가 자세에 따라 달라지기 때문이다.
대신 **자세마다 좌표를 다 계산해 보고 도달 가능한 것 중에서 고른다.**

## 순서는 자세 변경을 최소화한다

자세 변경은 Z 올림 + XY 후퇴 + yaw 회전으로 20초가 넘고, 같은 자세 안의 XY
이동은 몇 초다. 그래서 **지금 자세로 갈 수 있는 점이 남아 있으면 계속 그 자세로**
하고, 하나도 못 가게 되면 그때 자세를 바꾼다 (사용자 지정 규칙, 2026-10-04).
자세 안에서는 가까운 점부터 간다 (최근접, 최적해가 아니라 단순함을 택했다).

⚠ 사분면 규칙(`axes.yaml` 의 `pose_select`)은 **쓰지 않는다.** 그것은 점마다
독립으로 자세를 정하므로 1→4→1 같은 왕복 회전이 생긴다.

## 한 점이 실패하면

**그 점만 건너뛰고 다음으로 간다.** 거부 사유를 남긴다. 단 안전 정지나
`/stage/status` 단절처럼 전체가 못 돌아가는 사유면 계획 전체를 멈춘다 —
하나씩 실패하며 끝까지 돌면 사유가 묻힌다.

## 토픽

  구독  /rebar/crossings  RebarGrid     검출 결과 (카메라 좌표 mm)
        /tying/status     String(JSON)  한 점 시퀀스의 단계
        /safety/state     SafetyState   비상정지·STOP
        /plan/start       std_msgs/Empty  검출 → 계획 → 실행
        /plan/abort       std_msgs/Empty  즉시 중단
  발행  /rebar/detect     std_msgs/Empty  검출 트리거
        /tying/goal_pose  std_msgs/Int32  다음 점의 자세
        /tying/goal       Point           다음 점 (스테이지 mm, z=결속깊이)
        /tying/abort      std_msgs/Empty  중단 전파
        /plan/status      String(JSON)    계획·진행·사유

⚠ 검출은 **X 를 뺀 상태**에서 해야 한다. X 가 380mm 를 넘으면 건이 카메라를
가린다 (2026-10-04 실측: X 395mm 에서 검출 2개, 389mm 에서 0개).
"""

import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String
from rebar_base_interfaces.msg import SafetyState
from rebar_base_interfaces.msg import RebarGrid

from .axis_config import envelope_violation, load_envelope, pose_label

NAN = float('nan')

MODEL = os.path.expanduser(
    '~/ros2_ws/src/rebar_control/data/calibration/stage_camera.yaml')


class Plan:
    """계획된 한 점. 자세와 스테이지 좌표가 짝지어져 있어야 의미가 있다."""

    __slots__ = ('idx', 'pose', 'xyz', 'cam', 'why', 'state')

    def __init__(self, idx, pose, xyz, cam, why):
        self.idx, self.pose, self.xyz, self.cam, self.why = idx, pose, xyz, cam, why
        self.state = '대기'

    def as_dict(self):
        return {'idx': self.idx, 'pose': self.pose,
                'stage_mm': [round(v, 1) for v in self.xyz],
                'why': self.why, 'state': self.state}


class TyingPlanner(Node):
    def __init__(self):
        super().__init__('tying_planner')

        self.declare_parameter('model_yaml', MODEL)
        self.declare_parameter('detect_wait_sec', 15.0)
        self.declare_parameter('point_timeout_sec', 180.0)
        # 검출을 받고 바로 계획하지 않는다 — 검출은 여러 프레임을 합치므로
        # 발행까지 시간이 걸린다. 이 시간 안에 안 오면 실패로 본다.
        self.declare_parameter('min_confidence', 0.5)
        # 교차점이 같은 자리에 두 번 잡히면 두 번 결속하게 된다. 스테이지
        # 좌표로 이만큼 안쪽이면 같은 점으로 본다.
        self.declare_parameter('dedup_mm', 25.0)
        # 계획만 세우고 멈춘다. **실장비에서 먼저 이것으로 본다** — 모델이
        # 어디로 보내려 하는지, 몇 점이 도달 가능한지, 순서가 맞는지를
        # 움직이기 전에 확인해야 한다. 시연에서도 계획을 보여준 뒤
        # `/plan/execute` 로 이어 실행하는 흐름에 쓴다.
        self.declare_parameter('plan_only', False)
        self.plan_only = bool(self.get_parameter('plan_only').value)
        # Z 를 목표에 싣지 않는다 — 시퀀스가 Z 단계를 건너뛰고 XY·자세만 한다.
        # **처음 돌릴 때 이것으로 본다.** 모델 Y 에 +8.3mm 상수 치우침이 남아
        # 있어서(2026-10-04 종단시험 3회), Z 를 내리면 건 끝이 교차점이 아니라
        # 철근 위를 누를 수 있다. 위치가 멀쩡한 것을 먼저 확인해야 한다.
        self.declare_parameter('send_z', True)
        self.send_z = bool(self.get_parameter('send_z').value)
        # ⚠⚠ **검출 자세를 코드가 지킨다.** 건이 내려가 있거나 X 가 앞으로
        #   나와 있으면 건이 카메라를 가려 먼 교차점이 안 잡힌다. 2026-10-04 에
        #   X 273mm·Z -77mm 에서 검출해 8점 중 3점만 계획됐다 — 문서에만 적어
        #   두었더니 그대로 당했다. 그래서 검출 전에 **직접 빼낸다.**
        self.declare_parameter('detect_x_mm', 0.0)
        self.declare_parameter('detect_z_mm', 0.0)
        self.declare_parameter('detect_tol_mm', 5.0)
        self.detect_x = float(self.get_parameter('detect_x_mm').value)
        self.detect_z = float(self.get_parameter('detect_z_mm').value)
        self.detect_tol = float(self.get_parameter('detect_tol_mm').value)
        self.wait_sec = float(self.get_parameter('detect_wait_sec').value)
        self.pt_timeout = float(self.get_parameter('point_timeout_sec').value)
        self.min_conf = float(self.get_parameter('min_confidence').value)
        self.dedup = float(self.get_parameter('dedup_mm').value)

        self.env = load_envelope()
        self.model = self._load_model(str(self.get_parameter('model_yaml').value))

        self.detect_pub = self.create_publisher(Empty, '/rebar/detect', 10)
        # ⚠ L5 가 L3 에 직접 명령하는 **유일한 경우**다. 검출 자세로 빼내는
        #   것은 결속 지점 이동이 아니라서 `tying_sequence` 의 일이 아니고,
        #   이 노드만이 "이제 검출한다" 를 안다. 시퀀스가 멈춰 있을 때만
        #   보내므로 두 commander 가 겹치지 않는다.
        self.stage_pub = self.create_publisher(Point, '/stage/goal', 10)
        self.pose_pub = self.create_publisher(Int32, '/tying/goal_pose', 10)
        self.goal_pub = self.create_publisher(Point, '/tying/goal', 10)
        self.abort_pub = self.create_publisher(Empty, '/tying/abort', 10)
        self.status_pub = self.create_publisher(String, '/plan/status', 10)

        self.grid = None
        self.grid_t = 0.0
        self.stage = None             # /stage/status — 지금 자세와 위치
        self.seq = None               # /tying/status 최신 JSON
        self.safety = None
        self.create_subscription(RebarGrid, '/rebar/crossings', self._on_grid, 10)
        self.create_subscription(String, '/tying/status', self._on_seq, 10)
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 1)
        self.create_subscription(String, '/stage/status', self._on_stage, 10)
        self.create_subscription(Empty, '/plan/start', lambda m: self._start(), 10)
        self.create_subscription(Empty, '/plan/execute',
                                 lambda m: self._execute(), 10)
        self.create_subscription(Empty, '/plan/abort',
                                 lambda m: self._stop('중단 명령'), 10)

        self.phase = 'idle'           # idle / detect / run / done / failed
        self.plans = []
        self.cur = -1
        self.detail = '대기'
        self.t_phase = 0.0
        self.t_point = 0.0
        self._sent = 0.0
        self._seq_seen = None         # 이 점에 대해 시퀀스가 움직이기 시작했는가

        self.create_timer(0.3, self.tick)
        self.create_timer(1.0, self._publish)
        if self.model is None:
            self.get_logger().error(
                f"캘리브레이션 모델을 못 읽었다 — {self.get_parameter('model_yaml').value}. "
                f"교차점을 스테이지 좌표로 바꿀 수 없다. /plan/start 를 거부한다")
        else:
            self.get_logger().info(
                f"결속 플래너 시작 — 자세 {sorted(self.model)} 모델 적재, "
                f"/plan/start 로 검출→계획"
                + ('→실행' if not self.plan_only else ' (실행은 /plan/execute)')
                + f", Z {'포함' if self.send_z else '제외'}")

    # ---- 모델 --------------------------------------------------------------
    def _load_model(self, path):
        """`stage_camera.yaml` 의 자세별 A·b. 자세 오프셋은 b 에 녹아 있다."""
        try:
            d = yaml.safe_load(open(path, encoding='utf-8')) or {}
            out = {}
            for p, v in (d.get('poses') or {}).items():
                out[int(p)] = (np.array(v['A'], float).reshape(3, 3),
                               np.array(v['b'], float).reshape(3))
            return out or None
        except Exception as e:
            self.get_logger().error(f"모델 읽기 실패 ({path}): {e}")
            return None

    def _to_stage(self, pose, cam):
        A, b = self.model[pose]
        return A @ np.asarray(cam, float) + b

    # ---- 입력 --------------------------------------------------------------
    def _on_grid(self, msg):
        self.grid, self.grid_t = msg, time.time()

    def _on_stage(self, msg):
        try:
            self.stage = json.loads(msg.data)
        except ValueError:
            pass

    def _on_seq(self, msg):
        try:
            self.seq = json.loads(msg.data)
        except ValueError:
            pass

    def _on_safety(self, msg):
        self.safety = msg

    # ---- 진행 --------------------------------------------------------------
    def _start(self):
        if self.phase in ('detect', 'run'):
            return self.get_logger().warning(f'이미 진행 중이다 ({self.phase})')
        if self.model is None or self.env is None:
            return self._fail('모델이나 작업영역 표가 없다')
        self.plans, self.cur = [], -1
        self.grid = None
        self._sent = 0.0
        self.phase, self.t_phase = 'ready', time.time()
        self.detail = '검출 자세로 빼내는 중'
        self.get_logger().info(
            f'[ready] 검출 자세로 — X {self.detect_x:.0f}mm, Z {self.detect_z:.0f}mm '
            f'(건이 카메라를 가리면 먼 교차점이 안 잡힌다)')
        self._publish()

    def _fail(self, why):
        self.phase, self.detail = 'failed', f'실패: {why}'
        self.get_logger().error(self.detail)
        self._publish()

    def _stop(self, why):
        if self.phase == 'run':
            self.abort_pub.publish(Empty())
        self.phase, self.detail = 'failed', f'중단: {why}'
        self.get_logger().warning(self.detail)
        self._publish()

    def tick(self):
        if self.phase in ('idle', 'planned', 'done', 'failed'):
            return
        s = self.safety
        if s is not None and (s.estop or s.stop_switch or s.inputs_stale):
            return self._stop('안전 정지')
        if self.phase == 'ready':
            return self._do_ready()
        if self.phase == 'detect':
            return self._do_detect()
        self._do_run()

    def _do_ready(self):
        """검출 자세로 빼낸다 — 건이 카메라를 가리지 않는 X·Z."""
        m = (self.stage or {}).get('current_mm') or {}
        x, z = m.get('x'), m.get('z')
        if x is None or z is None:
            if time.time() - self.t_phase > 10.0:
                return self._fail('/stage/status 에 X·Z 가 없다 — 호밍했는가')
            return
        if abs(x - self.detect_x) <= self.detect_tol \
                and abs(z - self.detect_z) <= self.detect_tol:
            self.phase, self.t_phase = 'detect', time.time()
            self.detail = '검출 요청'
            self.get_logger().info(
                f'[detect] 검출 자세 확보 (X {x:.1f}, Z {z:.1f}) — 교차점 검출 요청')
            self.detect_pub.publish(Empty())
            return self._publish()
        if time.time() - self.t_phase > 90.0:
            return self._fail(
                f'검출 자세로 못 갔다 (X {x:.1f}→{self.detect_x:.0f}, '
                f'Z {z:.1f}→{self.detect_z:.0f}) — stage_node 를 보라')
        if (self.stage or {}).get('moving'):
            return
        if time.time() - self._sent > 2.0:
            self._sent = time.time()
            # ⚠ **Z 를 먼저 올린다.** 건이 내려간 채로 X 를 빼면 철근을 긁는다
            if abs(z - self.detect_z) > self.detect_tol:
                self.stage_pub.publish(Point(x=NAN, y=NAN, z=self.detect_z))
                self.detail = f'Z 를 {self.detect_z:.0f}mm 로'
            else:
                self.stage_pub.publish(Point(x=self.detect_x, y=NAN, z=NAN))
                self.detail = f'X 를 {self.detect_x:.0f}mm 로'

    def _do_detect(self):
        if self.grid is None or self.grid_t < self.t_phase:
            if time.time() - self.t_phase > self.wait_sec:
                return self._fail(
                    f'{self.wait_sec:.0f}s 안에 /rebar/crossings 가 오지 않았다 — '
                    f'crossing_detector 와 카메라를 확인하라')
            return
        if not self.grid.valid:
            return self._fail(f'유효 검출 없음 — {self.grid.error_message}')
        if 'frame=camera' not in self.grid.error_message:
            return self._fail(
                f'검출이 카메라 좌표가 아니다 ({self.grid.error_message}) — '
                f'검출 노드의 transform_yaml 을 비워라. 변환은 이 노드가 한다')
        self.plans = self._build(self.grid.detections)
        if not self.plans:
            return self._fail(
                f'검출 {len(self.grid.detections)}개 중 도달 가능한 점이 없다')
        self.cur = -1
        self.detail = f'{len(self.plans)}점 계획'
        turns = sum(1 for a, b in zip(self.plans, self.plans[1:])
                    if a.pose != b.pose)
        self.get_logger().info(
            f"[plan] 검출 {len(self.grid.detections)}개 → 결속 {len(self.plans)}점, "
            f"자세 변경 {turns}회")
        for p in self.plans:
            self.get_logger().info(
                f"    {p.idx:2d}번  {pose_label(p.pose)}  "
                f"X {p.xyz[0]:6.1f}  Y {p.xyz[1]:6.1f}  Z {p.xyz[2]:6.1f}mm")
        if self.plan_only:
            self.phase = 'planned'
            self.detail = (f'{len(self.plans)}점 계획 (자세 변경 {turns}회) — '
                           f'실행하려면 /plan/execute')
            self.get_logger().info(f"[planned] {self.detail}")
            return self._publish()
        self.phase = 'run'
        self._next()

    def _execute(self):
        """`plan_only` 로 세워 둔 계획을 실행한다."""
        if self.phase != 'planned':
            return self.get_logger().warning(
                f'실행할 계획이 없다 (지금 {self.phase}) — /plan/start 를 먼저')
        self.phase, self.cur = 'run', -1
        self.get_logger().info(f'[run] 계획 {len(self.plans)}점 실행')
        self._next()

    # ---- 계획 --------------------------------------------------------------
    def _build(self, dets):
        """도달 가능한 (교차점, 자세) 를 골라 **자세 변경을 최소화**하는 순서로.

        사용자 지정 규칙이다 (2026-10-04): **지금 자세로 갈 수 있는 점이 남아
        있으면 계속 그 자세로 한다.** 하나도 못 가게 되면 그때 비로소 자세를
        바꾼다. 사분면 규칙(`pose_select`)은 쓰지 않는다 — 그건 점마다 자세를
        정하므로 왕복 회전이 생긴다.

        회전이 비싸기 때문이다. 실측으로 자세 변경은 Z 올림 + XY 후퇴 + yaw
        회전으로 20초가 넘고, 같은 자세 안의 XY 이동은 몇 초다.

        ⚠ **지금 자세에서 출발해야 한다.** "점이 많은 자세부터" 로 하면 1번
        자세에 서 있는데 4번에 점이 더 많을 때 불필요하게 먼저 돈다.
        """
        reach = []                 # [(검출번호, 카메라좌표, {자세: 스테이지좌표})]
        for i, d in enumerate(dets):
            if d.confidence < self.min_conf:
                continue
            cam = (d.x, d.y, d.z)
            ok = {}
            for pose in sorted(self.model):
                xyz = self._to_stage(pose, cam)
                if envelope_violation(self.env, pose,
                                      {'x': float(xyz[0]),
                                       'y': float(xyz[1])}) is None:
                    ok[pose] = xyz
            if ok:
                reach.append((i, cam, ok))

        left = list(reach)
        pose = (self.stage or {}).get('pose')
        if pose not in self.model:
            # 자세를 모르면(자세 사이) 점이 남은 **가장 작은** 자세로 시작한다
            pose = self._next_pose(left, 0)

        # ── 1단계: 자세를 순서대로 쓸며 점을 자세에 배정한다 ──────────────
        groups, taken = [], set()
        while left:
            here = [t for t in left if pose in t[2] and t[0] not in taken]
            if here:
                groups.append((pose, here))
                taken |= {t[0] for t in here}
                left = [t for t in left if t[0] not in taken]
            if not left:
                break
            nxt = self._next_pose(left, pose)
            if nxt is None:
                break               # 남은 점은 어떤 자세로도 못 간다
            pose = nxt

        # ── 2단계: 각 자세 안의 방문 순서를 **다음 자세 첫 점까지 포함해** 정한다 ──
        order = self._order_groups(groups, self._here(groups[0][0]) if groups
                                   else np.zeros(2))
        out = []
        for pose, ts in order:
            for idx, cam, ok in ts:
                # ⚠ `ts` 원소의 셋째는 **자세별 좌표 사전**이다 (좌표가 아니다)
                xyz = ok[pose]
                if any(q.pose == pose
                       and np.hypot(*(np.asarray(q.xyz[:2]) - xyz[:2])) < self.dedup
                       for q in out):
                    continue        # 같은 자리에 두 번 결속하지 않는다
                out.append(Plan(idx, pose, [float(v) for v in xyz], cam,
                                f'{pose_label(pose)} 유지'
                                if out and out[-1].pose == pose
                                else f'{pose_label(pose)}로 변경'))
        return out

    def _order_groups(self, groups, start):
        """자세 묶음들의 방문 순서를 **총 이동거리 최소**로 정한다.

        자세 안에서만 가까운 순으로 가면 틀린다 — 그 자세의 **마지막 점이 다음
        자세의 첫 점에서 멀면** 거기서 다 잃는다 (사용자 지적, 2026-10-04).
        그래서 "이 묶음을 어디서 끝낼지" 까지 같이 고른다.

        묶음 뒤에서부터 계산한다 (`_best_order`):

            F(묶음 i, 들어온 위치 e) = min   [ e 에서 시작해 묶음 i 를 전부 돌고
                                     끝점 j     j 에서 끝나는 최단거리
                                              + F(묶음 i+1, j) ]

        자세 변경 자체의 비용(회전·Z 올림)은 자세 순서가 이미 정해져 있으므로
        어느 순서를 골라도 같다 — 그래서 거리만 비교하면 된다.

        ⚠ 묶음이 8점을 넘으면 최근접으로 내려간다. 완전탐색은 점 수에 대해
        지수로 커지고, 8점이면 이미 20만 가지다 (실측 8검출 중 도달 6점이라
        넘을 일이 드물다).
        """
        if not groups:
            return []
        memo = {}

        def F(i, e):
            if i >= len(groups):
                return 0.0, []
            key = (i, round(float(e[0]), 1), round(float(e[1]), 1))
            if key in memo:
                return memo[key]
            pose, ts = groups[i]
            best = None
            for seq in self._candidates(ts, e, pose):
                d = 0.0
                cur = e
                for t in seq:
                    d += float(np.hypot(*(t[2][pose][:2] - cur)))
                    cur = t[2][pose][:2]
                rest, tail = F(i + 1, cur)
                if best is None or d + rest < best[0]:
                    best = (d + rest, [(pose, list(seq))] + tail)
            memo[key] = best
            return best

        return F(0, np.asarray(start, float))[1]

    def _candidates(self, ts, e, pose):
        """방문 순서 후보. 8점 이하면 전부, 넘으면 최근접 하나만."""
        import itertools
        if len(ts) <= 8:
            return itertools.permutations(ts)
        seq, left, cur = [], list(ts), np.asarray(e, float)
        while left:
            k = min(range(len(left)),
                    key=lambda m: float(np.hypot(*(left[m][2][pose][:2] - cur))))
            t = left.pop(k)
            cur = t[2][pose][:2]
            seq.append(t)
        return [tuple(seq)]

    def _next_pose(self, left, cur):
        """다음 자세. **번호가 커지는 방향으로만** 간다 (1→2→3→4).

        사용자 지정 순서다 (2026-10-04). "남은 점을 가장 많이 덮는 자세" 로 하면
        1→4→2 처럼 건너뛰어 총 회전량이 늘고, 시연에서 다음 동작을 예측할 수
        없다. 번호 순으로 쓸면 yaw 가 한 방향으로만 돌아 왕복이 없다.

        큰 쪽에 남은 점이 없으면 작은 쪽으로 되돌아간다 — 시작 자세가 1번이
        아니었을 때(호밍 직후가 아닌 경우)를 위한 것이고, 그때는 되돌아가는
        회전이 한 번 생긴다.
        """
        poses = sorted(self.model)
        has = lambda q: any(q in ok for _, _, ok in left)
        for q in poses:
            if q > cur and has(q):
                return q
        for q in poses:
            if q < cur and has(q):
                return q
        return None

    def _here(self, pose):
        """그 자세에서의 현재 XY. 모르면 원점 쪽에서 출발한다."""
        m = (self.stage or {}).get('current_mm') or {}
        if (self.stage or {}).get('pose') == pose \
                and m.get('x') is not None and m.get('y') is not None:
            return np.array([float(m['x']), float(m['y'])])
        return np.array([0.0, 0.0])

    # ---- 실행 --------------------------------------------------------------
    def _next(self):
        self.cur += 1
        if self.cur >= len(self.plans):
            ok = sum(1 for p in self.plans if p.state == '완료')
            self.phase = 'done'
            self.detail = f'{ok}/{len(self.plans)}점 완료'
            self.get_logger().info(f"[done] {self.detail}")
            return self._publish()
        p = self.plans[self.cur]
        p.state = '진행'
        self.t_point = time.time()
        self._sent = 0.0
        self._seq_seen = False
        self.detail = (f'{self.cur + 1}/{len(self.plans)} — {p.idx}번 교차점 '
                       f'{pose_label(p.pose)} ({p.xyz[0]:.1f}, {p.xyz[1]:.1f}, '
                       f'{p.xyz[2]:.1f})')
        self.get_logger().info(f"[run] {self.detail}")
        self._publish()

    def _do_run(self):
        p = self.plans[self.cur]
        if time.time() - self.t_point > self.pt_timeout:
            p.state = f'타임아웃 {self.pt_timeout:.0f}s'
            self.get_logger().warning(f"{p.idx}번 {p.state} — 건너뛴다")
            self.abort_pub.publish(Empty())
            return self._next()
        # 자세를 **먼저** 보내고 목표를 보낸다 (시퀀스가 그 순서를 요구한다)
        if self._sent == 0.0:
            if self.seq is None:
                self.detail = '/tying/status 를 기다린다 — tying_sequence 가 떠 있는가'
                return
            self._sent = time.time()
            self.pose_pub.publish(Int32(data=int(p.pose)))
            self.goal_pub.publish(Point(
                x=p.xyz[0], y=p.xyz[1],
                z=p.xyz[2] if self.send_z else float('nan')))
            return
        step = (self.seq or {}).get('step')
        if not self._seq_seen:
            # 묵은 상태를 보고 바로 넘어가지 않도록 **움직이기 시작**을 먼저 본다
            if step not in (None, 'idle', 'done', 'failed'):
                self._seq_seen = True
            elif time.time() - self._sent > 5.0:
                p.state = '시퀀스가 시작하지 않았다'
                self.get_logger().warning(f"{p.idx}번 {p.state} — 건너뛴다")
                return self._next()
            return
        if step == 'done':
            p.state = '완료'
            return self._next()
        if step == 'failed':
            p.state = f"실패: {(self.seq or {}).get('detail', '')}"
            self.get_logger().warning(f"{p.idx}번 {p.state} — 건너뛴다")
            return self._next()
        self.detail = (f"{self.cur + 1}/{len(self.plans)} — {p.idx}번 "
                       f"[{step}] {(self.seq or {}).get('detail', '')}")

    # ---- 발행 --------------------------------------------------------------
    def _publish(self):
        self.status_pub.publish(String(data=json.dumps({
            'phase': self.phase,
            'total': len(self.plans),
            'current': self.cur if 0 <= self.cur < len(self.plans) else None,
            'points': [p.as_dict() for p in self.plans],
            'detail': self.detail,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TyingPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node._stop('노드 종료')
                time.sleep(0.1)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
