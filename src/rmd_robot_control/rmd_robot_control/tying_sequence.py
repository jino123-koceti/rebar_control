#!/usr/bin/env python3
"""L4 — 결속 지점 하나를 "자세 선택 → 회전 → XY 이동" 으로 묶는다.

⚠ 후퇴 목표는 창 **경계에서 `transit_pad_mm` 안쪽**이다. 경계에 붙여 놓고 돌리면
여유가 0 이고, 자세 사이 범위는 표본 몇 점으로 가늠한 것이라 그 사이에 더 나쁜
각도가 있을 수 있다 (실측 셋 다 양 끝보다 나빴다).

`stage_node` 는 "축을 목표까지 어떻게 보내나" 만 안다. 이 노드는 그 위에서
**순서**를 정한다 (YEAR3_ARCHITECTURE.md §4 L4). 축도 CAN 도 직접 건드리지
않는다 — `/stage/*` 로만 명령한다.

## 왜 "선택 → 회전 → 이동" 이 아니라 네 단계인가

**회전은 중간 자세를 지나간다.** 1번에서 3번으로 가면 12시와 2번을 지나므로,
회전을 시작하기 전에 XY 가 *지나가는 자세 전부의 교집합* 안에 있어야 한다.
그래서 **후퇴**가 한 단계 들어간다:

    PRECHECK → RETRACT → ROTATE → MOVE_XY

이게 빠지면 회전 도중에 프레임을 친다. 2026-10-03 에 12시의 X 상한(361.0mm)을
안 재고 3번(383.2)을 최악으로 쓰던 동안 그 여유가 22mm 과했다.

## 자세는 왜 사분면으로 고르나

사용자 지정 규칙이다 (`axes.yaml` 의 `stage.pose_select`): X 가 xmax 쪽이면
1·4번, xmin 쪽이면 2·3번, Y 가 ymin 쪽이면 1·2번, ymax 쪽이면 3·4번. 겹치면
하나로 떨어진다. 중앙 ±deadband 는 "절반지점" 으로 보고 **자세를 바꾸지 않는다.**

## 토픽

  구독  /tying/goal     geometry_msgs/Point  결속 지점 (x, y) mm. z 는 안 쓴다
        /tying/abort    std_msgs/Empty       즉시 중단
        /stage/status   String(JSON)         현재 mm·자세·이동 여부
        /safety/state   SafetyState          비상정지·STOP
  발행  /stage/goal     geometry_msgs/Point  XY 이동
        /stage/yaw_pose std_msgs/Int32       자세 회전
        /stage/stop     std_msgs/Empty       중단 전파
        /tying/status   String(JSON)         단계·목표·사유

## 실패는 전부 거부다

호밍 안 됨 / 자세 판별 실패(자세 사이) / 목표가 선택 자세로 안 닿음 / 단계
타임아웃 — 모두 멈추고 사유를 남긴다. 추정해서 움직이지 않는다. yaw 는 끝
리미트가 없고 상부축에는 기구 끝단 보호가 없다.
"""

import json
import math
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String
from rebar_base_interfaces.msg import SafetyState

from .axis_config import (envelope_violation, load_envelope, load_pose_id,
                          load_pose_select, pose_label, select_pose,
                          transit_window)

NAN = float('nan')


class Step(Enum):
    IDLE = 'idle'
    PRECHECK = 'precheck'
    RETRACT = 'retract'        # XY 를 회전 안전창 안으로
    ROTATE = 'rotate'          # yaw 를 목표 자세로
    MOVE_XY = 'move_xy'        # 결속 지점으로
    DONE = 'done'
    FAILED = 'failed'


class TyingSequence(Node):
    def __init__(self):
        super().__init__('tying_sequence')

        self.declare_parameter('step_timeout_sec', 45.0)
        self.declare_parameter('arrive_tol_mm', 2.0)
        # 후퇴 목표를 회전 안전창 **경계에서 이만큼 안쪽**으로 잡는다.
        # 경계에 딱 붙여 놓고 회전하면 여유가 0 이다 — 도달 판정이 ±2mm 고,
        # 자세 사이 범위는 표본 몇 점으로 가늠한 것이라 그 사이에 더 나쁜 각도가
        # 있을 수 있다 (실측 셋 다 양 끝보다 나빴다). `envelope.margin_mm` 은
        # 센서·측정 오차용이고, 이것은 **회전 중 자세 변화분**을 위한 별도 여유다.
        self.declare_parameter('transit_pad_mm', 15.0)
        self.step_timeout = float(self.get_parameter('step_timeout_sec').value)
        self.tol = float(self.get_parameter('arrive_tol_mm').value)
        self.pad = float(self.get_parameter('transit_pad_mm').value)

        self.env = load_envelope()
        self.sel = load_pose_select()
        self.pose_id = load_pose_id('yaw')
        if self.env is None or self.sel is None:
            self.get_logger().error(
                "axes.yaml 의 stage.envelope / stage.pose_select 를 못 읽었다 — "
                "자세 선택도 회전 안전 검사도 할 수 없다. 목표를 거부한다")

        self.goal_pub = self.create_publisher(Point, '/stage/goal', 10)
        self.yaw_pub = self.create_publisher(Int32, '/stage/yaw_pose', 10)
        self.stop_pub = self.create_publisher(Empty, '/stage/stop', 10)
        self.status_pub = self.create_publisher(String, '/tying/status', 10)

        self.stage = None            # /stage/status 최신 JSON
        self.safety = None
        self.create_subscription(String, '/stage/status', self._on_stage, 10)
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 1)
        self.create_subscription(Point, '/tying/goal', self._on_goal, 10)
        self.create_subscription(Empty, '/tying/abort',
                                 lambda m: self._fail('중단 명령'), 10)

        self.step = Step.IDLE
        self.goal = None             # (x, y) mm
        self.want_pose = None
        self._retract_tgt = None     # 보낸 후퇴 목표 (도착까지 붙잡는다)
        self._rej0 = None            # 단계 시작 시점의 stage_node 거부 횟수
        self._passed = []            # 회전이 지나가는 자세들
        self.detail = '대기'
        self.t_step = 0.0
        self._sent = 0.0

        self.create_timer(0.2, self.tick)
        self.create_timer(0.5, self._publish)
        self.get_logger().info(
            "결속 시퀀스 시작 — /tying/goal (x,y mm) 로 결속 지점을 준다. "
            "자세 선택 → 후퇴 → 회전 → 이동 순으로 진행한다")

    # ---- 입력 --------------------------------------------------------------
    def _on_stage(self, msg):
        try:
            self.stage = json.loads(msg.data)
        except ValueError:
            pass

    def _on_safety(self, msg):
        self.safety = msg

    def _on_goal(self, msg):
        if self.step not in (Step.IDLE, Step.DONE, Step.FAILED):
            return self._reject(f'이미 진행 중이다 ({self.step.value})')
        if math.isnan(msg.x) or math.isnan(msg.y):
            return self._reject('결속 지점은 X·Y 가 둘 다 있어야 한다')
        self.goal = (float(msg.x), float(msg.y))
        self.want_pose = None
        self._retract_tgt = None
        self._enter(Step.PRECHECK, f'목표 x={self.goal[0]:.1f} y={self.goal[1]:.1f}mm')

    # ---- 단계 전이 ---------------------------------------------------------
    def _enter(self, step, detail):
        # 이 단계를 시작하는 시점의 거부 횟수를 기억한다 — 이후에 늘면 내 명령 탓이다
        self._rej0 = (self.stage or {}).get('rejects')
        self.step = step
        self.detail = detail
        self.t_step = time.time()
        self._sent = 0.0
        self.get_logger().info(f"[{step.value}] {detail}")
        self._publish()

    def _reject(self, why):
        self.get_logger().error(f"거부 — {why}")
        self.step = Step.FAILED
        self.detail = f'거부: {why}'
        self._publish()

    def _fail(self, why):
        """진행 중이던 것을 멈추고 실패로 끝낸다. 축 정지는 stage_node 가 한다."""
        if self.step in (Step.IDLE, Step.DONE, Step.FAILED):
            return
        self.stop_pub.publish(Empty())
        self.step = Step.FAILED
        self.detail = f'중단: {why}'
        self.get_logger().warning(self.detail)
        self._publish()

    # ---- 상태 읽기 ---------------------------------------------------------
    def _mm(self):
        c = (self.stage or {}).get('current_mm') or {}
        return c.get('x'), c.get('y')

    def _pose(self):
        return (self.stage or {}).get('pose')

    def _stage_busy(self):
        return bool((self.stage or {}).get('moving'))

    def _stage_detail(self):
        return (self.stage or {}).get('detail', '')

    # ---- 진행 --------------------------------------------------------------
    def tick(self):
        if self.step in (Step.IDLE, Step.DONE, Step.FAILED):
            return
        s = self.safety
        if s is not None and (s.estop or s.stop_switch or s.inputs_stale):
            return self._fail('안전 정지')
        if self.stage is None:
            return self._fail('/stage/status 가 없다 — stage_node 가 떠 있는가')
        # stage_node 가 거부했으면 사유를 그대로 올린다 (삼키면 진단이 어렵다).
        # ⚠ `detail` 문자열로 보면 **한참 전의 거부**에 걸린다 — 그 필드는 다음
        #   일이 생길 때까지 남는다. 2026-10-03 실장비에서 그래서 시작 즉시
        #   중단됐다 (몇 분 전 가드 시험의 "모르는 자세 7" 이 남아 있었다).
        #   **거부 횟수가 늘었는지**로 본다.
        rej = (self.stage or {}).get('rejects')
        if rej is not None and self._rej0 is not None and rej > self._rej0:
            self._rej0 = rej
            return self._fail(f"stage_node — {self._stage_detail()[4:]}")
        if time.time() - self.t_step > self.step_timeout:
            return self._fail(f'{self.step.value} 타임아웃 {self.step_timeout:.0f}s')

        getattr(self, '_do_' + self.step.value)()

    def _do_precheck(self):
        if self.env is None or self.sel is None:
            return self._reject('범위 표나 자세 선택 규칙이 없다')
        x, y = self._mm()
        if x is None or y is None:
            return self._reject('현재 X·Y mm 를 모른다 — 먼저 호밍하세요 (/homing_cmd)')
        cur = self._pose()
        if cur is None:
            return self._reject(
                f"yaw 자세를 못 가린다 — {(self.stage or {}).get('pose_detail')}. "
                f"호밍하면 1번 자세로 정렬된다")
        want, why = select_pose(self.sel, self.goal[0], self.goal[1], cur)
        if want is None:
            return self._reject(f'자세를 고를 수 없다 — {why}')
        bad = envelope_violation(self.env, want,
                                 {'x': self.goal[0], 'y': self.goal[1]})
        if bad:
            return self._reject(f'{why} 인데 그 자세로 닿지 않는다 — {bad}')
        self.want_pose = want
        if want == cur:
            # 자세가 그대로면 후퇴도 회전도 필요 없다
            return self._enter(Step.MOVE_XY, f'{why} (자세 유지) → 바로 이동')
        self._enter(Step.RETRACT, f'{why} — 회전 전 XY 후퇴')

    def _retract_target(self):
        """회전 안전창 안으로 옮길 목표. 이미 다 안에 있으면 None.

        **목표를 창에 끼워 맞춘 값**으로 보낸다 (현재 위치를 끼운 값이 아니다).
        둘 다 창 안이라 회전 안전은 같은데, 이렇게 하면 축이 **되돌아가지 않는다** —
        현재 위치를 끼우면 Y 가 300 → 275.1 → 100 처럼 왔다 갔다 한다.
        창 안으로 끼운 값은 통과하는 모든 자세의 교집합 안이므로 **현재 자세에서도
        반드시 허용된다** (현재 자세가 통과 자세에 포함되기 때문이다).

        한 축이라도 창 밖이면 두 축을 함께 보낸다. 안에 있던 축도 어차피 목표
        방향으로 가는 것이라 헛걸음이 아니고, 회전 뒤 이동이 없어질 때가 많다.
        """
        win, passed = transit_window(self.env, self.pose_id,
                                     self._pose(), self.want_pose)
        if not win:
            return None, [], None
        cur = dict(zip(('x', 'y'), self._mm()))
        if all(win[ax][0] <= cur[ax] <= win[ax][1] for ax in ('x', 'y')):
            return None, passed, win
        tgt = {ax: self._inside(g, win[ax])
               for ax, g in zip(('x', 'y'), self.goal)}
        return tgt, passed, win

    def _inside(self, v, rng):
        """창 안으로 끼우되 **경계에 붙이지 않는다** (`transit_pad_mm` 만큼 안쪽).

        창이 여유 두 배보다 좁으면 **중앙**을 쓴다 — 그때는 어느 쪽 경계에서도
        최대한 떨어지는 것이 최선이다.
        """
        lo, hi = rng
        if hi - lo <= 2 * self.pad:
            return (lo + hi) / 2.0
        return min(max(v, lo + self.pad), hi - self.pad)

    def _do_retract(self):
        """후퇴를 **끝까지 기다린 뒤** 회전으로 넘어간다.

        ⚠ 창 안에 들어간 순간 넘어가면 안 된다. `stage_node` 는 이동 중 회전
        명령을 **거부**하므로(한 번에 한 동작) 거기서 시퀀스가 깨진다.
        2026-10-03 시뮬레이션에서 그렇게 잡혔다 — 창 안에 들자마자 회전을 보내
        XY 와 yaw 가 동시에 움직였고, 실장비라면 거부로 멈췄다.
        """
        if self._retract_tgt is None:
            tgt, passed, win = self._retract_target()
            if win is None:
                return self._fail('회전 안전창을 계산할 수 없다')
            self._passed = passed
            if tgt is None:
                names = ', '.join(passed)
                return self._enter(Step.ROTATE,
                                   f'[{names}] 교집합 안 — 후퇴 불필요, 회전')
            if self._stage_busy():
                return
            self._retract_tgt = tgt
            self._sent = time.time()
            self.goal_pub.publish(Point(x=tgt['x'], y=tgt['y'], z=NAN))
            self.detail = ('후퇴 ' + ', '.join(f'{k}={v:.1f}'
                                             for k, v in sorted(tgt.items())))
            return
        if self._stage_busy() or time.time() - self._sent < 1.0:
            return
        cur = dict(zip(('x', 'y'), self._mm()))
        off = {ax: cur[ax] - v for ax, v in self._retract_tgt.items()
               if cur[ax] is None or abs(cur[ax] - v) > self.tol}
        if not off:
            names = ', '.join(self._passed)
            return self._enter(Step.ROTATE, f'후퇴 완료 — [{names}] 안에서 회전')
        # 멈췄는데 목표에 못 닿았다 — 안전 차단이나 리미트다. 다시 보내지 않는다
        return self._fail(
            '후퇴가 끝나지 않았다 ('
            + ', '.join(f'{k} {v:+.1f}mm 남음' for k, v in sorted(off.items()))
            + f") — stage_node: {self._stage_detail()}")

    def _do_rotate(self):
        if self._pose() == self.want_pose:
            return self._enter(Step.MOVE_XY,
                               f'{pose_label(self.want_pose)} 도착 → 결속 지점으로')
        if self._stage_busy():
            return
        if time.time() - self._sent > 1.5:
            self._sent = time.time()
            self.yaw_pub.publish(Int32(data=int(self.want_pose)))
            self.detail = f'{pose_label(self.want_pose)}로 회전 중'

    def _do_move_xy(self):
        x, y = self._mm()
        if (x is not None and y is not None
                and abs(x - self.goal[0]) <= self.tol
                and abs(y - self.goal[1]) <= self.tol):
            self.step = Step.DONE
            self.detail = (f'완료 — x={x:.1f} y={y:.1f}mm, '
                           f'{pose_label(self.want_pose)}')
            self.get_logger().info(f"[done] {self.detail}")
            return self._publish()
        if self._stage_busy():
            return
        if time.time() - self._sent > 1.0:
            self._sent = time.time()
            self.goal_pub.publish(Point(x=self.goal[0], y=self.goal[1], z=NAN))
            self.detail = f'결속 지점으로 이동 중 ({self.goal[0]:.1f}, {self.goal[1]:.1f})'

    # ---- 발행 --------------------------------------------------------------
    def _publish(self):
        x, y = self._mm()
        self.status_pub.publish(String(data=json.dumps({
            'step': self.step.value,
            'goal_mm': ({'x': self.goal[0], 'y': self.goal[1]}
                        if self.goal else None),
            'pose_now': self._pose(),
            'pose_want': self.want_pose,
            'current_mm': {'x': x, 'y': y},
            'detail': self.detail,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TyingSequence()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node._fail('노드 종료')
                time.sleep(0.1)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
