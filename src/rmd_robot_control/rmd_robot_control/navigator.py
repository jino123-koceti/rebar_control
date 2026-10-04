#!/usr/bin/env python3
"""L4 — 웨이포인트 미션을 순서대로 실행한다.

시연은 **자율 작업이 아니라 미리 정한 순서**로 간다 (사용자 지정 2026-10-04).
그래야 다음 동작을 예측할 수 있고, 한 단계가 어긋나도 그 단계만 보면 된다.

미션은 문자열 목록이다 (`mission` 파라미터):

    ['tie', 'drive 350', 'tie', 'drive 350', 'tie']

  `tie`            지금 자리에서 검출→결속 한 바퀴 (`tying_planner`)
  `drive <mm>`     전진(+)·후진(-) 스텝 (`drive_node`)
  `lateral <회전>` 횡이동. +가 좌측, 1회전 = 100mm (`lateral_node`)
  `wait <초>`      그냥 기다린다 (사람이 확인할 틈)

## 왜 각 단계를 하위에 맡기는가

`tie` 한 번이 이미 **검출 자세 확보 → 검출 → 계획 → 점마다 결속 → 복귀** 다.
`tying_planner` 가 그걸 끝까지 하고 **검출 자세로 끝낸다** — 그래서 바로 다음
`drive` 로 이어진다. 이 노드는 순서와 "끝났는가" 만 본다.

## 한 단계가 실패하면 멈춘다

점 하나가 안 되는 것은 `tying_planner` 가 알아서 건너뛴다. 여기까지 올라온
실패는 **단계 자체가 안 된 것**(주행이 안 끝남, 검출 0개, 호밍 없음)이라,
다음 단계로 밀고 나가면 장비가 엉뚱한 자리에서 결속을 시도한다.

## 토픽

  구독  /mission/start   Empty         미션 실행
        /mission/abort   Empty         즉시 중단
        /drive/status    String(JSON)
        /plan/status     String(JSON)
        /lateral/complete String       "COMPLETE" / "FAILED"
        /safety/state    SafetyState
  발행  /drive/step      Float32
        /plan/start      Empty
        /lateral/step    Int32
        /drive/abort     Empty         중단 전파
        /plan/abort      Empty
        /mission/status  String(JSON)  단계·진행·사유
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Int32, String
from rebar_base_interfaces.msg import SafetyState


class Step:
    """미션 한 단계. 문자열에서 만든다."""

    KINDS = ('tie', 'drive', 'lateral', 'wait')

    def __init__(self, text):
        parts = str(text).split()
        self.kind = parts[0].lower()
        if self.kind not in self.KINDS:
            raise ValueError(f"모르는 단계: {text!r} (쓸 수 있는 것: "
                             f"{', '.join(self.KINDS)})")
        if self.kind == 'tie':
            self.arg = None
        else:
            if len(parts) < 2:
                raise ValueError(f"{self.kind} 는 값이 필요하다: {text!r}")
            self.arg = float(parts[1])
        self.text = str(text)
        self.state = '대기'

    def as_dict(self):
        return {'step': self.text, 'state': self.state}


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # 기본은 **한 자리에서 결속 한 바퀴**다. 주행이 들어간 미션은 명시적으로
        # 넘겨야 한다 — 기본값이 장비를 움직이면 안 된다.
        self.declare_parameter('mission', ['tie'])
        self.declare_parameter('drive_timeout_sec', 180.0)
        self.declare_parameter('tie_timeout_sec', 900.0)
        self.declare_parameter('lateral_timeout_sec', 180.0)

        self.t_drive = float(self.get_parameter('drive_timeout_sec').value)
        self.t_tie = float(self.get_parameter('tie_timeout_sec').value)
        self.t_lat = float(self.get_parameter('lateral_timeout_sec').value)

        self.steps, bad = [], None
        try:
            self.steps = [Step(t) for t in
                          (self.get_parameter('mission').value or [])]
        except ValueError as e:
            bad = str(e)

        self.drive_pub = self.create_publisher(Float32, '/drive/step', 10)
        self.plan_pub = self.create_publisher(Empty, '/plan/start', 10)
        self.lat_pub = self.create_publisher(Int32, '/lateral/step', 10)
        self.drive_abort = self.create_publisher(Empty, '/drive/abort', 10)
        self.plan_abort = self.create_publisher(Empty, '/plan/abort', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        self.drv = None
        self.plan = None
        self.lat = None             # 최근 /lateral/complete 와 받은 시각
        self.safety = None
        self.create_subscription(String, '/drive/status', self._on_drive, 10)
        self.create_subscription(String, '/plan/status', self._on_plan, 10)
        self.create_subscription(String, '/lateral/complete',
                                 lambda m: setattr(self, 'lat',
                                                   (m.data, time.time())), 10)
        self.create_subscription(SafetyState, '/safety/state', self._on_safety, 1)
        self.create_subscription(Empty, '/mission/start',
                                 lambda m: self._start(), 10)
        self.create_subscription(Empty, '/mission/abort',
                                 lambda m: self._abort('중단 명령'), 10)

        self.phase = 'idle'         # idle / run / done / failed
        self.cur = -1
        self.detail = '대기'
        self.t_step = 0.0
        self._sent = 0.0
        self._seen = False          # 이 단계에서 하위가 움직이기 시작했는가

        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self._publish)
        if bad:
            self.get_logger().error(f"미션을 읽을 수 없다 — {bad}")
        else:
            self.get_logger().info(
                f"미션 실행기 시작 — {len(self.steps)}단계, /mission/start 로 실행: "
                + ' → '.join(s.text for s in self.steps))

    # ---- 입력 --------------------------------------------------------------
    def _on_drive(self, msg):
        try:
            self.drv = json.loads(msg.data)
        except ValueError:
            pass

    def _on_plan(self, msg):
        try:
            self.plan = json.loads(msg.data)
        except ValueError:
            pass

    def _on_safety(self, msg):
        self.safety = msg

    # ---- 진행 --------------------------------------------------------------
    def _start(self):
        if self.phase == 'run':
            return self.get_logger().warning('이미 실행 중이다')
        if not self.steps:
            return self._fail('미션이 비어 있다')
        self.cur = -1
        self.phase = 'run'
        for s in self.steps:
            s.state = '대기'
        self.get_logger().info(f"[run] 미션 {len(self.steps)}단계 실행")
        self._next()

    def _fail(self, why):
        self.phase, self.detail = 'failed', f'실패: {why}'
        self.get_logger().error(self.detail)
        self._publish()

    def _abort(self, why):
        if self.phase == 'run':
            self.drive_abort.publish(Empty())
            self.plan_abort.publish(Empty())
        self.phase, self.detail = 'failed', f'중단: {why}'
        self.get_logger().warning(self.detail)
        self._publish()

    def _next(self):
        if 0 <= self.cur < len(self.steps):
            self.steps[self.cur].state = '완료'
        self.cur += 1
        if self.cur >= len(self.steps):
            self.phase = 'done'
            self.detail = f'{len(self.steps)}단계 완료'
            self.get_logger().info(f"[done] {self.detail}")
            return self._publish()
        s = self.steps[self.cur]
        s.state = '진행'
        self.t_step = time.time()
        self._sent = 0.0
        self._seen = False
        self.detail = f'{self.cur + 1}/{len(self.steps)} — {s.text}'
        self.get_logger().info(f"[{s.kind}] {self.detail}")
        self._publish()

    def tick(self):
        if self.phase != 'run':
            return
        sf = self.safety
        if sf is not None and (sf.estop or sf.stop_switch or sf.inputs_stale):
            return self._abort('안전 정지')
        s = self.steps[self.cur]
        limit = {'drive': self.t_drive, 'tie': self.t_tie,
                 'lateral': self.t_lat, 'wait': s.arg + 5.0 if s.arg else 10.0}[s.kind]
        if time.time() - self.t_step > limit:
            s.state = f'타임아웃 {limit:.0f}s'
            return self._abort(f'{s.text} — {s.state}')
        getattr(self, '_do_' + s.kind)(s)

    def _do_wait(self, s):
        if time.time() - self.t_step >= s.arg:
            self._next()
        else:
            self.detail = (f'{self.cur + 1}/{len(self.steps)} — 대기 '
                           f'{s.arg - (time.time() - self.t_step):.0f}s 남음')

    def _do_drive(self, s):
        if self._sent == 0.0:
            if self.drv is None:
                self.detail = '/drive/status 를 기다린다 — drive_node 가 떠 있는가'
                return
            self._sent = time.time()
            self._rej0 = (self.drv or {}).get('rejects')
            self.drive_pub.publish(Float32(data=float(s.arg)))
            return
        if (self.drv or {}).get('rejects') != self._rej0:
            s.state = f"거부: {(self.drv or {}).get('detail', '')}"
            return self._abort(f'{s.text} — {s.state}')
        # ⚠ **움직이기 시작하는 것을 먼저 본다.** 바로 moving 을 보면 직전
        #   단계의 묵은 False 에 걸려 끝난 줄로 안다.
        if not self._seen:
            if (self.drv or {}).get('moving'):
                self._seen = True
            elif time.time() - self._sent > 5.0:
                s.state = '주행이 시작하지 않았다'
                return self._abort(f'{s.text} — {s.state}')
            return
        if not (self.drv or {}).get('moving'):
            return self._next()
        self.detail = (f"{self.cur + 1}/{len(self.steps)} — {s.text}, "
                       f"남은 {(self.drv or {}).get('left_mm')}mm")

    def _do_tie(self, s):
        if self._sent == 0.0:
            if self.plan is None:
                self.detail = '/plan/status 를 기다린다 — tying_planner 가 떠 있는가'
                return
            self._sent = time.time()
            self.plan_pub.publish(Empty())
            return
        ph = (self.plan or {}).get('phase')
        if not self._seen:
            if ph in ('ready', 'detect', 'run', 'park'):
                self._seen = True
            elif time.time() - self._sent > 8.0:
                s.state = '결속 한 바퀴가 시작하지 않았다'
                return self._abort(f'{s.text} — {s.state}')
            return
        if ph == 'done':
            return self._next()
        if ph == 'failed':
            s.state = f"실패: {(self.plan or {}).get('detail', '')}"
            return self._abort(f'{s.text} — {s.state}')
        self.detail = (f"{self.cur + 1}/{len(self.steps)} — tie "
                       f"[{ph}] {(self.plan or {}).get('detail', '')}")

    def _do_lateral(self, s):
        if self._sent == 0.0:
            self._sent = time.time()
            self.lat = None
            self.lat_pub.publish(Int32(data=int(s.arg)))
            return
        if self.lat is None:
            self.detail = (f'{self.cur + 1}/{len(self.steps)} — 횡이동 '
                           f'{int(s.arg):+d}회전 ({abs(s.arg) * 100:.0f}mm)')
            return
        done, _t = self.lat
        if done.upper().startswith('COMPLETE'):
            return self._next()
        s.state = f'횡이동 실패: {done}'
        return self._abort(f'{s.text} — {s.state}')

    # ---- 발행 --------------------------------------------------------------
    def _publish(self):
        self.status_pub.publish(String(data=json.dumps({
            'phase': self.phase,
            'total': len(self.steps),
            'current': self.cur if 0 <= self.cur < len(self.steps) else None,
            'steps': [s.as_dict() for s in self.steps],
            'detail': self.detail,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Navigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node._abort('노드 종료')
                time.sleep(0.1)
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
