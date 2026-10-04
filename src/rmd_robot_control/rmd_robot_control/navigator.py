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
        # ── 경로 파일에서 미션 만들기 ──────────────────────────────────────
        # `번호, x, y` (mm) 한 줄씩. x = 전진(+), y = **좌측(+)** 이다
        # (횡이동 부호 규약과 같다 — `/lateral/step` 의 +N 이 좌측이다).
        # 웨이포인트 사이를 전진·횡이동 단계로 바꾸고 각 지점에서 결속한다.
        # ⚠ 이 장비는 조향이 없다 — **대각선 이동은 못 한다.** x·y 가 동시에
        #   바뀌는 구간은 거부한다 (조용히 둘로 쪼개면 경로가 사람 의도와
        #   달라질 수 있다 — 어느 쪽을 먼저 갈지는 공간이 정한다).
        self.declare_parameter('path_file', '')
        self.declare_parameter('tie_at_waypoints', True)
        self.declare_parameter('lateral_mm_per_turn', 100.0)
        self.declare_parameter('lateral_max_turns', 4)
        self.declare_parameter('drive_timeout_sec', 180.0)
        self.declare_parameter('tie_timeout_sec', 900.0)
        self.declare_parameter('lateral_timeout_sec', 180.0)
        # ⚠ **복귀 중에 주행을 먼저 시작한다** (사용자 요청 2026-10-04). 결속을
        #   다 끝낸 뒤 검출 자세로 돌아가는 동안 — Z 는 이미 올라왔고 남은 것은
        #   XY 후퇴와 yaw 회전뿐이다 — 주행을 겹치면 웨이포인트당 4초쯤 줄고
        #   동작이 이어져 보인다.
        # ⚠ 조건이 둘이다: (1) 다음 단계가 주행·횡이동일 때만 (또 결속이면
        #   겹칠 수 없다), (2) Z 가 이 높이 위로 올라왔을 때만. Z 가 철근에
        #   들어가 있는 채로 주행하면 건이 끌린다.
        self.declare_parameter('drive_overlap', True)
        self.declare_parameter('drive_overlap_z_mm', -3.0)
        # ⚠ 겹칠 단계. 기본은 **주행만**이다 — 횡이동은 상부체를 들어올려
        #   옮기는 기구라 주행보다 큰 동작이고, 복귀 중 리프팅은 아직 검증하지
        #   않았다. 검증되면 'lateral' 을 더하면 된다.
        self.declare_parameter('drive_overlap_kinds', ['drive'])

        self.t_drive = float(self.get_parameter('drive_timeout_sec').value)
        self.t_tie = float(self.get_parameter('tie_timeout_sec').value)
        self.t_lat = float(self.get_parameter('lateral_timeout_sec').value)
        self.overlap = bool(self.get_parameter('drive_overlap').value)
        self.overlap_z = float(self.get_parameter('drive_overlap_z_mm').value)
        self.overlap_kinds = tuple(
            self.get_parameter('drive_overlap_kinds').value or ())

        self.steps, bad = [], None
        path = str(self.get_parameter('path_file').value or '')
        try:
            texts = (self._from_path(path) if path
                     else (self.get_parameter('mission').value or []))
            self.steps = [Step(t) for t in texts]
        except (ValueError, OSError) as e:
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

    # ---- 경로 파일 ---------------------------------------------------------
    def _from_path(self, path):
        """`번호, x, y` (mm) 목록을 미션 단계로 바꾼다.

        x = 전진(+), y = 좌측(+). 웨이포인트마다 결속하고, 사이를 전진·횡이동
        으로 잇는다. 횡이동은 `lateral_mm_per_turn` 단위로만 가므로 나머지가
        남으면 거부한다 — 몰래 반올림하면 사람이 센 거리와 달라진다.
        """
        mm_t = float(self.get_parameter('lateral_mm_per_turn').value)
        max_t = int(self.get_parameter('lateral_max_turns').value)
        tie = bool(self.get_parameter('tie_at_waypoints').value)
        pts = []
        for i, line in enumerate(open(path, encoding='utf-8'), 1):
            line = line.split('#')[0].strip()
            if not line:
                continue
            f = [v.strip() for v in line.split(',')]
            if len(f) < 3:
                raise ValueError(f"{path}:{i} — '번호, x, y' 세 값이 필요하다: {line!r}")
            pts.append((float(f[1]), float(f[2])))
        if len(pts) < 2:
            raise ValueError(f"{path} — 웨이포인트가 {len(pts)}개뿐이다 (2개 이상)")
        out = ['tie'] if tie else []
        for k in range(1, len(pts)):
            dx = pts[k][0] - pts[k - 1][0]
            dy = pts[k][1] - pts[k - 1][1]
            if abs(dx) > 1.0 and abs(dy) > 1.0:
                raise ValueError(
                    f"{path} — {k}→{k+1} 구간이 대각선이다 "
                    f"(dx={dx:+.0f}, dy={dy:+.0f}). 이 장비는 조향이 없어 "
                    f"전진과 횡이동을 따로 해야 한다 — 경로를 두 줄로 나누라")
            if abs(dx) > 1.0:
                out.append(f'drive {dx:.0f}')
            elif abs(dy) > 1.0:
                turns = dy / mm_t
                if abs(turns - round(turns)) > 1e-6:
                    raise ValueError(
                        f"{path} — {k}→{k+1} 의 횡이동 {dy:+.0f}mm 가 "
                        f"{mm_t:.0f}mm 의 배수가 아니다 (기구가 회전 단위로만 간다)")
                turns = int(round(turns))
                # 한 번에 갈 수 있는 회전 수가 제한돼 있다 — 나눠 보낸다
                while turns:
                    step = max(-max_t, min(max_t, turns))
                    out.append(f'lateral {step:+d}')
                    turns -= step
            else:
                continue                      # 제자리 — 단계를 만들지 않는다
            if tie:
                out.append('tie')
        self.get_logger().info(
            f"경로 {path} — 웨이포인트 {len(pts)}개 → {len(out)}단계")
        return out

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

    def _next_kind(self):
        i = self.cur + 1
        return self.steps[i].kind if i < len(self.steps) else None

    def _do_tie(self, s):
        if self._sent == 0.0:
            if self.plan is None:
                self.detail = '/plan/status 를 기다린다 — tying_planner 가 떠 있는가'
                return
            # 앞 단계의 복귀가 아직 끝나지 않았으면 기다린다 (겹침으로 미션이
            # 먼저 넘어온 경우다) — 복귀 중에 새 바퀴를 시작하면 엉킨다
            if (self.plan or {}).get('phase') in ('ready', 'detect', 'run', 'park'):
                self.detail = f"앞 복귀가 끝나기를 기다린다 ({self.plan.get('phase')})"
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
            # ⚠ 복귀 실패는 **결속을 무효로 만들지 않지만 끝 상태를 깨뜨린다.**
            #   그대로 "완료" 로 넘기면 다음 웨이포인트가 엉뚱한 자세에서
            #   시작한다 — 2026-10-04 에 3번 자세로 끝났는데 미션은 완료라고
            #   보고했다. 상태에 남기고 경고한다 (`ready` 가 다음 바퀴에서
            #   바로잡지만, 미션 마지막 단계면 바로잡을 기회가 없다).
            if (self.plan or {}).get('park_ok') is False:
                s.state = '완료(복귀 실패)'
                self.get_logger().warning(
                    f"{s.text} — 결속은 됐지만 검출 자세 복귀가 실패했다. "
                    f"끝 자세가 다를 수 있다: {(self.plan or {}).get('detail', '')}")
            return self._next()
        if ph == 'failed':
            s.state = f"실패: {(self.plan or {}).get('detail', '')}"
            return self._abort(f'{s.text} — {s.state}')
        # ⚠ **복귀 중이면 다음 주행을 먼저 시작한다.** Z 가 올라온 뒤에만,
        #   그리고 다음 단계가 주행·횡이동일 때만이다.
        if (self.overlap and ph == 'park'
                and self._next_kind() in self.overlap_kinds):
            z = (self.plan or {}).get('z_mm')
            if z is not None and z >= self.overlap_z:
                s.state = '완료(복귀는 계속)'
                self.get_logger().info(
                    f"[{s.kind}] 복귀 중 — Z {z:.1f}mm 올라옴, "
                    f"다음 {self._next_kind()} 를 먼저 시작한다")
                return self._next()
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
