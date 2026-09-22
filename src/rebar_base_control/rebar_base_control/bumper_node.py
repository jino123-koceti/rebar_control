#!/usr/bin/env python3
"""하부 주행부 범퍼 감시 → 방향별 주행 차단.

## 왜 방향별인가
범퍼가 눌렸다는 건 **이미 부딪혔다**는 뜻이다. 여기서 전 방향을 막으면
빠져나올 수단까지 사라져 그 자리에 갇힌다(deck_edge에서 겪은 문제와 같다).
→ **부딪힌 방향만 막고 반대 방향은 열어둔다.**

## 하드웨어
하부 주행부 DIO (192.168.0.5, board_id=1) — 상부 리미트 보드(192.168.0.6,
board_id=0)와 **별개 보드**다. FASTECH Plus-E 라이브러리로 읽는다.

  IN08 전방 / IN09 우측 / IN10 후방 / IN11 좌측   (ezi_io.yaml 설정 기준)

★ **접점 극성 = b접점 (2026-08-19 실측 확정)**
  평상시 ON, **눌리면 OFF**. 4개 범퍼를 각 2회씩 순서대로 눌러 8회 모두 확인했고,
  위 채널 매핑(IN08 전 / IN09 우 / IN10 후 / IN11 좌)도 그때 같이 검증했다.
  극성을 반대로 잡으면 **눌러도 안 서거나 아예 출발을 못 한다.**
  실측 원본: 눌림 시 raw 0x0F18 → front 0x0E18 / rear 0x0B18 / left 0x0718 / right 0x0D18

## 발행
  /bumper/{front,rear,left,right}  Bool   눌림 여부(극성 해석 후)
  /bumper_block                    String  JSON {"forward","backward","left","right","reason"}
                                           forward/backward는 deck_edge_block과 같은 스키마
                                           (drive_controller가 그대로 쓴다).
                                           left/right는 횡이동용으로 추가한 필드다.

## 통신 두절
읽기가 연속 실패하면 재연결한다(ezi_io_controller와 동일한 이유 —
2026-08-19 상부 보드가 연결을 잃고도 재연결하지 않아 리미트를 뚫은 사고).
두절 중에는 **직전 차단 상태를 유지**한다. 범퍼를 못 읽는 상태에서
"차단 없음"으로 발행하면 감지가 죽은 채로 계속 밀고 나간다.
"""
import json
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

library_path = os.environ.get('FASTECH_LIBRARY_PATH', '/home/koceti/python/PE/Library')
if os.path.exists(library_path):
    sys.path.append(library_path)
    _FASTECH_AVAILABLE = True
else:
    _FASTECH_AVAILABLE = False

try:
    if _FASTECH_AVAILABLE:
        from FAS_EziMOTIONPlusE import *
        from MOTION_DEFINE import *
        from ReturnCodes_Define import *
except ImportError:
    _FASTECH_AVAILABLE = False


class BumperNode(Node):

    def __init__(self):
        super().__init__('bumper_node')

        self.declare_parameter('ip_address', [192, 168, 0, 5])
        self.declare_parameter('board_id', 1)
        self.declare_parameter('update_rate', 20.0)
        # 채널 (ezi_io.yaml 하부 보드와 동일)
        self.declare_parameter('front_channel', 8)
        self.declare_parameter('right_channel', 9)
        self.declare_parameter('rear_channel', 10)
        self.declare_parameter('left_channel', 11)
        # ⚠ 극성. True = b접점(평상시 ON, 눌리면 OFF)
        self.declare_parameter('active_low', True)
        # 채터링 억제: 이 횟수만큼 연속 같은 값이어야 상태를 바꾼다.
        # 범퍼는 기계 접점이라 한 주기 튀는 값으로 급정지시키면 오히려 위험하다.
        self.declare_parameter('debounce_count', 2)
        # 해제 지연(초). 부딪힌 직후 접점이 붙었다 떨어졌다 하는 동안
        # 차단이 깜빡이면 주행이 튄다. 눌림은 즉시, 해제는 늦게.
        self.declare_parameter('release_delay_sec', 0.5)
        self.declare_parameter('reconnect_after', 5)
        # ★ [2026-09-02] 오래 복구 못 하면 **스스로 종료**한다(launch가 respawn).
        #   배경: 제어기가 외부 전원이라 **장비 전원만 내려도 젯슨은 살아 있다.**
        #   그동안 DIO는 꺼져 있는데 노드는 계속 붙으려 하고, 그러다 FASTECH
        #   라이브러리의 프로세스 전역상태가 꼬이면 **전원이 돌아와도 복구가 안 된다.**
        #   실측(2026-09-02): 284만 회 연속 실패로 **39시간 좀비**. 새 프로세스로는
        #   즉시 연결·읽기 성공 → 프로세스 교체가 유일한 복구책이다.
        #   ⚠ launch에 respawn=True가 **반드시** 같이 있어야 한다. 없으면 죽은 채
        #     안 돌아와 좀비보다 나쁘다.
        #   장비 전원 OFF는 상시 상황이라 너무 짧으면 재시작만 반복한다 → 120초.
        self.declare_parameter('exit_after_down_sec', 120.0)

        ip = self.get_parameter('ip_address').value
        self.ip = [int(x) for x in ip]
        self.board_id = int(self.get_parameter('board_id').value)
        rate = float(self.get_parameter('update_rate').value)
        self.channels = {
            'front': int(self.get_parameter('front_channel').value),
            'rear': int(self.get_parameter('rear_channel').value),
            'left': int(self.get_parameter('left_channel').value),
            'right': int(self.get_parameter('right_channel').value),
        }
        self.active_low = bool(self.get_parameter('active_low').value)
        self.debounce_n = int(self.get_parameter('debounce_count').value)
        self.release_delay = float(self.get_parameter('release_delay_sec').value)
        self.reconnect_after = int(self.get_parameter('reconnect_after').value)
        self._exit_after = float(self.get_parameter('exit_after_down_sec').value)
        self._down_since = None

        self.pressed = {k: False for k in self.channels}      # 확정 상태
        self._cand = {k: (False, 0) for k in self.channels}   # (후보값, 연속횟수)
        self._release_at = {k: None for k in self.channels}   # 해제 예정 시각
        self._read_fail = 0
        self._last_reconnect = 0.0
        self._last_block = None
        self.connected = False

        self.bumper_pubs = {
            k: self.create_publisher(Bool, f'/bumper/{k}', 10) for k in self.channels}
        self.block_pub = self.create_publisher(String, '/bumper_block', 10)

        if not _FASTECH_AVAILABLE:
            self.get_logger().error(
                f'FASTECH 라이브러리 없음 ({library_path}) — 범퍼 감시 불가')
        else:
            self._connect()

        self.timer = self.create_timer(1.0 / rate, self._loop)
        self.get_logger().info(
            f"범퍼 감시 시작: {'.'.join(map(str, self.ip))} board={self.board_id} "
            f"채널={self.channels} active_low={self.active_low}")

    # ── 연결 ────────────────────────────────────────────
    def _connect(self):
        try:
            if FAS_ConnectTCP(self.ip[0], self.ip[1], self.ip[2], self.ip[3],
                              self.board_id) == 0:
                self.get_logger().error('범퍼 DIO 연결 실패')
                self.connected = False
                return False
            self.get_logger().info(
                f"✅ 범퍼 DIO 연결: {'.'.join(map(str, self.ip))}")
            self.connected = True
            return True
        except Exception as e:
            self.get_logger().error(f'범퍼 DIO 연결 오류: {e}')
            self.connected = False
            return False

    def _reconnect(self):
        import time as _t
        now = _t.time()
        if now - self._last_reconnect < 1.0:      # 폭주 방지
            return
        self._last_reconnect = now
        self.get_logger().warn(f'범퍼 읽기 {self._read_fail}회 연속 실패 → 재연결')
        try:
            FAS_Close(self.board_id)
        except Exception:
            pass
        self.connected = False
        if self._connect():
            self._read_fail = 0
            self._down_since = None
        else:
            if self._down_since is None:
                self._down_since = now
            down = now - self._down_since
            if self._exit_after > 0 and down > self._exit_after:
                self.get_logger().fatal(
                    f'🛑 범퍼 DIO {down:.0f}s 복구 실패 → **노드 종료.** '
                    f'프로세스를 새로 띄워야만 복구되는 경우가 있다. launch가 되살린다.')
                raise SystemExit(1)

    # ── 주기 ────────────────────────────────────────────
    def _loop(self):
        if not _FASTECH_AVAILABLE:
            return

        raw = None
        if self.connected:
            try:
                r, inputs, _latch = FAS_GetInput(self.board_id)
                if r == FMM_OK:
                    raw = inputs
                    if self._read_fail:
                        self.get_logger().info(
                            f'✅ 범퍼 읽기 복구 (실패 {self._read_fail}회 후)')
                        self._read_fail = 0
                else:
                    self._read_fail += 1
            except Exception as e:
                self._read_fail += 1
                self.get_logger().error(f'범퍼 읽기 오류: {e}',
                                        throttle_duration_sec=5.0)
        else:
            self._read_fail += 1

        if raw is None:
            # ⚠ 못 읽는 동안은 직전 차단 상태를 그대로 유지한다.
            #   "차단 없음"으로 발행하면 감지가 죽은 채 계속 주행한다.
            self.get_logger().error(
                f'범퍼 상태 읽기 불가 ({self._read_fail}회) → 직전 차단상태 유지',
                throttle_duration_sec=3.0)
            if self._read_fail >= self.reconnect_after:
                self._reconnect()
            if self._last_block is not None:
                self._publish_block(self._last_block, force=True)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        for name, ch in self.channels.items():
            bit = bool(raw & (1 << ch))
            hit = (not bit) if self.active_low else bit
            self._update(name, hit, now)

        self._publish()

    def _update(self, name, hit, now):
        """디바운스 + 해제 지연을 적용해 확정 상태를 갱신한다."""
        cand, cnt = self._cand[name]
        self._cand[name] = (hit, cnt + 1 if hit == cand else 1)
        cand, cnt = self._cand[name]
        if cnt < self.debounce_n:
            return

        if cand and not self.pressed[name]:
            self.pressed[name] = True                 # 눌림은 즉시
            self._release_at[name] = None
            self.get_logger().warn(f'🛑 범퍼 눌림: {name}')
        elif not cand and self.pressed[name]:
            if self._release_at[name] is None:        # 해제는 지연
                self._release_at[name] = now + self.release_delay
            elif now >= self._release_at[name]:
                self.pressed[name] = False
                self._release_at[name] = None
                self.get_logger().info(f'✅ 범퍼 해제: {name}')
        elif cand:
            self._release_at[name] = None

    def _publish(self):
        for name, pub in self.bumper_pubs.items():
            m = Bool()
            m.data = self.pressed[name]
            pub.publish(m)

        # 부딪힌 방향만 막는다 — 축이 서로 독립이다.
        #   좌/우 범퍼는 **전후진을 막지 않는다**: 측면 접촉으로 전후진까지 막으면
        #   벽을 스치기만 해도 갇힌다. 반대로 전/후 범퍼도 횡이동을 막지 않는다.
        hit = [k for k in ('front', 'rear', 'left', 'right') if self.pressed[k]]
        self._publish_block({
            'forward': self.pressed['front'],
            'backward': self.pressed['rear'],
            'left': self.pressed['left'],
            'right': self.pressed['right'],
            'reason': ('범퍼 ' + '+'.join(hit)) if hit else '',
        })

    def _publish_block(self, block, force=False):
        # ⚠ 로그는 **차단 방향**이 바뀔 때만 찍는다.
        #   reason까지 비교하면 좌/우 범퍼처럼 방향을 막지 않는 접촉에도
        #   dict가 달라져 "차단 해제"가 헛돈다(실측으로 확인, 2026-08-19).
        #   발행 자체는 상태와 무관하게 매 주기 한다 — 수신측 워치독이
        #   신호 두절을 stale로 판정해야 하기 때문이다.
        prev = self._last_block
        dirs_of = lambda b: [d for d, k in (('전진', 'forward'), ('후진', 'backward'),
                                            ('좌', 'left'), ('우', 'right')) if b.get(k)]
        key = tuple(dirs_of(block))
        prev_key = None if prev is None else tuple(dirs_of(prev))
        if key != prev_key:
            if key:
                self.get_logger().warn(
                    f"🛑 범퍼 차단: {' '.join(key)} ({block['reason']})")
            elif prev is not None:
                self.get_logger().info('✅ 범퍼 차단 해제')
        self._last_block = block
        m = String()
        m.data = json.dumps(block, ensure_ascii=False)
        self.block_pub.publish(m)

    def destroy_node(self):
        try:
            if self.connected:
                FAS_Close(self.board_id)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BumperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM(서비스 재시작·systemd 정지)로 컨텍스트가 이미 내려간 경우다.
        # 잡지 않으면 종료 때마다 traceback이 찍혀 진짜 오류를 가린다.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():                     # 이미 shutdown됐으면 다시 부르면 RCLError
            rclpy.shutdown()


if __name__ == '__main__':
    main()
