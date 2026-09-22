#!/usr/bin/env python3
"""좌우 주행모터 — **지령 vs 실제**를 나란히 보고 한쪽이 안 도는지 잡는다.

## 왜 둘을 같이 보나
"우측이 안 도는 것 같다"는 원인이 세 군데 중 하나다:

  ① 지령이 안 나감      → `/drive_control` 의 right_speed 가 0
  ② 지령은 나가는데 무응답 → 0x242 피드백이 안 오거나 speed 가 0
  ③ 둘 다 정상인데 안 돎  → 기구·부하 (소프트웨어 밖)

한쪽만 보면 셋을 못 가른다. 그래서 지령(`/drive_control`)과
실제(`/motor_feedback` 0xA2 응답: 속도·전류·온도)를 한 줄에 찍는다.

## 읽는 법
```
지령 L +0.100  R +0.100  |  실제 L  +512dps  1.2A 34°C   R    +0dps  0.1A 33°C   ❌ R 무회전
                                                          └ 지령은 갔는데 안 돈다 = ②
```
· `실제`가 양쪽 다 도는데 속도만 다르면 → 부하 차이거나 한쪽 슬립
· 전류가 한쪽만 크면 → 그쪽이 부하를 다 받고 있다(기구 간섭 의심)
· 피드백 자체가 안 오면 `무피드백` 으로 찍힌다 → CAN/모터 전원

⚠ 구독만 한다. 아무것도 발행하지 않으므로 주행에 영향이 없다.

## 사용
    python3 tools/diag/drive_motor_watch.py            # 움직일 때만 출력
    python3 tools/diag/drive_motor_watch.py --all      # 정지 중에도 계속
    python3 tools/diag/drive_motor_watch.py --hz 5     # 출력 빈도
"""
import argparse
import time

import rclpy
from rclpy.node import Node

from rebar_base_interfaces.msg import DriveControl, MotorFeedback

LEFT_ID = 0x41          # 0x141 - 0x100 ... 응답 0x241 - 0x200 = 0x41
RIGHT_ID = 0x42
MOVING = 0.005          # m/s. 이보다 크면 '움직이는 중'으로 본다
STALE = 1.0             # s. 이보다 오래된 피드백은 '무피드백'


class W(Node):
    def __init__(self, a):
        super().__init__('drive_motor_watch')
        self.a = a
        self.cmd = {'l': 0.0, 'r': 0.0, 't': 0.0}
        self.fb = {LEFT_ID: None, RIGHT_ID: None}
        self.last_print = 0.0
        self.n_cmd = 0
        self.stat = {LEFT_ID: {'n': 0, 'spd': 0.0, 'max': 0.0},
                     RIGHT_ID: {'n': 0, 'spd': 0.0, 'max': 0.0}}
        self.was_moving = False
        self.create_subscription(DriveControl, '/drive_control', self._cmd_cb, 10)
        self.create_subscription(MotorFeedback, '/motor_feedback', self._fb_cb, 50)
        print('▶ 좌우 주행모터 감시 — 리모콘으로 움직여 보세요 (Ctrl+C 종료)\n')

    def _cmd_cb(self, m):
        self.cmd = {'l': float(m.left_speed), 'r': float(m.right_speed),
                    't': time.time()}
        self.n_cmd += 1

    def _fb_cb(self, m):
        if m.motor_id not in self.fb:
            return
        # ⚠ 0x92(각도) 응답도 같은 토픽으로 오는데 speed=0으로 채워 보낸다.
        #   속도·전류가 둘 다 0인 프레임은 0xA2가 아닐 수 있으니 그대로 두되,
        #   판정은 '움직이는 동안의 최대값'으로 해서 그런 프레임에 안 속는다.
        self.fb[m.motor_id] = {
            'spd': float(m.current_speed), 'cur': int(m.current_current),
            'tmp': int(m.temperature), 'err': int(m.error_code), 't': time.time()}
        s = self.stat[m.motor_id]
        if abs(self.cmd['l']) > MOVING or abs(self.cmd['r']) > MOVING:
            s['n'] += 1
            s['spd'] += abs(float(m.current_speed))
            s['max'] = max(s['max'], abs(float(m.current_speed)))

    def _fmt(self, mid):
        f = self.fb[mid]
        if f is None:
            return '무피드백    '
        if time.time() - f['t'] > STALE:
            return f'낡음({time.time()-f["t"]:.0f}s)'
        e = ' ⚠E%d' % f['err'] if f['err'] else ''
        return f"{f['spd']:+6.0f}dps {f['cur']/1000.0:4.1f}A {f['tmp']:2d}°C{e}"

    def tick(self):
        now = time.time()
        moving = abs(self.cmd['l']) > MOVING or abs(self.cmd['r']) > MOVING
        if moving:
            self.was_moving = True
        if not moving and not self.a.all:
            if self.was_moving:          # 멈춘 순간 한 번 정리해서 보여준다
                self.was_moving = False
                self._summary()
            return
        if now - self.last_print < 1.0 / self.a.hz:
            return
        self.last_print = now

        # 판정: 지령이 나갔는데 실제가 안 도는 쪽 찾기
        mark = ''
        for side, mid, c in (('L', LEFT_ID, self.cmd['l']),
                             ('R', RIGHT_ID, self.cmd['r'])):
            f = self.fb[mid]
            if abs(c) <= MOVING:
                continue
            if f is None or now - f['t'] > STALE:
                mark += f'  ❌ {side} 무피드백'
            elif abs(f['spd']) < 5.0:
                mark += f'  ❌ {side} 무회전(지령 {c:+.3f})'
        print(f"지령 L {self.cmd['l']:+.3f}  R {self.cmd['r']:+.3f}  |  "
              f"실제 L {self._fmt(LEFT_ID)}   R {self._fmt(RIGHT_ID)}{mark}",
              flush=True)

    def _summary(self):
        print('  ── 이번 동작 요약 ──')
        for side, mid in (('L(0x141)', LEFT_ID), ('R(0x142)', RIGHT_ID)):
            s = self.stat[mid]
            avg = s['spd'] / s['n'] if s['n'] else 0.0
            print(f"     {side}  샘플 {s['n']:4d}  평균 {avg:6.0f}dps  "
                  f"최대 {s['max']:6.0f}dps")
        l, r = self.stat[LEFT_ID]['max'], self.stat[RIGHT_ID]['max']
        if max(l, r) > 5.0:
            lo, hi = min(l, r), max(l, r)
            print(f"     좌우 최대속도 비 {lo/hi*100:.0f}%"
                  f"{'  ⚠ 한쪽이 거의 안 돌았다' if lo < hi * 0.3 else ''}")
        for k in self.stat:
            self.stat[k] = {'n': 0, 'spd': 0.0, 'max': 0.0}
        print(flush=True)


def measure(n, sec):
    """고정 시간 기록 → 좌우 비교표. 스왑 전/후에 **같은 값**으로 돌릴 것."""
    print(f'▶ {sec:.0f}초 측정 시작 — 리모콘으로 전/후진을 몇 번 짧게 주세요\n')
    rec = {LEFT_ID: [], RIGHT_ID: []}
    cmds = []
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.02)
        c = max(abs(n.cmd['l']), abs(n.cmd['r']))
        if c > MOVING:
            cmds.append(c)
            for mid in rec:
                f = n.fb[mid]
                if f and time.time() - f['t'] < STALE and f['tmp'] > 0:
                    rec[mid].append((abs(f['spd']), abs(f['cur']) / 1000.0, f['tmp']))
    print(f'{"":8}{"샘플":>6}{"평균dps":>9}{"최대dps":>9}{"평균A":>8}{"최대A":>8}{"온도":>7}')
    print('-' * 56)
    out = {}
    for mid, name in ((LEFT_ID, '0x141'), (RIGHT_ID, '0x142')):
        r = rec[mid]
        if not r:
            print(f'{name:8}{"유효 샘플 없음":>20}')
            out[mid] = None
            continue
        sp = [x[0] for x in r]; cu = [x[1] for x in r]; tp = [x[2] for x in r]
        out[mid] = (sum(sp)/len(sp), max(sp), sum(cu)/len(cu), max(cu), max(tp))
        print(f'{name:8}{len(r):>6}{out[mid][0]:>9.0f}{out[mid][1]:>9.0f}'
              f'{out[mid][2]:>8.1f}{out[mid][3]:>8.1f}{max(tp):>6}°C')
    print('-' * 56)
    if cmds:
        print(f'  지령 구간 {len(cmds)}샘플, 평균 {sum(cmds)/len(cmds):.3f} m/s')
    a, b = out[LEFT_ID], out[RIGHT_ID]
    if a and b:
        print(f'  최대속도 비 (0x141/0x142) = {a[1]/b[1]*100:.0f}%' if b[1] > 1
              else '  0x142도 안 돌아 비교 불가 — 지령을 더 크게')
        for mid, name, v in ((LEFT_ID, '0x141', a), (RIGHT_ID, '0x142', b)):
            if v[1] < 5 and v[3] > 2.0:
                print(f'  ❌ {name}: 전류 {v[3]:.1f}A 인데 회전 {v[1]:.0f}dps '
                      f'= 스톨 (토크를 못 냄)')
    print('\n  ※ 커넥터 스왑 후 **같은 명령**으로 다시 측정해 이 표를 비교할 것.')
    print('     증상이 0x141 쪽에 남으면 하네스/전원, 모터를 따라가면 모터 불량.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hz', type=float, default=3.0, help='출력 빈도')
    ap.add_argument('--all', action='store_true', help='정지 중에도 계속 출력')
    # ★ 커넥터 스왑 전/후 비교용 **고정 프로토콜**.
    #   전원을 껐다 켜면 과열보호가 풀려 열 상태가 달라진다 → "돌아가나?" 눈으로
    #   보는 건 지표가 못 된다(사용자 지적). 그래서 **차가운 상태에서 같은 시간
    #   동안** 재고, 지령 대비 실제회전·전류를 숫자로 남겨 두 번을 직접 비교한다.
    ap.add_argument('--measure', type=float, default=0.0, metavar='초',
                    help='이 시간 동안 기록 후 비교표 출력 (스왑 전/후 동일하게)')
    a = ap.parse_args()
    rclpy.init()
    n = W(a)
    try:
        if a.measure > 0:
            measure(n, a.measure)
            return
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            n.tick()
    except KeyboardInterrupt:
        pass
    finally:
        print(f'\n■ 종료 — /drive_control {n.n_cmd}회 수신')
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
