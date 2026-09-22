#!/usr/bin/env python3
"""직진 주행 튜닝 — **얼마나 휘는지 재고, 트림 값을 계산해 준다.**

## 왜 이 도구가 필요한가 (2026-09-14)
0x141·0x142를 둘 다 X4-36으로 통일하고 파라미터를 좌우 동일로 맞췄다
(`check_drive_pair.py` 실측: 가감속 1200/1200, 토크 69/69, 최고속 498/498,
위치계획 5000/5000, 펌웨어 2026042402 — 전부 동일).
**그것만으로 직진이 되지는 않는다.** 궤도 장력·마모·하중 분포가 다르면 같은 지령에도
좌우 이동거리가 달라진다. 남은 소프트 보정은 `can_sender` 의 `left/right_speed_trim`
뿐인데(0x30 PID는 이 펌웨어가 읽기를 지원 안 함 — 양쪽 다 0), **얼마를 넣을지는
재봐야만 안다.** 이 도구가 그 측정과 계산을 맡는다.

## 무엇으로 재나 — 0x92 멀티턴 절대각
`can_sender._request_drive_encoder` 가 주행모터 0x92를 **20 Hz로 폴링**하고
(`drive_encoder_rate`), `can_parser` 가 출력축 각도(0.01°/LSB)로 풀어
`/motor_feedback` 에 `status=0x92` 로 낸다. 이건 **절대값이라 적분오차가 없다** —
dps 평균을 적분하는 것보다 훨씬 정확하다. `encoder_odom` 이 쓰는 것과 같은 소스다.
  · 출력축 1° = 0.5000 mm  (2πr/360, r=0.02865 → 1회전 = 0.18 m)
  · 우측 모터는 물리적으로 반대로 달려 있어 **부호를 뒤집는다**(encoder_odom과 동일)

## 세 가지 모드
| 모드 | 로봇을 움직이나 | 무엇을 보나 |
|---|---|---|
| `--watch` | **아니오(구독만)** | 리모콘 manual 주행을 관찰. 직진 구간만 자동으로 골라 좌우 이동거리 비교 |
| `--run`   | **예(cmd_vel 발행)** | 정해진 속도·시간으로 자동 직진 |
| `--drift` | 아니오(계산기) | 줄자로 잰 횡편차 → 트림 값 |

**`--watch` 를 먼저 쓰는 걸 권한다.** 사람이 리모콘을 쥐고 있으니 언제든 멈출 수 있고,
이 도구는 아무것도 발행하지 않아 주행에 영향을 주지 않는다.

## ⚠ 엔코더가 못 보는 것
0x92는 **바퀴가 얼마나 돌았는지**만 안다. 궤도가 미끄러지면 바퀴는 제대로 돌았는데
차체는 휜다. 그래서 엔코더가 "좌우 일치"라고 해도 실제로 휘면 원인은 **기계**다
(궤도 장력부터 볼 것). 최종 확인은 줄자 → `--drift`.
기계 문제를 트림으로 덮으면 한쪽에 부하가 쏠린 채 숨는다 — 0x141이 2026-09-09에
죽은 게 정확히 그 패턴이었다.

## 사용법
```bash
# ① 관찰 (권장): 서비스 띄우고 리모콘 manual 로 직진 주행하면서
sudo systemctl start robot-control
python3 tools/motor/straight_drive_test.py --watch

# ② 자동 주행 시험 (로봇이 스스로 움직인다)
python3 tools/motor/straight_drive_test.py --run --speed 0.06 --sec 10

# ③ 줄자 실측 → 트림 계산 → yaml 기록
python3 tools/motor/straight_drive_test.py --drift 0.10 --dist 3.0 --apply
```
"""
import argparse
import math
import statistics
import time
from collections import deque

import yaml

CFG = '/home/koceti/ros2_ws/src/rebar_base_control/config/can_devices.yaml'
TRIM_MIN, TRIM_MAX = 0.8, 1.2     # can_sender 와 같은 클램프
LEFT, RIGHT = 0x41, 0x42          # can_parser 변환 ID (0x141 → 0x41)
MM_PER_DEG = 2.0 * math.pi * 0.02865 * 1000.0 / 360.0   # 0.5 mm/° (출력축)

# 직진 구간 자동 판정 (--watch)
STRAIGHT_TOL = 0.05     # 좌우 지령 차이가 이 비율 이내면 '직진 지령'
MOVING_MPS = 0.01       # 이보다 빨라야 '움직이는 중'
MIN_SEG_SEC = 2.0       # 이보다 짧은 구간은 버린다 (가감속만 들어 있다)
MIN_SEG_M = 0.20        # 이보다 짧게 간 구간도 버린다

# 정지거리 측정 — 지령이 0으로 떨어진 순간부터 실제로 설 때까지 얼마나 더 가나.
# 자율주행은 "정해진 지점에서 선다"가 목표라 이 값이 곧 위치오차의 하한이다.
STOPPED_DPS = 3.0       # 이보다 느리면 섰다고 본다
STOPPED_HOLD = 0.3      # 그 상태가 이만큼 지속돼야 확정

# 안전 감시 — 0x141을 죽인 패턴을 그대로 감시한다.
# ⚠ 순간값으로 보면 오탐이 난다(2026-09-14 실측: 정속인데 3~17A로 출렁인다).
#   → 최근 WARN_WIN 샘플의 **평균**으로 판정한다.
WARN_TEMP = 70          # °C
WARN_AMP = 5.0          # A (평균). 69% 토크상한이 ≈5.9A, 거버너 상한이 6.0A다.
WARN_WIN = 40           # 샘플 (20Hz → 2초)


# ───────────────────────────── 트림 계산 ─────────────────────────────
def trim_from_e(e, cur_l=1.0, cur_r=1.0):
    """상대 속도오차 e → 좌우 트림. 양쪽에 절반씩 나눠 준다.

    한쪽만 건드리면 평균 속도가 바뀌고, 최고속 포화 지점도 좌우가 달라진다.
    """
    return cur_l * (1.0 + e / 2.0), cur_r * (1.0 - e / 2.0)


def e_from_drift(y_m, dist_m, wheel_base):
    """줄자로 잰 횡편차 → 상대 속도오차.

    원호 근사: 거리 D를 가는 동안 횡으로 y 밀렸다면 방향각 θ ≈ 2y/D
    (y = D²/2R, θ = D/R). 차동구동에서 θ = (s_R − s_L)/W 이므로
        e = (s_R − s_L)/D = 2·y·W/D²
    y 부호는 **진행방향 기준 오른쪽으로 밀렸으면 +** → 오른쪽이 덜 갔다 → e < 0.
    """
    return -2.0 * y_m * wheel_base / (dist_m ** 2)


def clamp_report(name, v):
    c = max(TRIM_MIN, min(TRIM_MAX, v))
    if abs(c - v) > 1e-9:
        print(f"  ⚠ {name} {v:.4f} 는 허용범위({TRIM_MIN}~{TRIM_MAX}) 밖 → {c:.4f} 로 제한.")
        print(f"    이 정도로 벌어지면 트림이 아니라 **기계 점검** 대상이다.")
    return c


def show_trim(e, cur_l, cur_r, note=''):
    l, r = trim_from_e(e, cur_l, cur_r)
    print(f"\n[계산된 트림] {note}")
    print(f"  상대 속도오차 e = {e * 100:+.2f} %  "
          f"({'오른쪽이 더 감 → 차체는 왼쪽으로 휨' if e > 0 else '왼쪽이 더 감 → 차체는 오른쪽으로 휨'})")
    l = clamp_report('left_speed_trim', l)
    r = clamp_report('right_speed_trim', r)
    print(f"  left_speed_trim:  {cur_l:.4f} → {l:.4f}")
    print(f"  right_speed_trim: {cur_r:.4f} → {r:.4f}")
    return l, r


def apply_yaml(l, r):
    """can_devices.yaml 의 트림 두 줄만 치환한다(주석·서식 보존)."""
    import re
    src = open(CFG).read()
    for key, val in (('left_speed_trim', l), ('right_speed_trim', r)):
        pat = re.compile(rf'^(\s*{key}\s*:\s*)([-\d.]+)', re.M)
        if not pat.search(src):
            print(f"❌ {CFG} 에 {key} 가 없다 — 수동으로 넣을 것")
            return False
        src = pat.sub(lambda m: f"{m.group(1)}{val:.4f}", src)
    open(CFG, 'w').write(src)
    print(f"\n✅ {CFG} 기록 완료. 반영:  sudo systemctl restart robot-control")
    print("   적용 확인은 can_sender 기동 로그의 'Drive trim: L×… / R×…' 줄.")
    return True


# ─────────────────────── 공통: 구간 측정 로직 ───────────────────────
class Seg:
    """직진 한 구간의 엔코더 적분 결과."""
    __slots__ = ('t0', 'a0l', 'a0r', 't1', 'a1l', 'a1r', 'cmd', 'amp_l', 'amp_r',
                 'temp_l', 'temp_r', 'dps_l', 'dps_r')

    def __init__(self, t, al, ar, cmd):
        self.t0, self.a0l, self.a0r, self.cmd = t, al, ar, cmd
        self.t1 = self.a1l = self.a1r = None
        self.amp_l, self.amp_r, self.temp_l, self.temp_r = [], [], [], []
        self.dps_l, self.dps_r = [], []

    @property
    def sec(self):
        return (self.t1 - self.t0) if self.t1 else 0.0

    @property
    def dl(self):
        return (self.a1l - self.a0l) * MM_PER_DEG / 1000.0   # m

    @property
    def dr(self):
        return (self.a1r - self.a0r) * MM_PER_DEG / 1000.0   # m

    @property
    def ok(self):
        return (self.t1 is not None and self.sec >= MIN_SEG_SEC
                and abs((self.dl + self.dr) / 2.0) >= MIN_SEG_M)


def stop_report(stops):
    if not stops:
        print("\n[정지거리] 측정된 정지 없음 (달리다 멈추는 동작이 있어야 잰다)")
        return
    print("\n" + "=" * 74)
    print(f"[정지거리] 지령 0 → 실제 정지까지 (자율주행 위치오차의 하한)")
    print(f"{'#':>2} {'직전속도':>12} {'시간(s)':>9} {'더 간 거리':>12}")
    print("-" * 74)
    for i, (sec, dist, dps) in enumerate(stops, 1):
        print(f"{i:>2} {dps:7.0f} dps  {sec:9.2f} {dist*1000:9.0f} mm")
    print("-" * 74)
    d = statistics.mean([x[1] for x in stops]) * 1000
    t = statistics.mean([x[0] for x in stops])
    print(f"평균  {t:.2f}초 · {d:.0f} mm")
    if d > 30:
        print(f"\n  ⚠ {d:.0f}mm 는 결속 피치 대비 크다. 줄일 수 있는 곳은 세 군데다:")
        print("    ① 모터 속도계획 **감속**(0x43 index 3, 지금 1200 dps/s)")
        print("       → rmd_accel.py --index 3 --set <값>  (가속 index 2는 그대로 둘 것)")
        print("    ② drive_controller `decel_limit_mps2` (지금 0.8 m/s², 0이면 무제한)")
        print("    ③ `publish_frequency` 20Hz → 지령 지연 최대 50ms")
        print("    ⚠ **가속(index 2)은 올리지 말 것** — 전류 피크가 프리즈와 겹쳤던 값이다")
        print("       ([[robot_freeze_safety]] 2026-08-14: 피크 28.3A). 감속은 회생이라 다르다.")


def seg_report(segs, wheel_base, cur_l, cur_r):
    good = [s for s in segs if s.ok]
    if not good:
        print("\n❌ 쓸 만한 직진 구간이 없었다.")
        print(f"   조건: 좌우 지령 차이 {STRAIGHT_TOL*100:.0f}% 이내로 "
              f"{MIN_SEG_SEC:.0f}초 이상 · {MIN_SEG_M*100:.0f}cm 이상 주행.")
        print("   조이스틱을 한쪽으로 기울이지 말고 **똑바로 전진만** 시켜야 한다.")
        return

    print("\n" + "=" * 74)
    print(f"{'#':>2} {'향':>4} {'초':>5} {'좌(m)':>8} {'우(m)':>8} {'차(mm)':>8} "
          f"{'e(%)':>7} {'Δθ(°)':>7} {'3m환산(mm)':>11}")
    print("-" * 74)
    # ⚠ 전진 구간과 후진 구간을 **그냥 더하면 상쇄된다**(부호가 반대다).
    #   e 는 부호 있는 평균거리로 나누므로 방향과 무관하게 "어느 바퀴가 덜 도는가"를
    #   같은 부호로 준다 → **구간별 e 를 거리로 가중평균**하는 것이 맞다.
    #   덤으로 가중평균은 램프만 들어 있는 짧은 구간의 영향을 자동으로 줄여준다.
    we = wsum = 0.0
    for i, s in enumerate(good, 1):
        avg = (s.dl + s.dr) / 2.0
        e = (s.dr - s.dl) / avg if avg else 0.0
        dth = math.degrees((s.dr - s.dl) / wheel_base)
        # 3 m 갔을 때 예상 횡편차: y = e·D²/(2W)
        y3 = -e * 9.0 / (2.0 * wheel_base) * 1000.0
        way = '전진' if avg < 0 else '후진'      # 음수 = 전진 (2026-09-14 실측)
        print(f"{i:>2} {way} {s.sec:5.1f} {s.dl:8.3f} {s.dr:8.3f} {(s.dr-s.dl)*1000:8.1f} "
              f"{e*100:7.2f} {dth:7.2f} {y3:11.0f}")
        we += e * abs(avg)
        wsum += abs(avg)
    print("-" * 74)
    e = we / wsum if wsum else 0.0
    print(f"거리가중 평균   주행 {wsum:.3f} m                    {e*100:7.2f} %")

    # 부하 균형 — 0x141 사망 패턴 감시
    al = [v for s in good for v in s.amp_l]
    ar = [v for s in good for v in s.amp_r]
    tl = [v for s in good for v in s.temp_l]
    tr = [v for s in good for v in s.temp_r]
    if al and ar:
        print(f"\n[부하] 좌 평균 {statistics.mean(al):.2f}A / 최대 {max(al):.1f}A"
              f" · 최고 {max(tl) if tl else 0}°C")
        print(f"       우 평균 {statistics.mean(ar):.2f}A / 최대 {max(ar):.1f}A"
              f" · 최고 {max(tr) if tr else 0}°C")
        hi, lo = max(statistics.mean(al), statistics.mean(ar)), min(statistics.mean(al), statistics.mean(ar))
        if lo > 0.05 and hi / lo > 3.0:
            print(f"  ⚠ 전류가 한쪽에 {hi/lo:.1f}배 쏠려 있다 → **기계 점검**(궤도 장력·간섭).")
            print(f"    2026-09-09 혼합 구성에서 10배 쏠려 96°C로 모터가 죽었다.")

    # 판정
    print()
    if abs(e) < 0.005:
        print("✅ 엔코더 기준 좌우 이동거리 일치(0.5% 이내). **모터는 문제 없다.**")
        print("   그래도 차체가 휜다면 원인은 궤도 슬립·장력 — 엔코더는 그걸 못 본다.")
        print("   줄자로 편차를 재서 `--drift` 로 최종 확인할 것.")
    else:
        show_trim(e, cur_l, cur_r, '(엔코더 기준 — 궤도 슬립은 반영 못 한다)')
        print("\n   ※ 이건 **바퀴 회전량**만 맞추는 값이다. 적용 후 줄자로 재확인:")
        print("     python3 tools/motor/straight_drive_test.py --drift <y_m> --dist 3.0")


# ─────────────────────────── ROS 노드 ───────────────────────────
def make_node(cfg):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from rebar_base_interfaces.msg import DriveControl, MotorFeedback

    class T(Node):
        """구독은 항상. 발행은 --run 에서만 쓴다."""

        def __init__(self):
            super().__init__('straight_drive_test')
            self.create_subscription(MotorFeedback, '/motor_feedback', self.fb, 50)
            self.create_subscription(DriveControl, '/drive_control', self.dc, 50)
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.mode_pub = self.create_publisher(String, '/control_mode', 10)

            self.ang = {LEFT: None, RIGHT: None}     # 0x92 절대각(°), 우측은 부호반전
            self.live = {LEFT: None, RIGHT: None}    # (dps, A, °C) — 0xA2
            self.cmd = (0.0, 0.0)
            self.segs = []
            self.seg = None
            self.warned = set()
            # ⚠ 단위 주의: MotorFeedback.current_current 는 **mA** 다
            #   (can_parser 311행: `int(torque_current * 1000)`).
            #   2026-09-14에 /100 으로 읽어 **10배 뻥튀기**된 전류를 보고
            #   "부하 이상"이라고 오판할 뻔했다. drive_controller 도 /1000 을 쓴다.
            self.recent = {LEFT: deque(maxlen=WARN_WIN), RIGHT: deque(maxlen=WARN_WIN)}
            self.brake = {LEFT: 0, RIGHT: 0}          # 전류가 회전과 반대 = 제동 중
            self.motor = {LEFT: 0, RIGHT: 0}          # 전류가 회전과 같은 방향 = 구동 중
            # 정지거리 측정 상태
            self.stop_t0 = None
            self.stop_a0 = None
            self.stop_since = None
            self.stops = []                           # (초, 거리 m, 직전속도 dps)

        # ── 구독 ──
        def fb(self, m):
            if m.motor_id not in (LEFT, RIGHT):
                return
            if m.status == 0x92:
                # 우측은 물리적으로 반대로 달려 있다 (encoder_odom 과 같은 규약)
                self.ang[m.motor_id] = (m.current_position if m.motor_id == LEFT
                                        else -m.current_position)
            elif m.status == 0xA2:
                # ⚠ 0x90/0x92 응답도 같은 토픽에 오는데 그건 current=0, temp=0 이다.
                #   0xA2 만 골라야 전류·온도가 진짜 값이다.
                dps, amp = m.current_speed, m.current_current / 1000.0
                self.live[m.motor_id] = (dps, amp, m.temperature)
                self.recent[m.motor_id].append(abs(amp))
                # ★ 구동인가 제동인가 — 전류 부호와 회전 부호가 **반대면 제동**이다.
                #   2026-09-09 혼합 구성에서 한쪽이 브레이크로 작용해 반대쪽을 태웠다.
                #   그 패턴이 재발했는지 보는 가장 직접적인 지표다.
                if abs(dps) > 20 and abs(amp) > 0.3:
                    if dps * amp > 0:
                        self.motor[m.motor_id] += 1
                    else:
                        self.brake[m.motor_id] += 1
                self._safety(m.motor_id)

        def dc(self, m):
            self.cmd = (m.left_speed, m.right_speed)

        def _safety(self, mid):
            dps, amp, temp = self.live[mid]
            name = '좌 0x141' if mid == LEFT else '우 0x142'
            if temp >= WARN_TEMP and (mid, 'T') not in self.warned:
                self.warned.add((mid, 'T'))
                print(f"\n🔥 {name} {temp}°C — 즉시 정지할 것. "
                      f"(2026-09-09: 96°C에서 모터가 죽었다)")
            q = self.recent[mid]
            if len(q) == WARN_WIN and (sum(q) / len(q)) >= WARN_AMP \
                    and (mid, 'A') not in self.warned:
                self.warned.add((mid, 'A'))
                print(f"\n⚡ {name} 최근 2초 평균 {sum(q)/len(q):.1f}A — 부하 확인.")

        # ── 직진 구간 자동 절단 ──
        def tick(self):
            l, r = self.cmd
            mag = max(abs(l), abs(r))
            straight = (mag > MOVING_MPS and l * r > 0
                        and abs(l - r) / mag < STRAIGHT_TOL)
            have = self.ang[LEFT] is not None and self.ang[RIGHT] is not None
            now = time.time()

            if straight and have:
                if self.seg is None:
                    self.seg = Seg(now, self.ang[LEFT], self.ang[RIGHT], (l + r) / 2.0)
                self.seg.t1 = now
                self.seg.a1l, self.seg.a1r = self.ang[LEFT], self.ang[RIGHT]
                for mid, amps, temps, dpss in ((LEFT, self.seg.amp_l, self.seg.temp_l, self.seg.dps_l),
                                               (RIGHT, self.seg.amp_r, self.seg.temp_r, self.seg.dps_r)):
                    if self.live[mid]:
                        d, a, t = self.live[mid]
                        dpss.append(abs(d))
                        amps.append(abs(a))
                        temps.append(t)
            elif self.seg is not None:
                self._close()

            self._stop_watch(mag, now)

        def _stop_watch(self, mag, now):
            """지령이 0으로 떨어진 뒤 **실제로 설 때까지** 얼마나 더 가는지 잰다.

            자율주행은 "정해진 지점에서 선다"가 목표다. 이 값이 곧 위치오차의 하한이고,
            가감속을 올릴지 말지는 이걸 재고 나서 판단해야 한다.
            모터 0x43 감속·drive_controller 슬루·20Hz 지령주기가 **모두** 여기에 합쳐져
            나온다 — 어느 하나만 봐서는 알 수 없다.
            """
            have = self.ang[LEFT] is not None and self.ang[RIGHT] is not None
            if not have:
                return
            dps = max(abs(self.live[m][0]) if self.live[m] else 0.0
                      for m in (LEFT, RIGHT))

            if mag > MOVING_MPS:
                # 아직 지령이 살아 있다 — 마지막으로 달리던 속도를 기억해 둔다
                self.stop_t0 = None
                self.stop_since = None
                self.last_dps = dps
                return

            if self.stop_t0 is None:
                if getattr(self, 'last_dps', 0.0) < 20:
                    return                      # 애초에 안 달리고 있었다
                self.stop_t0 = now
                self.stop_a0 = ((self.ang[LEFT] + self.ang[RIGHT]) / 2.0)
                self.stop_since = None
                return

            if dps > STOPPED_DPS:
                self.stop_since = None
                return
            if self.stop_since is None:
                self.stop_since = now
                return
            if now - self.stop_since < STOPPED_HOLD:
                return

            # 확정
            a1 = (self.ang[LEFT] + self.ang[RIGHT]) / 2.0
            dist = abs(a1 - self.stop_a0) * MM_PER_DEG / 1000.0
            sec = self.stop_since - self.stop_t0
            self.stops.append((sec, dist, self.last_dps))
            print(f"  ⏹ 정지: 지령 0 이후 {dist*1000:.0f} mm 더 감 "
                  f"({sec:.2f}초, 직전 {self.last_dps:.0f} dps"
                  f" = {self.last_dps*MM_PER_DEG:.0f} mm/s)")
            self.stop_t0 = None
            self.stop_since = None
            self.last_dps = 0.0

        def _close(self):
            s, self.seg = self.seg, None
            if s is None:
                return
            self.segs.append(s)
            if s.ok:
                avg = (s.dl + s.dr) / 2.0
                e = (s.dr - s.dl) / avg if avg else 0.0
                print(f"  ▸ 구간 #{len([x for x in self.segs if x.ok])} 종료: "
                      f"{s.sec:.1f}초 · 좌 {s.dl:.3f}m / 우 {s.dr:.3f}m · "
                      f"차 {(s.dr-s.dl)*1000:+.0f}mm ({e*100:+.2f}%)")
            else:
                print(f"  · (짧은 구간 무시: {s.sec:.1f}초, {abs((s.dl+s.dr)/2)*100:.0f}cm)")

        # ── 발행 (--run 전용) ──
        def send(self, v):
            t = Twist()
            # ⚠ 부호 정리 (2026-09-14 실측으로 확정):
            #   · `/drive_control` 의 left/right 가 **음수일 때 전진**이다.
            #     (리모콘 AN3 전진 = -1.0~0.0 → manual 경로는 부호를 안 뒤집는다)
            #   · cmd_vel 경로는 `linear = -clamp(linear.x)` 로 한 번 뒤집으므로
            #     **cmd_vel linear.x 양수 = 전진** (ROS 표준과 일치).
            #   → 여기서 다시 뒤집으면 안 된다. 전에 그렇게 짜서 --run 이 후진할 뻔했다.
            t.linear.x = v
            t.angular.z = 0.0
            self.cmd_pub.publish(t)

        def set_mode(self, s):
            m = String()
            m.data = s
            self.mode_pub.publish(m)

    rclpy.init()
    return rclpy, T()


def live_line(n, el):
    def one(mid):
        v = n.live[mid]
        if not v:
            return '무피드백        '
        return f"{v[0]:+6.0f}dps {v[1]:5.1f}A {v[2]:3d}°C"
    seg = ''
    if n.seg is not None:
        seg = f"  [직진 {n.seg.sec:4.1f}s  차 {(n.seg.dr-n.seg.dl)*1000:+5.0f}mm]"
    return (f"  {el:5.1f}s  지령 L{n.cmd[0]:+.3f} R{n.cmd[1]:+.3f}"
            f" | L {one(LEFT)}  R {one(RIGHT)}{seg}")


def watch(a, cfg):
    rclpy, n = make_node(cfg)
    print("▶ 관찰 모드 — **아무것도 발행하지 않는다.** 리모콘(manual)으로 직진 주행할 것.")
    print(f"  좌우 지령 차이 {STRAIGHT_TOL*100:.0f}% 이내로 {MIN_SEG_SEC:.0f}초 이상 "
          f"간 구간만 자동으로 골라 잰다.")
    print("  가능하면 **같은 거리를 몇 번** 반복하면 평균이 안정된다. Ctrl-C = 종료·집계\n")
    t0 = time.time()
    last = 0.0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.05)
            n.tick()
            el = time.time() - t0
            if el - last >= 1.0:
                last = el
                if max(abs(n.cmd[0]), abs(n.cmd[1])) > MOVING_MPS or a.all:
                    print(live_line(n, el))
    except KeyboardInterrupt:
        print("\n집계 중…")
    finally:
        n._close()
        rclpy.shutdown()
    seg_report(n.segs, cfg['wheel_base'],
               cfg.get('left_speed_trim', 1.0), cfg.get('right_speed_trim', 1.0))
    stop_report(n.stops)
    brake_report(n)


def run(a, cfg):
    rclpy, n = make_node(cfg)
    speed = -a.speed if a.reverse else a.speed
    total = a.settle + a.sec + a.settle
    print(f"▶ 자동 주행 — 직진 지령 {speed:+.3f} m/s · {total:.0f}초.  Ctrl-C = 즉시 정지\n")
    t0 = time.time()
    last = 0.0
    try:
        for _ in range(5):
            n.set_mode('auto')
            rclpy.spin_once(n, timeout_sec=0.02)
            time.sleep(0.05)
        while time.time() - t0 < total:
            n.send(speed)
            rclpy.spin_once(n, timeout_sec=0.0)
            n.tick()
            el = time.time() - t0
            if el - last >= 1.0:
                last = el
                print(live_line(n, el))
            time.sleep(1.0 / a.hz)
    except KeyboardInterrupt:
        print("\n⛔ 중단 — 정지 지령")
    finally:
        for _ in range(10):                  # 정지를 확실히 밀어넣는다
            n.send(0.0)
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(0.02)
        n.set_mode('idle')
        rclpy.spin_once(n, timeout_sec=0.1)
        n._close()
        rclpy.shutdown()
    seg_report(n.segs, cfg['wheel_base'],
               cfg.get('left_speed_trim', 1.0), cfg.get('right_speed_trim', 1.0))
    stop_report(n.stops)
    brake_report(n)


def brake_report(n):
    """한쪽이 브레이크로 작용하고 있는지 — 2026-09-09에 모터를 태운 그 패턴."""
    print("\n[구동/제동]  전류 부호가 회전과 반대면 그 바퀴는 **끌려가며 제동**하는 중이다")
    bad = False
    for mid, name in ((LEFT, '좌 0x141'), (RIGHT, '우 0x142')):
        m, b = n.motor[mid], n.brake[mid]
        tot = m + b
        if not tot:
            print(f"  {name}: 판정할 샘플 없음")
            continue
        pct = b / tot * 100
        print(f"  {name}: 구동 {m} / 제동 {b}  → 제동 {pct:.0f}%")
        if pct > 40:
            bad = True
    if bad:
        print("  ⚠ 한쪽이 절반 가까이 제동하고 있다 → 반대쪽이 그 몫까지 민다.")
        print("    2026-09-09 혼합 구성에서 이 상태로 25초 만에 96°C가 났다. 기계 점검.")
    else:
        print("  ✅ 양쪽 다 구동 중 — 한쪽이 브레이크로 작용하는 상태는 아니다.")


def main():
    p = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter, description=__doc__)
    p.add_argument('--watch', action='store_true',
                   help='관찰 전용 — 리모콘 manual 주행을 보며 잰다 (발행 없음, 권장)')
    p.add_argument('--all', action='store_true', help='--watch 에서 정지 중에도 계속 출력')
    p.add_argument('--run', action='store_true',
                   help='자동 직진 시험 — **로봇이 스스로 움직인다**')
    p.add_argument('--speed', type=float, default=0.06, help='--run 속도 m/s (기본 0.06)')
    p.add_argument('--sec', type=float, default=10.0, help='--run 정속 구간 초 (기본 10)')
    p.add_argument('--settle', type=float, default=2.0, help='--run 앞뒤 가감속 여유 초')
    p.add_argument('--hz', type=float, default=20.0, help='--run cmd_vel 발행 주기')
    p.add_argument('--reverse', action='store_true', help='--run 후진으로')
    p.add_argument('--drift', type=float, help='줄자로 잰 횡편차 m (오른쪽으로 밀렸으면 +)')
    p.add_argument('--dist', type=float, default=3.0, help='--drift 의 주행거리 m (기본 3.0)')
    p.add_argument('--apply', action='store_true', help='계산된 트림을 can_devices.yaml 에 기록')
    a = p.parse_args()

    doc = yaml.safe_load(open(CFG))
    cfg = doc['can_sender']['ros__parameters']
    # 윤거(wheel_base)는 can_sender 가 아니라 drive_controller 섹션에 있다.
    cfg['wheel_base'] = doc['drive_controller']['ros__parameters']['wheel_base']
    cur_l = cfg.get('left_speed_trim', 1.0)
    cur_r = cfg.get('right_speed_trim', 1.0)

    if a.drift is not None:
        e = e_from_drift(a.drift, a.dist, cfg['wheel_base'])
        print(f"횡편차 {a.drift*1000:+.0f} mm / {a.dist:.1f} m, 윤거 {cfg['wheel_base']:.2f} m")
        print(f"현재 트림 L×{cur_l:.4f} / R×{cur_r:.4f} 에 누적 보정")
        l, r = show_trim(e, cur_l, cur_r, '(차체 실측 기준 — 이게 최종값이다)')
        if a.apply:
            apply_yaml(l, r)
        else:
            print("\n  기록하려면 --apply 를 붙일 것")
        return

    if a.watch:
        watch(a, cfg)
    elif a.run:
        run(a, cfg)
    else:
        p.print_help()


if __name__ == '__main__':
    main()
