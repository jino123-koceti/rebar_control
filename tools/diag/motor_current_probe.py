#!/usr/bin/env python3
"""모터 전류 프로브 — 프리즈 직전 **토크 전류 파형**을 남긴다.

## 왜 (2026-08-14)
"모터에 부하가 걸려 순간 소비 토크가 높아질 때 죽는 것 같다"는 관측을 검증하기 위함.
지금까지 배제된 것: 자원 고갈(freeze_probe 5회) · Jetson 입력 전압 강하(volt_monitor,
프리즈 직전까지 18.30~18.44V 평탄) · 카메라 대수(GMSL 3→2로 줄였더니 오히려 20초 만에 사망).

⚠ `volt_monitor`가 보는 INA3221(1-0040)은 **Jetson 모듈 입력**(VDD_GPU_SOC 등)뿐이다.
   주행 모터는 별도 전원 계통이라 **모터 서지는 거기 안 잡힌다.** 그래서 모터 전류를
   CAN 피드백에서 직접 받아 따로 남긴다.

## 데이터 출처
`can_parser`가 RMD 0xA2 응답에서 토크 전류를 파싱해 `/motor_feedback`으로 105Hz 발행:
  motor_id / current_speed / current_position / **current_current(mA)** / temperature / error_code
`can_parser`는 20A 초과 시 error_code=0x04(Overcurrent)도 실어준다.

## 기록 방식
**매 레코드 fsync.** 프리즈는 하드 리셋이라 fsync 없이는 마지막 몇 분이 통째로 날아간다
(ROS 로그·journald가 그렇게 4번 유실됐다).
105Hz를 그대로 fsync하면 과하므로 **모터별로 `hz`(기본 20Hz)로 다운샘플**하되,
그 사이의 **최대 절대전류**를 함께 남긴다 — 스파이크를 놓치지 않기 위함이다.

판독:
  프리즈 직전 전류 스파이크 있음 → 사용자 가설 확정. 전원/배선/필터 쪽 대응
  전류 평온한데 사망           → 모터 부하도 무관. 남은 건 EMI/그라운드 결합

사용:
    python3 tools/diag/motor_current_probe.py
    python3 tools/diag/motor_current_probe.py --analyze /var/log/robot_control/motor_current.csv
"""
import argparse
import os
import time

OUT = '/var/log/robot_control/motor_current.csv'
COLS = ['time', 'up', 'motor', 'ma', 'ma_peak', 'speed', 'temp', 'err', 'n']


def analyze(path, tail=60):
    """uptime 되감김 = 리셋. 그 직전 구간을 모터별로 보여준다."""
    rows = [l.rstrip('\n').split(',') for l in open(path) if l.strip()]
    if len(rows) < 2:
        print('데이터 없음'); return
    hdr, data = rows[0], [r for r in rows[1:] if len(r) == len(rows[0])]
    iu = hdr.index('up')
    # 리셋 경계 = **BOOT 마커**만. (RESTART = 프로브만 재시작한 것이므로 제외)
    cuts = [i for i in range(len(data)) if data[i][2] == 'BOOT']
    if not cuts:
        print('리셋 흔적 없음. 마지막 구간만 표시.')
        cuts = [len(data)]
    for c in cuts[-2:]:
        seg = data[max(0, c - tail):c]
        print(f"\n{'='*78}\n리셋 직전 (마지막 기록 {data[c-1][0]}, up={data[c-1][iu]}s)\n{'='*78}")
        print(' | '.join(f'{h:>9}' for h in hdr))
        for r in seg:
            print(' | '.join(f'{v:>9}' for v in r))
        peaks = [abs(int(r[hdr.index('ma_peak')] or 0)) for r in seg]
        if peaks:
            print(f"\n  구간 최대 |전류| = {max(peaks)} mA ({max(peaks)/1000:.2f} A)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--hz', type=float, default=20.0, help='모터별 기록 주기')
    ap.add_argument('--analyze', metavar='CSV')
    a = ap.parse_args()
    if a.analyze:
        analyze(a.analyze); return

    import rclpy
    from rebar_base_interfaces.msg import MotorFeedback

    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    new = not os.path.exists(a.out) or os.path.getsize(a.out) == 0
    f = open(a.out, 'a')
    if new:
        f.write(','.join(COLS) + '\n')
    # 시작 마커. ⚠ **부팅과 프로브 재시작을 구분한다** — 둘 다 BOOT으로 찍었더니
    #   분석 시 리셋 경계가 섞여 "전류 0.06A에서 프리즈"처럼 보였다(2026-08-14).
    #   uptime 60초 미만이면 갓 부팅한 것 = 직전에 프리즈/재부팅이 있었다는 뜻.
    up0 = float(open('/proc/uptime').read().split()[0])
    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{up0:.0f},"
            f"{'BOOT' if up0 < 60 else 'RESTART'},,,,,,\n")
    f.flush(); os.fsync(f.fileno())

    period = 1.0 / a.hz
    acc = {}          # motor_id -> [peak_abs_ma, last_msg, count, last_write_t]

    rclpy.init()
    node = rclpy.create_node('motor_current_probe')

    def cb(m):
        st = acc.setdefault(m.motor_id, [0, None, 0, 0.0])
        ma = int(m.current_current)
        if abs(ma) > abs(st[0]):
            st[0] = ma                 # 구간 내 최대 절대전류(부호 유지)
        st[1] = m
        st[2] += 1

    node.create_subscription(MotorFeedback, '/motor_feedback', cb, 50)
    print(f'motor_current_probe → {a.out}  (모터별 {a.hz:.0f}Hz, 매 레코드 fsync)',
          flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            up = float(open('/proc/uptime').read().split()[0])
            for mid, st in acc.items():
                # ⚠ **새 메시지가 있을 때만** 기록한다 (st[2] = 마지막 기록 이후 수신 수).
                #   이 조건이 없으면 직전 값을 계속 재기록해 같은 타임스탬프가 수십 번
                #   찍히고(2026-08-14 실측), 모터 7개 × 20Hz = 초당 140 fsync가 되어
                #   프로브 자신이 스토리지 부하가 된다.
                if st[1] is None or st[2] == 0 or now - st[3] < period:
                    continue
                m = st[1]
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{up:.0f},"
                        f"0x{mid + 0x100:03X},{int(m.current_current)},{st[0]},"
                        f"{m.current_speed:.0f},{m.temperature},{m.error_code},{st[2]}\n")
                f.flush()
                os.fsync(f.fileno())   # ★ 하드 리셋에서 살아남게
                st[0] = 0; st[2] = 0; st[3] = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
