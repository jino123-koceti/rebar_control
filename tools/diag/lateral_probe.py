#!/usr/bin/env python3
"""횡이동 모터(0x143) 실측 프로브 — 이동 중 속도·전류를 0x9C 폴링으로 잡는다.

## 왜 (2026-08-18)
0x143은 **위치명령(0xA4/0xA9)** 으로 제어되는데, 이 명령은 응답이 **1회뿐**이라
`/motor_feedback`에는 이동 중 속도·전류가 전혀 안 잡힌다. 그래서 "횡이동이 얼마나
힘든 동작인가"를 하루 종일 추정만 하고 있었다.
→ `0x9C`(Read Motor Status 2: temp/iq/speed/angle)를 고속 폴링해 직접 샘플링한다.

## 이 도구로 확인된 것
```
명령 200 dps: 4.80s | 속도 평균 60 최대 182 (편차 +203%) | 전류 평균 6.3 최대 30.7A
명령  80 dps: 5.53s | 속도 평균 54 최대  86 (편차  +60%) | 전류 평균 6.3 최대 30.4A
```
- 속도를 2.5배 낮춰도 **피크 전류가 안 변한다** → 부하가 관성이 아니라 **정적**(로봇을
  철근 위로 들어올리는 힘)이라는 뜻. 모션 프로파일로는 전류를 못 줄인다.
- 피크 30.5A는 `iq` 진폭값이고 rms로 환산하면 30.5/√2 = 21.6A.
  X4-36 데이터시트의 **최대 상전류 21.5A(rms)** 와 거의 정확히 일치 = 드라이버가
  스펙 한계에서 자르고 있다. **여유 0%** 라 성공/실패가 잡음으로 갈린다.
- 평균 6.3A(진폭) = 4.5A(rms)로 정격 6.1A 이내 → 연속 운전 자체는 문제없다.
  위험한 건 **미도달 상태로 피크를 오래 무는 것**(권선 과열).

사용:
    python3 tools/diag/lateral_probe.py                  # 15분(기본) 관측
    python3 tools/diag/lateral_probe.py --sec 3600 --id 0x143
결과: /var/log/robot_control/lateral_probe.csv (이동 1회 = 1줄, 매 줄 flush)
"""
import argparse
import struct
import time

OUT = '/var/log/robot_control/lateral_probe.csv'

# ★ [2026-09-14] **횡이동 모터를 신품으로 교체했다.** 아래 기존 수치는 전부
#   **고장 직전의 모터** 기준이라 그대로 믿으면 안 된다([[lateral_motor_sizing]]).
#   교체 후 재측정이 이 도구의 현재 목적이다.
#
# X4-36 데이터시트 한계 (0x9C 의 iq 는 **진폭**이다 → rms = iq/√2 로 환산해 비교할 것)
RATED_A_RMS = 6.1        # 연속 정격 상전류
MAX_A_RMS = 21.5         # 최대 상전류 — 드라이버가 여기서 자른다
WARN_TEMP = 70           # °C. 옛 주행모터가 96~100°C 에서 죽었다
STOP_TEMP = 85           # °C. 여기 닿으면 즉시 중단할 것
SQRT2 = 1.41421356


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--channel', default='can2')
    ap.add_argument('--id', default='0x143')
    ap.add_argument('--sec', type=float, default=900.0, help='관측 시간(초)')
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()

    import can
    mid = int(a.id, 0)
    reply_id = mid + 0x100

    f = open(a.out, 'a')
    f.write('# %s 0x%03X 프로브 시작\n' % (time.strftime('%F %T'), mid))
    f.flush()
    bus = can.interface.Bus(channel=a.channel, interface='socketcan')

    samples = []
    moves = []
    warned = set()
    tmax = 0
    moving = False
    tstart = None
    t0 = time.time()
    print('READY (0x%03X, %.0f초)' % (mid, a.sec), flush=True)
    try:
        while time.time() - t0 < a.sec:
            try:
                bus.send(can.Message(arbitration_id=mid,
                                     data=[0x9C, 0, 0, 0, 0, 0, 0, 0],
                                     is_extended_id=False))
            except Exception:
                time.sleep(0.2)          # BUS-OFF 등 — 죽지 말고 재시도
                continue
            d = None
            te = time.time()
            while time.time() - te < 0.02:
                m = bus.recv(timeout=0.02)
                if m and m.arbitration_id == reply_id and m.data[0] == 0x9C:
                    d = m.data
                    break
            if d is None:
                continue
            temp = d[1]
            iq = abs(struct.unpack('<h', bytes(d[2:4]))[0] * 0.01)
            spd = abs(struct.unpack('<h', bytes(d[4:6]))[0])

            # ★ 온도는 **이동 중이 아닐 때도** 본다 — 권선 열은 멈춘 뒤에 하우징으로
            #   퍼지며 더 오른다(2026-09-09 주행모터 사망 때 확인). 식는 것까지 봐야 한다.
            tmax = max(tmax, temp)
            if temp >= STOP_TEMP and 'stop' not in warned:
                warned.add('stop')
                print('\n⛔ %d°C — **즉시 중단**. 권선 과열이다. 식을 때까지 횡이동 금지.'
                      % temp, flush=True)
            elif temp >= WARN_TEMP and 'warn' not in warned:
                warned.add('warn')
                print('\n🔥 %d°C — 경고. 옛 주행모터는 96°C 에서 죽었다. 연속 횡이동 중단 권장.'
                      % temp, flush=True)

            if spd > 5:
                if not moving:
                    moving = True
                    tstart = time.time()
                    samples = []
                samples.append((spd, iq, temp))
            elif moving and time.time() - tstart > 0.3:
                dur = time.time() - tstart
                if len(samples) > 20:     # 짧은 흔들림은 이동으로 치지 않는다
                    sp = [x[0] for x in samples]
                    cu = [x[1] for x in samples]
                    tp = [x[2] for x in samples]
                    avg = sum(sp) / len(sp)
                    mx = max(sp)
                    ripple = 100.0 * (mx - avg) / avg if avg else 0.0
                    c_avg, c_max = sum(cu) / len(cu), max(cu)
                    # 데이터시트는 rms 기준이다. iq(진폭)를 √2로 나눠야 비교가 된다.
                    r_avg, r_max = c_avg / SQRT2, c_max / SQRT2
                    # 최대치 부근에 **얼마나 오래 물려 있었나** — 권선을 태우는 건 이것이다
                    near = sum(1 for c in cu if c / SQRT2 > MAX_A_RMS * 0.9)
                    hold = dur * near / len(cu)
                    line = ('%s | %.2fs | 속도 평균 %.0f 최대 %.0f dps (편차 +%.0f%%)'
                            ' | 전류 평균 %.1f 최대 %.1f A(진폭)'
                            ' = rms %.1f / %.1f A | 정격대비 %.0f%% 최대대비 %.0f%%'
                            ' | 한계부근 %.2fs | 온도 %d→%d°C'
                            % (time.strftime('%T'), dur, avg, mx, ripple,
                               c_avg, c_max, r_avg, r_max,
                               r_avg / RATED_A_RMS * 100, r_max / MAX_A_RMS * 100,
                               hold, tp[0], tp[-1]))
                    f.write(line + '\n')
                    f.flush()
                    print(line, flush=True)
                    if r_max >= MAX_A_RMS * 0.95:
                        print('   ⚠ 드라이버 상한(%.1fA rms)에서 잘리고 있다 — **여유 0%%**.'
                              ' 성공/실패가 잡음으로 갈린다.' % MAX_A_RMS, flush=True)
                    if hold > 1.0:
                        print('   ⚠ 한계 부근을 %.1f초 물었다 — 권선 과열 경로다.'
                              ' (옛 모터가 이렇게 죽었다)' % hold, flush=True)
                    moves.append((dur, r_avg, r_max, hold, max(tp)))
                moving = False
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
        if moves:
            n = len(moves)
            print('\n' + '=' * 72)
            print('[요약] 횡이동 %d회 · 최고온도 %d°C' % (n, tmax))
            print('  평균전류(rms) %.1f A  = 연속정격 %.1f A 의 %.0f%%'
                  % (sum(m[1] for m in moves) / n, RATED_A_RMS,
                     sum(m[1] for m in moves) / n / RATED_A_RMS * 100))
            print('  최대전류(rms) %.1f A  = 드라이버 상한 %.1f A 의 %.0f%%'
                  % (max(m[2] for m in moves), MAX_A_RMS,
                     max(m[2] for m in moves) / MAX_A_RMS * 100))
            print('  한계부근 체류 합계 %.1f초 (1회 최대 %.1f초)'
                  % (sum(m[3] for m in moves), max(m[3] for m in moves)))
            print('-' * 72)
            print('  판정 기준:')
            print('   · 평균이 정격 이내면 **연속 운전 자체는 문제없다**')
            print('   · 위험한 건 **미도달 상태로 피크를 오래 무는 것**(권선 과열)')
            print('   · 최고온도가 70°C 를 넘기 시작하면 연속 횡이동 횟수를 줄일 것')
            print('=' * 72)
        f.close()


if __name__ == '__main__':
    main()
