#!/usr/bin/env python3
"""yaw 를 **절대 위치 목표**로 옮긴다 (0xA4, 단계적 + 되읽기).

## 왜 절대 목표인가

이전에 "속도 제어 + 이동량 적분" 방식 도구를 만들었다가 철회했다 — 적분이 한 번
어긋나면 정지 조건이 무의미해져 **명령보다 크게 움직인다** (건 1.0° 명령에 14.37°,
그 전엔 wrap 결함으로 22.42° 명령이 전류 상한까지 폭주). 여기서는 매 스텝마다
**목표까지 남은 거리**를 다시 읽으므로 그 실패 모드가 없다.

## 부호 (틀리기 쉽다)

    0x92 · 0x60 · 0x61 (생값)   counts 와 **같은 부호** — 건 각도와 함께 증가
    /motor_*_position (토픽)    노드가 뒤집어 발행 (= −counts/728)

따라서 **건 각도를 키우려면 0x92 목표를 키운다.** 토픽 규약을 생값에 적용해
반대로 밀어 기구에 처박은 일이 있다 (2026-10-03).

## 정지마찰

정지 상태에서 출발하면 전류가 올라가며 2초쯤 거의 안 움직이다 풀린다. 전류 상한을
돌파 요구치보다 **위로** 둘 것 — 5~6A 로 잡으면 풀리기 직전에 끊긴다.

⚠ yaw 는 끝 리미트가 없다. 목표를 잘못 주면 기구 끝단으로 간다. 자세 범위
(건 −16.74 ~ +18.80°) 밖이면 거부한다.
"""

import argparse
import socket
import struct
import sys
import time

IFACE, YAW = 'can2', 0x148
CPR, GEAR = 262144, 12.5
CPD = CPR / 360.0
CPG = CPD * GEAR
NOON = 4055
POSE = {1: -16.74, 2: -5.46, 3: 6.81, 4: 18.80}
GUN_MIN, GUN_MAX = -18.5, 20.5          # 자세 범위 + 약간의 여유

s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind((IFACE,))


def flush(t=0.02):
    end = time.time() + t
    while time.time() < end:
        s.settimeout(max(0.003, end - time.time()))
        try:
            s.recv(16)
        except OSError:
            return


def send(d8):
    s.send(struct.pack("IB3x8s", YAW, 8, d8))


def ask(cmd, tries=4):
    for _ in range(tries):
        flush()
        send(bytes([cmd, 0, 0, 0, 0, 0, 0, 0]))
        end = time.time() + 0.09
        while time.time() < end:
            s.settimeout(max(0.003, end - time.time()))
            try:
                raw = s.recv(16)
            except OSError:
                break
            cid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            d = raw[8:16]
            if cid == YAW + 0x100 and d[0] == cmd:
                return d
    return None


def ang():
    d = ask(0x92)
    return None if d is None else struct.unpack('<i', d[4:8])[0] / 100.0


def single():
    d = ask(0x61)
    return None if d is None else struct.unpack('<i', d[4:8])[0] % CPR


def st2():
    d = ask(0x9C)
    if d is None:
        return None
    return (struct.unpack('<h', d[2:4])[0] * 0.01, d[1])


def wrap(x):
    return (x + CPR // 2) % CPR - CPR // 2


def gun_once(sg):
    """단회전 → 12시 기준 건 각도. **시작할 때 한 번만** 쓴다.

    ⚠⚠ 모터 1회전(건 28.8°)마다 접히므로 후보가 여럿이고, 어느 것을 고를지는
    **자세 범위 안에 드는지**로 가린다. 자세 범위가 35.5° 로 한 회전보다 넓어서
    후보 두 개가 동시에 들 수 있는데, 그때 "자세에 가까운 쪽" 같은 규칙을 쓰면
    **읽을 때마다 뒤집힌다.** 2026-10-03 에 그렇게 yaw 를 59스텝 ±2.4° 로 2분간
    왕복시켰다 (전류 5~6A, 40°C).

    그래서 **이동 중에는 다시 부르지 않는다.** 시작 시 한 번 절대각을 잡고,
    이후는 0x92(전원 세션 내 연속값)로 추적한다 — 접히지 않으므로 진동이 불가능하다.
    """
    # 1순위: 호밍과 **같은 논리**로 가린다 — 자세값과 ±tol 안에서 맞추므로
    # 자세에 세워져 있으면 유일하다. 범위 창(39°)은 모터 1회전(28.8°)보다 넓어
    # 후보가 둘 들어오므로 그것만으로는 못 가린다.
    try:
        from rmd_robot_control.axis_config import load_pose_id, identify_pose
        n, det = identify_pose(load_pose_id('yaw'), sg)
        if n is not None:
            return float(det['gun']) + float(det['err_gun']), None
    except Exception:
        pass
    # 2순위: 자세 사이에 있는 경우. 범위 창으로 후보가 하나면 그것을 쓴다.
    g = wrap(sg - NOON) / CPG
    cand = [g - 28.8, g, g + 28.8]
    inside = [x for x in cand if GUN_MIN <= x <= GUN_MAX]
    if len(inside) == 1:
        return inside[0], None
    if not inside:
        return None, f"어느 후보도 자세 범위에 안 든다: {[round(x,2) for x in cand]}"
    return None, (f"자세 사이에 있고 후보가 여럿이다: {[round(x,2) for x in inside]} — "
                  f"자세에 맞추거나 --from-gun 으로 지정하세요")


def goto(deg, spd):
    send(struct.pack("<BBHi", 0xA4, 0x00, int(spd), int(round(deg * 100))))


def halt():
    send(struct.pack("<BB2xi", 0xA2, 100, 0))


def park():
    halt()
    time.sleep(0.2)
    send(bytes([0x78, 0, 0, 0, 0, 0, 0, 0]))
    time.sleep(0.1)
    send(bytes([0x80, 0, 0, 0, 0, 0, 0, 0]))


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--pose', type=int, choices=[1, 2, 3, 4], help='목표 자세')
    g.add_argument('--gun', type=float, help='목표 건 각도 (12시 기준)')
    ap.add_argument('--step', type=float, default=15.0, help='단계 크기 (모터축 도)')
    ap.add_argument('--speed', type=float, default=30.0, help='0xA4 maxSpeed')
    ap.add_argument('--tol', type=float, default=1.5, help='도달 판정 (모터축 도)')
    ap.add_argument('--i-stop', type=float, default=9.0)
    ap.add_argument('--t-stop', type=float, default=45.0)
    # 자세 사이에 있으면 단회전만으로는 **원리적으로** 가릴 수 없다 (자세 범위 35.5°
    # > 모터 1회전 28.8°). 사람이 눈으로 아는 경우를 위한 탈출구다. 입력값을
    # **후보와 대조**해서 건 2° 안에 맞는 후보가 없으면 거부하므로, 오타로 엉뚱한
    # 데로 가지 않는다.
    ap.add_argument('--from-gun', type=float, default=None,
                    help='현재 건 각도를 사람이 지정 (자세 사이일 때)')
    a = ap.parse_args()

    want_gun = POSE[a.pose] if a.pose else a.gun
    if not (GUN_MIN <= want_gun <= GUN_MAX):
        print(f"✗ 목표 건 {want_gun:+.2f}° 가 자세 범위({GUN_MIN}~{GUN_MAX}°) 밖이다")
        return 1

    ask(0x9A)
    sg = single()
    cur_gun, why = gun_once(sg)
    if a.from_gun is not None:
        g = wrap(sg - NOON) / CPG
        cand = [g - 28.8, g, g + 28.8]
        hit = [x for x in cand if abs(x - a.from_gun) <= 2.0]
        if len(hit) != 1:
            print(f"✗ 지정값 건 {a.from_gun:+.2f}° 에 맞는 후보가 "
                  f"{'없다' if not hit else '여럿이다'}: {[round(x, 2) for x in cand]}")
            return 1
        cur_gun = hit[0]
        print(f"  (사람 지정 {a.from_gun:+.2f}° → 후보 {cur_gun:+.2f}° 채택)")
    elif cur_gun is None:
        print(f"✗ 현재 자세를 가릴 수 없다 (단회전 {sg}) — {why}")
        print(f"   자세에 손으로 맞추거나 --from-gun 으로 현재 건 각도를 지정하세요")
        return 1
    print(f"현재 단회전 {sg}  건 {cur_gun:+.2f}°")
    print(f"목표 {'%d번 자세' % a.pose if a.pose else '건 %+.2f°' % want_gun} "
          f"= 건 {want_gun:+.2f}°  →  이동 건 {want_gun - cur_gun:+.2f}°")
    print(f"한계 {a.i_stop:.1f}A / {a.t_stop:.0f}°C")

    print("\n[0] 0x77 해제 — DATA[3]=1 확인까지")
    for i in range(12):
        send(bytes([0x77, 0, 0, 0, 0, 0, 0, 0]))
        time.sleep(0.25)
        b = ask(0x9A)
        if b and b[3] == 1:
            print(f"    확인 ({(i + 1) * 0.25:.2f}초)")
            break
    else:
        print("    ✗ 해제 실패")
        park()
        return 1

    # 0x92 목표를 **한 번** 계산한다. 이후 별칭 해소를 다시 하지 않으므로 진동 불가.
    a0 = ang()
    if a0 is None:
        print("✗ 0x92 읽기 실패")
        park()
        return 1
    tgt_92 = a0 + (want_gun - cur_gun) * GEAR            # 건 + → 0x92 +
    print(f"    0x92 {a0:+.2f}° → 목표 {tgt_92:+.2f}° (모터축 {tgt_92 - a0:+.2f}°)")
    print(f"\n    {'단계':>4} {'남은(모터°)':>12} {'실이동':>9} {'전류':>7} {'온도':>5}")
    stuck = 0
    for step in range(1, 40):
        cur_a = ang()
        if cur_a is None:
            print("    ✗ 읽기 실패")
            break
        remain = tgt_92 - cur_a
        if abs(remain) <= a.tol:
            print(f"    도달 — 남은 모터축 {remain:+.2f}°")
            break
        d_motor = max(-a.step, min(a.step, remain))
        goto(cur_a + d_motor, a.speed)
        t0, pi, pt = time.time(), 0.0, 0
        while time.time() - t0 < 2.0:
            v = st2()
            if v:
                pi, pt = max(pi, abs(v[0])), max(pt, v[1])
                if abs(v[0]) > a.i_stop or v[1] >= a.t_stop:
                    break
            time.sleep(0.03)
        halt()
        time.sleep(0.25)
        na = ang()
        moved = (na - cur_a) if na is not None else 0.0
        print(f"    {step:>4} {remain:>+11.2f}° {moved:>+8.2f}° {pi:>6.2f}A {pt:>4}°C")
        if pi > a.i_stop or pt >= a.t_stop:
            print("    ✗ 전류/온도 한계 — 중단")
            break
        if abs(moved) < 0.5:
            stuck += 1
            if stuck >= 4:
                print("    ✗ 4회 연속 진행 없음 — 중단")
                break
        else:
            stuck = 0

    park()
    sg = single()
    fg, _ = gun_once(sg)
    if fg is None:                       # 가릴 수 없으면 0x92 추적값으로 보고한다
        fg = cur_gun + (ang() - a0) / GEAR
    near = min(POSE.items(), key=lambda kv: abs(kv[1] - fg))
    print(f"\n[끝] 정지 + 0x78 + 0x80")
    print(f"    단회전 {sg}  건 {fg:+.2f}°  →  {near[0]}번 자세에서 {fg - near[1]:+.2f}°")
    print(f"    목표 대비 {fg - want_gun:+.2f}°")
    s.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
