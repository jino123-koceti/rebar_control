#!/usr/bin/env python3
"""GMSL 4대 동시 스트리밍 감시 + **자원 여유 실측**.

## 왜
두 가지를 같이 재야 판단이 선다.

1. **4대가 버티는가** — 지금까지 실패한 두 번은 전부 *돌아가는 중에 추가로 열기*였고,
   직접 원인은 대수가 아니라 자식의 비정상 종료가 Argus 소켓을 끊은 것이었다:
       (Argus) Error EndOfFile: reading socket  →  다른 카메라 SIGSEGV(exit -11)
   런치 시점에 한꺼번에 여는 경로는 2026-09-14 정지 5분 시험을 통과했다.
   **남은 미검증은 "움직이면서"** — 과거 사고는 전부 횡이동 중 전류 14A 상황에서 났다.

2. **카메라 4대 + 하위제어 + 자율결속을 같이 돌릴 CPU 여유가 있는가** —
   ⚠ `load average` 로는 답이 안 나온다. D상태(I/O 대기)를 포함해 CPU 압박을
   과대평가한다(2026-09-14 실측: load 26 인데 CPU idle 35%). **진짜 지표는 idle** 이고,
   여유가 없을 때 어디를 줄일지 정하려면 **서브시스템별 점유**가 필요하다.
   그래서 /proc/stat + /proc/<pid>/stat 을 직접 델타로 읽어 코어 수로 환산한다.

## 사용
    # 서비스에 use_zedxone:=true use_zedxone_left:=true 를 넣고 재시작한 뒤
    python3 tools/vision_test/gmsl4_watch.py --min 10          # 운용 조건(compressed)
    python3 tools/vision_test/gmsl4_watch.py --raw --min 5     # 최악조건 스트레스

    # ★ 주행·횡이동을 시키면서 같이 돌릴 것. 그게 남은 검증이다.

## 보는 것
  · 4개 토픽 각각의 실제 Hz (목표 대비)
  · CPU 사용 코어수 / 12, 서브시스템별(카메라·하위제어·상위비전) 내역
  · GPU 부하, CPU/GPU 온도, CPU 클럭(전력모드 상한에 닿았는지)
  · CORRUPTED FRAME / Argus EndOfFile / process has died 누적
성공 판정: 4개 다 프레임이 오고, died 0건, 몇 분간 Hz가 유지되고,
          피크에서도 CPU 여유가 남는다.
"""
import argparse, os, re, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

# ⚠ **compressed 를 구독한다** (2026-09-14 실측으로 수정).
#   raw 로 4개를 동시 구독하면 ZED X One HD1200 이 **9.2 MB/프레임**이라 모노 2대만
#   수십 MB/s 가 이 프로세스로 쏟아진다 → **측정 자체가 카메라를 굶긴다**
#   (첫 시험에서 스테레오가 1.1~1.6Hz 로 찍혔다. 목표 6Hz).
#   deck_edge 가 실제로 구독하는 것도 compressed 다. 그게 운용 조건이다.
# ⚠ 그리고 모노 2대는 평소 **구독자가 0이라 발행하지 않는다**(측면 판정 때만 열린다).
#   즉 이 도구로 재는 건 언제나 **실제보다 무거운 조건**이다 — 여기서 버티면 실제로도 버틴다.
TOPICS = {
    '전방 zedxmini2': '/zedxmini2/zed_node/rgb/color/rect/image/compressed',
    '후방 zedxmini1': '/zedxmini1/zed_node/rgb/color/rect/image/compressed',
    '우측 zedxone':   '/zedxone/zed_node/rgb/color/rect/image/compressed',
    '좌측 zedxone_left': '/zedxone_left/zed_node/rgb/color/rect/image/compressed',
}
PATTERNS = {'CORRUPTED FRAME': r'CORRUPTED FRAME',
            'Argus EndOfFile': r'Argus\).*EndOfFile',
            'process has died': r'process has died'}

# 서브시스템 분류 — cmdline 부분일치. **위에서부터** 먼저 맞는 것으로 친다.
#   zed 4대는 component_container_isolated, Orbbec 은 component_container 로 뜬다.
BUCKETS = [
    ('카메라', ('nvargus-daemon', 'ZEDX_Daemon', 'component_container',
                'camera_container', 'zed_node')),
    ('하위제어', ('can_sender', 'can_parser', 'drive_controller', 'joint_controller',
                  'navigator', 'ezi_io', 'authority', 'homing', 'position_control',
                  'pololu')),
    ('상위비전', ('deck_edge', 'rebar_drive', 'tying_orchestrator', 'rebar_detection',
                  'orbbec_detector', 'auto_tying', 'rebar_grid', 'frame_stream')),
]
HZ = os.sysconf('SC_CLK_TCK')
NCPU = os.cpu_count() or 1


def newest_log():
    d = '/var/log/robot_control'
    fs = [os.path.join(d, f) for f in os.listdir(d) if f.startswith('control_')]
    return max(fs, key=os.path.getmtime) if fs else None


def count(log):
    if not log:
        return {k: 0 for k in PATTERNS}
    try:
        # ⚠ 로그에 바이너리 바이트가 섞여 있다(ZED 가 raw 를 뱉는 줄이 있음).
        #   errors='replace' 없이 읽으면 여기서 죽는다.
        txt = open(log, 'r', errors='replace').read()
    except Exception:
        return {k: 0 for k in PATTERNS}
    return {k: len(re.findall(v, txt)) for k, v in PATTERNS.items()}


def read_int(path, default=None):
    try:
        with open(path) as f:
            return int(f.read().strip())
    except Exception:
        return default


def cpu_totals():
    """/proc/stat 첫 줄 → (전체 tick, idle tick). idle 에 iowait 포함."""
    with open('/proc/stat') as f:
        v = [int(x) for x in f.readline().split()[1:]]
    return sum(v), v[3] + (v[4] if len(v) > 4 else 0)


def proc_ticks():
    """살아있는 프로세스별 (버킷, 누적 tick). 죽은 PID 는 조용히 건너뛴다."""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
            # cmdline 이 비면 커널 스레드다. 건너뛰면 안 된다 —
            # 주파수 거버너(sugov)·irq·ksoftirqd 가 수 코어를 먹는다(2026-09-15 실측
            # 에서 총계 6.6 중 2.2 코어가 여기였다). 버킷 합이 총계와 안 맞으면
            # 어디가 먹는지 못 짚는다.
            with open(f'/proc/{pid}/stat') as f:
                st = f.read()
            # comm 에 공백/괄호가 들어갈 수 있어 마지막 ')' 뒤부터 자른다.
            fields = st[st.rindex(')') + 2:].split()
            ticks = int(fields[11]) + int(fields[12])   # utime + stime
        except Exception:
            continue
        bucket = '커널' if not cmd else '기타'
        for name, pats in BUCKETS:
            if cmd and any(p in cmd for p in pats):
                bucket = name
                break
        out[pid] = (bucket, ticks)
    return out


def bucket_cores(prev, cur, elapsed):
    """버킷별 사용 코어수. 새로 뜬 PID 는 이번 구간 값이 없으니 제외한다."""
    acc = {name: 0.0 for name, _ in BUCKETS}
    acc['기타'] = 0.0
    acc['커널'] = 0.0
    for pid, (bucket, ticks) in cur.items():
        if pid not in prev:
            continue
        d = ticks - prev[pid][1]
        if d > 0:
            acc[bucket] += d / HZ / elapsed
    return acc


def temps():
    t = {}
    base = '/sys/devices/virtual/thermal'
    try:
        zones = sorted(os.listdir(base))
    except Exception:
        return t
    for z in zones:
        if not z.startswith('thermal_zone'):
            continue
        try:
            with open(f'{base}/{z}/type') as f:
                name = f.read().strip()
        except Exception:
            continue
        v = read_int(f'{base}/{z}/temp')
        if v is not None:
            t[name] = v / 1000.0
    return t


def gpu_pct():
    # tegra 는 per-mille(0~1000) 로 낸다.
    v = read_int('/sys/devices/platform/gpu.0/load')
    if v is None:
        v = read_int('/sys/devices/gpu.0/load')
    return v / 10.0 if v is not None else None


def cpu_mhz():
    cur = read_int('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq')
    mx = read_int('/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq')
    return (cur / 1000.0 if cur else None, mx / 1000.0 if mx else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--min', type=float, default=5.0, help='감시 시간 (분)')
    ap.add_argument('--every', type=float, default=30.0, help='보고 주기 (초)')
    ap.add_argument('--raw', action='store_true',
                    help='raw Image 로 구독 (최악조건 스트레스. 평소엔 쓰지 말 것)')
    a = ap.parse_args()

    rclpy.init()
    n = Node('gmsl4_watch')
    cnt = {k: 0 for k in TOPICS}

    def mk(k):
        def cb(_m):
            cnt[k] += 1
        return cb
    typ = Image if a.raw else CompressedImage
    for k, t in TOPICS.items():
        topic = t[:-len('/compressed')] if (a.raw and t.endswith('/compressed')) else t
        n.create_subscription(typ, topic, mk(k), qos_profile_sensor_data)

    log = newest_log()
    base = count(log)
    _, mx_mhz = cpu_mhz()
    print(f'▶ GMSL 4대 감시 {a.min:.0f}분 · {NCPU}코어 '
          + (f'· CPU 상한 {mx_mhz:.0f}MHz' if mx_mhz else '') + f' · 로그 {log}')
    print('  ★ 주행·횡이동을 시키면서 볼 것. 정지 상태는 이미 통과했다(2026-09-14).')
    print('  카메라가 죽으면 그 방향 주행이 차단된다.\n')

    # 피크 추적 — 판단은 평균이 아니라 피크로 한다.
    peak = {'cpu': 0.0, 'gpu': 0.0, 'tj': 0.0}
    peak_buckets = {}
    min_hz = {k: None for k in TOPICS}

    prev_proc = proc_ticks()
    prev_tot, prev_idle = cpu_totals()
    t_end = time.time() + a.min * 60
    try:
        while time.time() < t_end:
            for k in cnt:
                cnt[k] = 0
            t0 = time.time()
            while time.time() - t0 < a.every and time.time() < t_end:
                rclpy.spin_once(n, timeout_sec=0.1)
            el = time.time() - t0

            parts = []
            for k in TOPICS:
                hz = cnt[k] / el if el else 0
                if min_hz[k] is None or hz < min_hz[k]:
                    min_hz[k] = hz
                parts.append(f'{k} {hz:5.2f}Hz' + ('' if hz > 0.5 else ' ❌'))

            tot, idle = cpu_totals()
            dtot, didle = tot - prev_tot, idle - prev_idle
            used = NCPU * (1 - didle / dtot) if dtot > 0 else 0.0
            prev_tot, prev_idle = tot, idle

            cur_proc = proc_ticks()
            bc = bucket_cores(prev_proc, cur_proc, el)
            prev_proc = cur_proc

            g = gpu_pct()
            tp = temps()
            cur_mhz, _ = cpu_mhz()

            peak['cpu'] = max(peak['cpu'], used)
            if g is not None:
                peak['gpu'] = max(peak['gpu'], g)
            if 'tj-thermal' in tp:
                peak['tj'] = max(peak['tj'], tp['tj-thermal'])
            for name, v in bc.items():
                peak_buckets[name] = max(peak_buckets.get(name, 0.0), v)

            now = count(log)
            delta = {k: now[k] - base[k] for k in PATTERNS}
            bad = ' '.join(f'{k}+{v}' for k, v in delta.items() if v)
            ts = time.strftime('%H:%M:%S')
            print(f'  {ts}  ' + ' | '.join(parts) + ('   ⚠ ' + bad if bad else ''))
            print(f'           CPU {used:4.1f}/{NCPU}코어 (여유 {NCPU - used:4.1f})'
                  f'  [카메라 {bc["카메라"]:.1f} · 하위 {bc["하위제어"]:.1f}'
                  f' · 비전 {bc["상위비전"]:.1f} · 커널 {bc["커널"]:.1f}'
                  f' · 기타 {bc["기타"]:.1f}]'
                  + (f'  GPU {g:4.1f}%' if g is not None else '')
                  + (f'  CPU {tp.get("cpu-thermal", 0):.0f}°C'
                     f'/tj {tp.get("tj-thermal", 0):.0f}°C' if tp else '')
                  + (f'  {cur_mhz:.0f}MHz' if cur_mhz else ''))

            if delta['process has died']:
                print('\n⛔ 카메라 노드가 죽었다 → **4대 동시는 불가**. 로그의 Argus 줄을 볼 것:')
                print(f'   grep -anE "Argus|died" {log} | tail -20')
                break
            base = now
    except KeyboardInterrupt:
        print('\n(중단)')

    print('\n── 요약 (판단은 평균이 아니라 피크로) ──')
    print(f'  CPU 피크   {peak["cpu"]:.1f}/{NCPU}코어  → 여유 {NCPU - peak["cpu"]:.1f}코어 '
          f'({100 * (1 - peak["cpu"] / NCPU):.0f}%)')
    if peak_buckets:
        print('  버킷 피크  ' + ' · '.join(f'{k} {v:.1f}' for k, v in peak_buckets.items()))
    print(f'  GPU 피크   {peak["gpu"]:.0f}%      tj 피크 {peak["tj"]:.0f}°C')
    print('  최저 Hz    ' + ' · '.join(
        f'{k} {v:.2f}' for k, v in min_hz.items() if v is not None))
    print('  ⚠ 모노 2대는 실제 운용에선 측면 판정 때만 구독자가 붙는다 → 이 수치는 상한이다.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
