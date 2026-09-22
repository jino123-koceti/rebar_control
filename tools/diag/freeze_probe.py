#!/usr/bin/env python3
"""프리즈 프로브 — 하드 리셋에서 **살아남는** 시스템 상태 기록기.

## 왜 필요한가 (2026-08-13 프리즈 2회에서 드러남)
Jetson이 얼어붙어 **리모콘 명령이 안 먹고, 사람이 전원을 내려야 복구**됐다.
그런데 프리즈 직전 2~4분의 기록이 어디에도 남지 않았다 — ROS 로그도 journald도
fsync를 하지 않아 페이지 캐시에 있던 꼬리가 하드 리셋에 통째로 날아갔기 때문이다.
→ **`os.fsync()`** 로 물리 디바이스 쓰기 완료까지 기다리면 마지막 1초까지 남는다.
   1Hz·짧은 레코드라 NVMe에서 비용은 무시할 만하다(100Hz로 하면 안 된다).

## 이 프로브가 실제로 밝혀낸 것 (2차 프리즈, 17:31:01)
리셋 직전 40초가 **전부 정상**이었고 마지막 1초까지 아무 전조가 없었다:
    fsync 5~15ms / lag 24~88ms / dirty·writeback 0 / iowait 0
    memavail 52.7GB / tj 57°C / CPU idle 40%
→ 자원 고갈(스토리지 정체·CPU 기아·메모리·발열)은 **전부 기각**. 점진적 악화 없는
   즉사이므로 하드웨어/드라이버 레벨 정지를 가리킨다. 원인은 아직 미확정.

⚠ 남아 있는 핵심 단서: **두 번 다 횡이동 단계에서만 죽었다.** 전진 주행은 같은
   카메라 구성으로 수 분간 멀쩡했고, 측면 카메라 구독을 45회 반복해도(테스트 A)
   죽지 않았다. 횡이동에만 있는 차이는 0x143 모터 구동이다.

## 무엇을 기록하나
`PSI`(/proc/pressure)가 이 커널엔 비활성(CONFIG_PSI 미설정)이라 못 쓴다. 대신:

  · Dirty/Writeback   — 페이지 캐시 압력. 폭증하면 I/O 정체가 원인
  · MemFree/Available — 회수 압력 (buff/cache는 대부분 회수 가능하니 Available을 볼 것)
  · cpu us/sy/wa/si   — CPU를 삼킨 게 연산인지 I/O 대기인지 인터럽트인지
  · run/tasks         — 실행가능 태스크 수 / 전체 (loadavg의 정체를 분해)
  · gpu               — seg·YOLO·JPEG 인코딩 부하
  · tj                — 온도 (2026-08-13 프리즈에선 58°C로 무관했지만 유지)
  · top3              — CPU 상위 3개 프로세스 (범인 지목용)
  · **fsync_ms**      — 직전 레코드의 fsync 소요(ms). ★ 스토리지 정체의 직접 계측
  · **lag_ms**        — 루프가 예정보다 늦은 시간(ms). CPU 기아의 직접 계측

★ **fsync_ms / lag_ms 를 넣은 이유** — 이 둘이 원인을 갈라준다:
    fsync_ms 폭증 + lag_ms 정상  → **스토리지/writeback 정체**
    lag_ms 폭증 + fsync_ms 정상  → **CPU 스케줄 기아**
    둘 다 폭증                    → 시스템 전반 정지
    둘 다 정상인데 즉사          → 자원 문제 아님 (2차 프리즈가 여기였다)
  ⚠ `lag_ms` 해석 주의: 이 시스템의 평상시 분포가 중앙값 44ms · p99 62ms다.
     50ms대는 전체의 22%로 **일상**이고, 88ms도 0.1%로 드물 뿐 단발이면 의미 없다.
     추세(우상향)가 있어야 신호다.

⚠ 한계: 스토리지가 완전히 멈추면 이 프로브도 fsync에서 블로킹돼 그 이후를 못 남긴다.
   대신 **마지막 레코드의 시각이 곧 정체 시작 시점**이고, 정체가 잠깐 풀리면
   다음 레코드의 fsync_ms에 그 길이가 통째로 찍힌다. 그게 결정적 증거가 된다.

⚠ **loadavg만 보면 안 된다.** 이 시스템은 카메라 5대 + ROS 노드 수십 개라 짧게
   깨어나는 스레드가 많아 loadavg가 25까지 뛰면서도 CPU idle이 41%인 상태가 나온다.
   그래서 분해된 지표를 같이 남긴다.

ROS 의존 없이 /proc 읽기만 한다 — ROS가 죽어도, 죽어가는 중에도 기록된다.

사용:
    python3 tools/diag/freeze_probe.py --out /var/log/robot_control/freeze_probe.csv
분석(재부팅 후):
    python3 tools/diag/freeze_probe.py --analyze /var/log/robot_control/freeze_probe.csv
"""
import argparse
import os
import time

CLK = os.sysconf('SC_CLK_TCK')
# Tegra PMC가 남기는 **마지막 리셋 사유**. 부팅 직후 한 번 읽어 기록한다.
#   SYS_RESET_N = 리셋 핀이 외부에서 당겨짐 (전원 버튼/수동)
#   WATCHDOG 계열 = 워치독이 물어 리셋
# ⚠ 이 레지스터는 **가장 최근 리셋만** 담는다. 워치독이 물어 재부팅이 시작된 뒤
#   사람이 전원을 내리면 그 위를 덮어써 워치독 흔적이 사라진다. 그래서 매 부팅마다
#   여기 기록해두면 "워치독 리셋 → (부팅 중 전원차단) → 수동 리셋"이 **두 줄로** 남는다.
#   (tegra_wdt 드라이버는 sysfs에 bootstatus 를 노출하지 않아 이게 유일한 단서다)
RESET_REASON = '/sys/devices/platform/bus@0/c360000.pmc/reset_reason'
GPU_LOAD = '/sys/devices/platform/bus@0/17000000.gpu/load'   # 0~1000
COLS = ['time', 'up', 'load1', 'run', 'tasks', 'memfree_mb', 'memavail_mb',
        'dirty_mb', 'writeback_mb', 'us', 'sy', 'wa', 'si', 'gpu', 'tj',
        'fsync_ms', 'lag_ms', 'top3']


def _read(path, default=''):
    """sysfs/procfs 안전 읽기.

    ⚠ 바이너리로 읽고 **None까지 처리한다** — 일부 thermal zone은 읽기가 블로킹
       상태가 되어 `read()`가 None을 돌려준다. 텍스트 모드로 열면 그게 디코더
       안에서 TypeError로 터지는데 OSError가 아니라 안 잡힌다(실측 2026-08-13).
    """
    try:
        with open(path, 'rb') as f:
            return (f.read() or b'').decode('utf-8', 'ignore')
    except (OSError, ValueError, TypeError):
        return default


def cpu_times():
    """/proc/stat 첫 줄 → (total, user, sys, iowait, softirq) jiffies."""
    parts = _read('/proc/stat').split('\n', 1)[0].split()[1:]
    v = [int(x) for x in parts[:8]]
    return sum(v), v[0] + v[1], v[2], v[4], v[6]


def meminfo():
    m = {}
    for line in _read('/proc/meminfo').splitlines():
        k, _, rest = line.partition(':')
        if k in ('MemFree', 'MemAvailable', 'Dirty', 'Writeback'):
            m[k] = int(rest.split()[0]) // 1024       # MB
    return m


def tj_temp():
    """가장 뜨거운 thermal zone (°C). tj가 없으면 최대값."""
    best = 0.0
    for z in range(24):
        t = _read(f'/sys/class/thermal/thermal_zone{z}/temp')
        if t.strip().lstrip('-').isdigit():
            best = max(best, int(t) / 1000.0)
    return round(best, 1)


def proc_cpu():
    """{pid: (name, utime+stime jiffies)} — /proc 스캔."""
    out = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        s = _read(f'/proc/{pid}/stat')
        if not s:
            continue
        try:
            name = s[s.index('(') + 1:s.rindex(')')]
            f = s[s.rindex(')') + 2:].split()
            out[pid] = (name, int(f[11]) + int(f[12]))   # utime + stime
        except (ValueError, IndexError):
            continue
    return out


def top3(prev, cur, dt):
    """직전 대비 CPU를 가장 많이 쓴 3개 → 'name:pct' 문자열."""
    d = []
    for pid, (name, t) in cur.items():
        if pid in prev:
            used = (t - prev[pid][1]) / CLK / dt * 100.0
            if used > 1.0:
                d.append((used, name))
    d.sort(reverse=True)
    return ' '.join(f'{n[:12]}:{p:.0f}' for p, n in d[:3])


def run(out_path, hz):
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    new = not os.path.exists(out_path) or os.path.getsize(out_path) == 0
    f = open(out_path, 'a')
    if new:
        f.write(','.join(COLS) + '\n')
    # ★ 부팅 마커 — 이 부팅이 무엇 때문에 시작됐는지를 맨 앞에 못박아둔다
    reason = _read(RESET_REASON).strip() or 'unknown'
    up0 = _read('/proc/uptime').split()
    mark = [time.strftime('%Y-%m-%d %H:%M:%S'), f'{float(up0[0]):.0f}' if up0 else '-']
    mark += [''] * (len(COLS) - 3)
    mark.append(f'BOOT reset_reason={reason}')
    f.write(','.join(mark) + '\n')
    f.flush()
    os.fsync(f.fileno())
    print(f'  부팅 사유: {reason}', flush=True)
    period = 1.0 / hz
    fsync_ms = lag_ms = 0.0
    pt = cpu_times()
    pp = proc_cpu()
    ptime = time.time()
    print(f'freeze_probe → {out_path} @ {hz}Hz (fsync 매 레코드)', flush=True)
    while True:
        time.sleep(period)
        now = time.time()
        dt = max(1e-3, now - ptime)
        ct = cpu_times()
        cp = proc_cpu()
        dtot = max(1, ct[0] - pt[0])
        us, sy, wa, si = ((ct[i] - pt[i]) * 100.0 / dtot for i in range(1, 5))
        m = meminfo()
        la = _read('/proc/loadavg').split()
        # /proc/loadavg 4번째 필드는 '실행가능/전체' (블로킹 아님).
        # run이 순간 40+까지 튀는 게 loadavg 22 + CPU idle 41%의 정체다.
        run_blk = la[3].split('/') if len(la) > 3 else ('0', '0')
        gpu = _read(GPU_LOAD).strip() or '-'
        row = [time.strftime('%Y-%m-%d %H:%M:%S'),
               f"{float(_read('/proc/uptime').split()[0]):.0f}",
               la[0] if la else '-', run_blk[0], run_blk[1],
               m.get('MemFree', 0), m.get('MemAvailable', 0),
               m.get('Dirty', 0), m.get('Writeback', 0),
               f'{us:.0f}', f'{sy:.0f}', f'{wa:.0f}', f'{si:.0f}',
               gpu, tj_temp(),
               # 직전 레코드의 fsync 소요·루프 지연을 여기 싣는다(아래 설명).
               f'{fsync_ms:.0f}', f'{lag_ms:.0f}', top3(pp, cp, dt)]
        f.write(','.join(str(x) for x in row) + '\n')
        f.flush()
        t_fs = time.time()
        os.fsync(f.fileno())      # ★ 이 한 줄이 하드 리셋에서 살아남게 한다
        fsync_ms = (time.time() - t_fs) * 1000.0
        lag_ms = max(0.0, (now - ptime - period) * 1000.0)
        pt, pp, ptime = ct, cp, now


def analyze(path, tail=40):
    """재부팅 후: uptime이 되감긴 지점 = 리셋. 그 직전 구간을 보여준다."""
    rows = [l.rstrip('\n').split(',') for l in open(path) if l.strip()]
    if len(rows) < 2:
        print('데이터 없음'); return
    hdr, data = rows[0], rows[1:]
    iu = hdr.index('up')
    cuts = [i for i in range(1, len(data))
            if float(data[i][iu] or 0) < float(data[i - 1][iu] or 0)]
    if not cuts:
        print('리셋 흔적 없음(uptime 되감김 없음). 마지막 구간만 표시.')
        cuts = [len(data)]
    boots = [r for r in data if r[-1].startswith('BOOT')]
    if boots:
        print('=== 부팅 이력 (리셋 사유) ===')
        for b in boots[-8:]:
            print(f'  {b[0]}  {b[-1]}')
    for c in cuts[-3:]:
        print(f"\n{'='*100}\n리셋 직전 {tail}초 (마지막 기록 {data[c-1][0]}, "
              f"up={data[c-1][iu]}s)\n{'='*100}")
        print(' | '.join(f'{h:>11}' for h in hdr))
        for r in data[max(0, c - tail):c]:
            print(' | '.join(f'{v:>11}' for v in r))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/var/log/robot_control/freeze_probe.csv')
    ap.add_argument('--hz', type=float, default=1.0)
    ap.add_argument('--analyze', metavar='CSV')
    a = ap.parse_args()
    if a.analyze:
        analyze(a.analyze)
    else:
        try:
            run(a.out, a.hz)
        except KeyboardInterrupt:
            pass
