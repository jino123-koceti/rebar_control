#!/usr/bin/env python3
"""리모콘으로 yaw 를 돌리는 동안 **감지 구간을 실측으로 찾는다.**

왜 필요한가 (2026-10-07): yaw 호밍이 반복 실패했다. 사용자 관찰로 원인이
드러났다 — **탐색이 감지 구간 반대쪽으로 가서 3번·4번 자세를 지나 기구부에
박고 스톨한다.** 그 충격이 커플링을 미끄러뜨려 12시 기준값까지 어긋났다.

`axes.yaml` 의 방향 주석은 **옛 기준값**(12시가 감지 구간보다 위) 기준으로
쓰여 있어 지금과 맞지 않는다. 그리고 yaw 가동범위(건 약 35°)가 단회전 주기
(28.8°)보다 **넓어 별칭**이 생기므로, 단회전값 산수로는 방향을 확정할 수 없다
(1번·4번은 특히 못 가려진다).

그래서 **모터를 움직이지 않고** 읽기만 하면서, 사용자가 리모콘으로 돌리는 동안
센서가 켜지는 지점을 잡는다. 얻는 것:
  · 감지 구간의 단회전값 (양쪽 에지, 접근 방향별)
  · 12시 기준값에서 어느 **방향**으로 몇 도인가 → `seek_dir`

사용 (Ctrl-C 로 종료, 그때까지의 결과를 요약한다)
  python3 yaw_edge_find.py
"""

import socket
import struct
import subprocess
import sys
import time

import yaml

FMT = "IB3x8s"
CMD_RAW_MULTI = 0x61
CID = 0x148
AXES_YAML = ('/home/koceti/ros2_ws/src/rebar_control/'
             'src/rebar_base_control/config/axes.yaml')


def noon_cpr_gear():
    d = yaml.safe_load(open(AXES_YAML, encoding='utf-8')) or {}
    st = d.get('stage') or {}
    enc = ((st.get('yaw') or {}).get('encoder') or {})
    return (int(enc['noon_single']), int(enc.get('cpr', 262144)),
            float(st.get('gear', 12.5)))


def read_raw(sock):
    for _ in range(3):
        sock.settimeout(0.01)
        while True:
            try:
                sock.recv(16)
            except OSError:
                break
        sock.send(struct.pack(FMT, CID, 8,
                              bytes([CMD_RAW_MULTI, 0, 0, 0, 0, 0, 0, 0])))
        end = time.time() + 0.05
        while time.time() < end:
            sock.settimeout(max(0.003, end - time.time()))
            try:
                raw = sock.recv(16)
            except OSError:
                break
            rid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            d = raw[8:16]
            if rid == CID + 0x100 and d[0] == CMD_RAW_MULTI:
                return struct.unpack("<i", d[4:8])[0]
    return None


def sensor_reader():
    """`/limit_sensors/yaw_home` 을 백그라운드로 흘려 읽는다.

    `--once` 를 반복 호출하면 매번 노드를 띄워 수백 ms 가 걸려 에지를 놓친다.
    그래서 스트리밍으로 열어 두고 마지막 값을 쓴다.
    """
    cmd = ('source /opt/ros/humble/setup.bash && '
           'source /home/koceti/ros2_ws/install/setup.bash && '
           'export ROS_DOMAIN_ID=33 && '
           'exec ros2 topic echo /limit_sensors/yaw_home --field data')
    return subprocess.Popen(['bash', '-lc', cmd], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, bufsize=1)


def main():
    noon, cpr, gear = noon_cpr_gear()
    cpg = (cpr / 360.0) * gear
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind(("can2",))
    proc = sensor_reader()
    print(f"12시 기준 {noon}  (counts/건1도 = {cpg:.1f})")
    print("리모콘으로 yaw 를 천천히 돌려 주세요. Ctrl-C 로 종료합니다.\n")
    print(f"{'단회전':>8s} {'건각도':>9s}  센서")

    on = None
    last_print = 0.0
    events = []
    prev_single = None
    try:
        while True:
            # 센서 — 쌓인 줄을 다 비우고 **마지막** 값을 쓴다
            import select as _sel
            while _sel.select([proc.stdout], [], [], 0)[0]:
                line = proc.stdout.readline()
                if not line:
                    break
                t = line.strip().lower()
                if t in ('true', 'false'):
                    now_on = (t == 'true')
                    if on is not None and now_on != on:
                        raw = read_raw(sock)
                        if raw is not None:
                            s1 = raw % cpr
                            d = (s1 - noon + cpr // 2) % cpr - cpr // 2
                            way = ('증가' if prev_single is not None
                                   and ((s1 - prev_single + cpr // 2) % cpr
                                        - cpr // 2) > 0 else '감소')
                            events.append((s1, d / cpg, now_on, way))
                            print(f"\n  ★ 센서 {'ON ' if now_on else 'OFF'} "
                                  f"단회전 {s1}  건 {d / cpg:+.2f}°  "
                                  f"({way} 방향 접근)\n", flush=True)
                    on = now_on
            raw = read_raw(sock)
            if raw is not None:
                s1 = raw % cpr
                d = (s1 - noon + cpr // 2) % cpr - cpr // 2
                if time.time() - last_print > 0.3:
                    last_print = time.time()
                    print(f"{s1:>8d} {d / cpg:>+8.2f}°  "
                          f"{'ON' if on else 'off' if on is not None else '?'}",
                          flush=True)
                prev_single = s1
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        sock.close()
    print("\n=== 센서 전이 기록")
    if not events:
        print("  없음 — 감지 구간을 지나가지 않았습니다.")
        return 1
    for s1, gun, now_on, way in events:
        print(f"  {'ON ' if now_on else 'OFF'} 단회전 {s1}  건 {gun:+.2f}°  ({way} 접근)")
    guns = [e[1] for e in events]
    print(f"\n감지 구간은 12시에서 건 {min(guns):+.2f}° ~ {max(guns):+.2f}° 에 있습니다.")
    print("→ 이 부호가 탐색 방향이다. 양수면 counts 증가(seek_dir +1), "
          "음수면 감소(seek_dir -1).")
    return 0


if __name__ == '__main__':
    sys.exit(main())
