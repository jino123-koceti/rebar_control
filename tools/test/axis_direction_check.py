#!/usr/bin/env python3
"""상부 축의 **원점 방향 부호**를 실측한다. 호밍을 돌리기 전에 이것부터 한다.

## 왜 필요한가

`homing_node` 의 `x_dir`·`y_dir`·`z_dir`·`yaw_dir` 은 "원점 리미트로 가는 속도 부호" 다.
지금 값(-1,-1,+1,-1)은 **아무도 실측한 적이 없는 기본값**이다. 부호가 반대면 호밍은
원점이 아니라 **반대쪽 끝으로 달린다.** 노드가 반대쪽 리미트를 보고 중단하긴 하지만,
그 전에 전 구간을 긁고 지나간다.

그래서 사람이 리모콘으로 축을 움직이고, 이 도구가 "어느 부호의 명령이 어느 리미트를
켰는지" 를 받아 적는다. 모터에 명령을 내리지 않는다 — **보기만 한다.**

## 쓰는 법

    python3 tools/test/axis_direction_check.py

띄워 둔 채로 리모콘으로 한 축씩 천천히 움직인다. 리미트가 켜지면 그 자리에서
결론을 한 줄 찍는다. Ctrl-C 로 끝내면 축별 요약과 **호밍에 넣을 파라미터**를 낸다.

⚠ 브레이크가 풀려 있어야 움직인다:
    ros2 topic pub --once /brake_cmd std_msgs/String "{data: 'release x,y'}"
⚠ Z 는 자중 낙하 위험이 있어 따로 판단한다 (`axes.yaml` never_auto_release).
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float64

# 축 → (명령 토픽 번호, 모터 ID 문자열, 원점 리미트, 반대쪽 리미트)
AXES = {
    'x':   dict(joint=3, motor='0x145', home='x_min', far='x_max'),
    'y':   dict(joint=4, motor='0x146', home='y_min', far='y_max'),
    'z':   dict(joint=5, motor='0x147', home='z_min', far='z_max'),
    'yaw': dict(joint=6, motor='0x148', home='yaw_home', far=None),
}
LIMITS = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', 'yaw_home')
# 명령이 이 시간 안에 있었어야 "그 명령 때문에 닿았다" 고 본다
RECENT_SEC = 1.5


class DirCheck(Node):
    def __init__(self):
        super().__init__('axis_direction_check')
        self.speed = {n: 0.0 for n in AXES}        # 마지막 명령 속도
        self.speed_t = {n: 0.0 for n in AXES}      # 마지막 0 아닌 명령 시각
        self.last_nonzero = {n: 0.0 for n in AXES}
        self.pos = {n: None for n in AXES}
        self.pos_at_cmd = {n: None for n in AXES}  # 명령 시작 시점 위치
        self.limit = {n: None for n in LIMITS}
        self.findings = []                          # (축, 리미트, 명령부호, 위치변화)

        for n, c in AXES.items():
            self.create_subscription(Float32, f"/joint_{c['joint']}/speed",
                                     lambda m, k=n: self._on_speed(k, m), 10)
            # ⚠ Float32 다 (position_control_node 발행 타입). Float64 로 받으면 안 온다
            self.create_subscription(Float32, f"/motor_{c['motor']}_position",
                                     lambda m, k=n: self._on_pos(k, m), 10)
        for n in LIMITS:
            self.create_subscription(Bool, f'/limit_sensors/{n}',
                                     lambda m, k=n: self._on_limit(k, m), 10)
        self.create_timer(1.0, self._heartbeat)
        self._last_beat = ''
        print("■ 관측 시작 — 리모콘으로 한 축씩 천천히 움직이세요. Ctrl-C 로 요약.\n")

    def _on_speed(self, axis, msg):
        v = float(msg.data)
        if abs(v) > 0.01:
            if abs(self.last_nonzero[axis]) <= 0.01:
                self.pos_at_cmd[axis] = self.pos[axis]     # 움직이기 시작한 지점
            self.speed_t[axis] = time.time()
            self.last_nonzero[axis] = v
        self.speed[axis] = v

    def _on_pos(self, axis, msg):
        self.pos[axis] = float(msg.data)

    def _on_limit(self, name, msg):
        was = self.limit[name]
        now = bool(msg.data)
        self.limit[name] = now
        if was is None or was == now or not now:
            return
        self._on_limit_hit(name)

    def _on_limit_hit(self, name):
        """리미트가 꺼짐→켜짐. 방금 어느 축을 어느 부호로 밀고 있었나."""
        axis = next((a for a, c in AXES.items()
                     if name in (c['home'], c['far'])), None)
        if axis is None:
            return
        age = time.time() - self.speed_t[axis]
        sign = self.last_nonzero[axis]
        if age > RECENT_SEC or abs(sign) <= 0.01:
            print(f"  [{name}] 켜짐 — 그런데 최근 {RECENT_SEC:.1f}초 안에 {axis} 축 "
                  f"명령이 없습니다. 손으로 눌렀거나 다른 원인입니다.")
            return
        d = None
        if self.pos[axis] is not None and self.pos_at_cmd[axis] is not None:
            d = self.pos[axis] - self.pos_at_cmd[axis]
        which = '원점' if name == AXES[axis]['home'] else '반대쪽'
        print(f"  ★ [{name}] 켜짐 ({which}) — {axis} 축을 "
              f"{'양수' if sign > 0 else '음수'}({sign:+.1f} dps) 로 밀었습니다"
              + (f", 위치 변화 {d:+.1f}" if d is not None else ""))
        self.findings.append((axis, name, sign, d))

    def _heartbeat(self):
        act = [f"{a}={self.speed[a]:+.0f}" for a in AXES if abs(self.speed[a]) > 0.01]
        on = [n for n in LIMITS if self.limit.get(n)]
        line = f"  움직이는 축: {', '.join(act) if act else '없음'}" \
               f"   켜진 리미트: {', '.join(on) if on else '없음'}"
        if line != self._last_beat:
            print(line)
            self._last_beat = line


def summarize(node):
    print("\n■ 요약")
    if not node.findings:
        print("  리미트에 닿은 기록이 없습니다. 축을 리미트까지 밀어야 판정됩니다.")
        return
    params = {}
    for axis, name, sign, _d in node.findings:
        home = AXES[axis]['home']
        s = 1 if sign > 0 else -1
        if name == home:
            params[axis] = s                    # 원점에 닿은 부호가 곧 dir
        else:
            params.setdefault(axis, -s)         # 반대쪽에 닿았으면 부호를 뒤집는다
        which = '원점' if name == home else '반대쪽'
        print(f"  {axis:4s} {name:9s}({which})  명령부호 {'+' if sign > 0 else '-'}")

    print("\n■ 호밍에 넣을 값")
    for axis in AXES:
        if axis in params:
            src = '원점 직접 확인' if any(n == AXES[axis]['home']
                                    for a, n, _s, _d in node.findings if a == axis) \
                  else '반대쪽에서 추론'
            print(f"  {axis}_dir := {params[axis]:+d}   ({src})")
        else:
            print(f"  {axis}_dir := ?    (아직 안 재봤습니다)")
    print("\n  적용:  ros2 run rmd_robot_control homing_node --ros-args \\")
    print("           " + " ".join(f"-p {a}_dir:={v}" for a, v in params.items()))


def main():
    rclpy.init()
    node = DirCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    summarize(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
