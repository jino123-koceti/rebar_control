#!/usr/bin/env python3
"""
횡이동 2축 (0x143 #1, 0x144 #2) 코어 로직 — ROS 의존 없음

2026-09-08 실물 검증된 규약:
  * 부호: 0x143 = +1, 0x144 = -1.
    두 모터가 마주보게 설치되어 있어 동일 부호를 주면 기구가 서로 반대로 간다.
    반대 부호를 줘야 같은 방향. **+방향 = 좌측 횡이동.**
    (두 축은 기계적으로 연결돼 있지 않아 반대로 가도 전류가 튀지 않는다.
     방향은 전류가 아니라 육안으로 확인할 것.)
  * 1회전 = 리드스크류 1바퀴 = 50 mm (7.2 °/mm)
  * 엔코더 1회전 = 262144 counts (18bit)
  * 홈(12시) 기준은 0x61: `0x61 mod 262144` 가 단회전 절대 위치로 전원 재투입 후에도
    유지된다 (검증 완료). 0x60/0x92 는 전원 인가 시점 기준이라 홈 기준으로 쓰면 안 된다.
  * 이 펌웨어는 0x90(단회전 엔코더)과 0x30/0x31(PID)을 지원하지 않는다.

주의: 각도 부호는 CAN raw 규약(0x92 읽기값과 동일)이다.
      position_control_node 의 /joint_N/position 은 부호를 반전해서 보내므로 규약이 다르다.
"""

import math
import socket
import struct
import time

CAN_FRAME_FMT = "IB3x8s"

ENC_CPR = 262144          # 엔코더 1회전 counts (18bit)
# ⚠ [2026-10-04] 3차년도 장비는 **1회전 = 100mm** 다 (2차년도는 50mm).
#   둘 다 표시용으로만 쓰인다 — 실제 이동량은 기구가 정하고, `/lateral/step` 은
#   회전 수를 받는다. 그래도 로그가 틀리면 사람이 거리를 잘못 센다.
DEG_PER_MM = 3.6          # 360° = 100 mm
MM_PER_TURN = 100.0

CMD_STATUS1 = 0x9A
CMD_STATUS2 = 0x9C
CMD_MULTITURN_ANGLE = 0x92
CMD_ENC_ORIGINAL = 0x61
CMD_POSITION = 0xA4
CMD_SPEED = 0xA2
CMD_STOP = 0x81
CMD_SHUTDOWN = 0x80

DEFAULT_SPEED_DPS = 50
DEFAULT_TOLERANCE_DEG = 1.0

# 0xA2/0xA4 DATA[1] = maxTorque. 0 으로 두면 힘 제어가 활성화되지 않는다
# (프로토콜 2.20.1 주석 3). 구동륜은 2026-09-08 에 이걸 0 으로 보내던 것이
# "감속이 관성처럼 느리다" 의 원인으로 밝혀져 0x64(100) 로 고쳤는데,
# 횡이동은 같은 함정이 남아 있었다 (2026-09-15 발견).
# 의미가 "제한 없음(0)" 인지 "힘 제어 비활성(0)" 인지 확정되지 않았으므로
# 파라미터로 열어두고 실측으로 정한다.
DEFAULT_MAX_TORQUE = 255

# ── 전류/열 보호 ────────────────────────────────────────────────────────────
# RMD-X4-36 (RMD-X4-P36-36-C) 데이터시트: 정격 상전류 6.1 A(rms), 피크 21.5 A(rms)
#
# 2차년도 장비에서 횡이동 중 모터를 여러 개 소손시켰다. 당시 소프트웨어 제한이
# 30 A 로 **피크 정격 21.5 A 보다도 높았고**, 정격 초과 '지속'을 감시하지 않았다.
# 모터를 태우는 것은 순간 피크가 아니라 정격을 넘긴 상태의 지속(I²t)이다.
# 부하(장비 리프팅) 테스트 설정 — 2026-09-08
# 횡이동은 장비를 들어올리므로 정격 6.1A 를 넘기는 것이 '정상 동작'이다.
# 따라서 정격 초과 자체로는 멈추지 않고, 아래 세 가지로 보호한다:
#   (1) 순간 18A 초과   → 즉시 0x80 차단 (유일한 하드 차단)
#   (2) I²t 열 적산     → 12시로 후퇴 후 정지
#   (3) 시간 제한/스톨/온도 → 12시로 후퇴 후 정지
# 후퇴가 즉시 차단보다 안전하다: 0x80 은 유지 토크를 없애 들린 장비를 떨어뜨린다.
I_RATED_A = 6.1           # 데이터시트 정격 상전류 (I²t 기준선)
I_HARD_A = 18.0           # 즉시 0x80 차단. 데이터시트 피크 21.5A(rms) 아래
I_SUSTAIN_ENABLED = False # 정격 초과 '지속' 단독 차단 — 리프팅에서는 비활성
I_SUSTAIN_T = 1.0
I2T_BUDGET = 600.0        # (I²-I_rated²) 적산 허용치 [A²·s]
                          #   10A→9.6초, 12A→5.6초, 15A→3.2초, 18A→2.0초 후 후퇴
I2T_COOL_TAU = 20.0       # 정격 이하일 때 적산값 감쇠 시정수 (초)
TEMP_STOP_C = 60          # 모터 보고 온도 상한 (부하 테스트라 보수적으로)
STALL_SPEED_DPS = 3       # 이 속도 미만인데
STALL_I_A = 3.0           #   전류가 이 이상이고
STALL_T = 0.7             #   이 시간 지속되면 스톨로 판정 (초)

DEFAULT_MOVE_TIMEOUT = 8.0    # 이동 제한의 '하한'. 실제 제한은 속도에서 계산한다.
                              # 200dps 1회전=1.8s 이므로 하한 8s 가 그대로 적용되고,
                              # 50dps 1회전=7.2s 같은 저속에서만 자동으로 늘어난다.
                              # 고정 8s 로 두면 저속에서 정상 동작이 헛트립한다.
RETREAT_SPEED_DPS = 30        # 후퇴(12시 복귀) 속도
RETREAT_TIMEOUT = 12.0        # 후퇴 자체의 제한. 넘기면 0x80
RETREAT_SETTLE_T = 1.5        # 후퇴 시작 전 전류가 정격 아래로 내려오기를 기다리는 상한

SEV_HARD = "hard"    # 즉시 0x80
SEV_SOFT = "soft"    # 12시로 후퇴


class Protection:
    """축 1개의 전류·열·스톨 감시 상태.

    핵심은 순간 피크가 아니라 **정격 초과의 지속**을 잡는 것이다.
    판정되면 호출자가 즉시 0x80(SHUTDOWN)을 보내야 한다 —
    0xA2 speed=0 이나 0x81(STOP)은 모터를 여자 상태로 남긴다 (2026-09-08 실측 확인).
    """

    def __init__(self, motor_id, name="", i_hard=None):
        self.motor_id = motor_id
        self.name = name
        # 축별 하드 차단 임계 [A]. None 이면 모듈 기본값 I_HARD_A 를 쓴다.
        # 횡이동은 장비를 들어올려 기동 서지가 크므로 노드에서 더 높게 준다.
        # 주행(position_control_node)은 기본값 그대로 두어야 한다 — 부하가 전혀 다르다.
        self.i_hard = I_HARD_A if i_hard is None else float(i_hard)
        self.reset()

    def reset(self):
        self.i2t = 0.0
        self.over_since = None
        self.stall_since = None
        self.peak_a = 0.0
        self.peak_temp = 0
        self._last_t = None

    def update(self, current_a, speed_dps, temp_c, now):
        """감시 1스텝. 차단해야 하면 사유 문자열, 아니면 None."""
        i = abs(current_a)
        self.peak_a = max(self.peak_a, i)
        self.peak_temp = max(self.peak_temp, temp_c)

        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now

        # (1) 하드 — 즉시 0x80. 이것만 유일하게 즉시 차단한다.
        if i > self.i_hard:
            return f"순간 과전류 {i:.2f}A > {self.i_hard}A", SEV_HARD

        # (2) 온도 — 후퇴
        if temp_c >= TEMP_STOP_C:
            return f"온도 {temp_c}°C >= {TEMP_STOP_C}°C", SEV_SOFT

        # (3) 정격 초과 — 리프팅에서는 정상이므로 I²t 로만 판단
        if i > I_RATED_A:
            if self.over_since is None:
                self.over_since = now
            elif I_SUSTAIN_ENABLED and now - self.over_since > I_SUSTAIN_T:
                return (f"정격 초과 지속 {i:.2f}A > {I_RATED_A}A "
                        f"({now - self.over_since:.1f}s)"), SEV_SOFT
            self.i2t += (i * i - I_RATED_A * I_RATED_A) * dt
            if self.i2t > I2T_BUDGET:
                return (f"열 적산 초과 I²t={self.i2t:.0f} > {I2T_BUDGET:.0f} A²·s "
                        f"(최대 {self.peak_a:.1f}A)"), SEV_SOFT
        else:
            self.over_since = None
            if dt > 0 and self.i2t > 0:
                self.i2t *= math.exp(-dt / I2T_COOL_TAU)

        # (4) 스톨 — 힘은 쓰는데 안 움직임 → 후퇴
        if abs(speed_dps) < STALL_SPEED_DPS and i > STALL_I_A:
            if self.stall_since is None:
                self.stall_since = now
            elif now - self.stall_since > STALL_T:
                return (f"스톨 감지 (속도 {speed_dps} dps, "
                        f"전류 {i:.2f}A)"), SEV_SOFT
        else:
            self.stall_since = None

        return None, None

    def summary(self):
        return (f"0x{self.motor_id:03X} 최대 {self.peak_a:.2f}A, "
                f"I²t {self.i2t:.0f} A²·s, 최고 {self.peak_temp}°C")


class LateralAxis:
    """횡이동 축 하나의 설정"""

    def __init__(self, motor_id, sign, home_enc_single, name=""):
        self.motor_id = motor_id
        self.sign = sign                      # +1 / -1
        self.home_enc_single = home_enc_single  # 0x61 mod ENC_CPR, 12시 위치
        self.name = name or f"0x{motor_id:03X}"


class LateralAxes:
    """횡이동 2축 동기 제어 (CAN 직접)"""

    def __init__(self, axes, interface="can2", speed_dps=DEFAULT_SPEED_DPS,
                 tolerance_deg=DEFAULT_TOLERANCE_DEG, logger=None, i_hard_a=None,
                 max_torque=None):
        self.axes = list(axes)
        self.speed_dps = speed_dps
        self.tolerance_deg = tolerance_deg
        self.max_torque = (DEFAULT_MAX_TORQUE if max_torque is None
                           else max(0, min(255, int(max_torque))))
        self.i_hard_a = I_HARD_A if i_hard_a is None else float(i_hard_a)
        # 직전 이동의 출발점(12시). 후퇴는 여기로 되돌아간다 — _retreat 주석 참조.
        self._move_home = {}
        self.prot = {ax.motor_id: Protection(ax.motor_id, ax.name, i_hard=self.i_hard_a)
                     for ax in self.axes}
        # 진단용 샘플 훅. 설정하면 감시 루프의 모든 샘플이 전달된다.
        #   recorder(motor_id, t_since_start, current_a, speed_dps, temp_c, i2t, angle_raw)
        self.recorder = None
        self._log = logger
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.settimeout(0.3)

    # ---- 로깅 -------------------------------------------------------------
    def _info(self, msg):
        if self._log:
            self._log.info(msg)
        else:
            print(msg)

    def _warn(self, msg):
        if self._log:
            self._log.warning(msg)
        else:
            print(msg)

    # ---- CAN 기본 ---------------------------------------------------------
    def _drain(self):
        """수신 버퍼 비우기 — 논블로킹.

        settimeout(0.02) 로 비우면 호출마다 20ms 를 소모해 감시 주기가
        축당 15Hz 까지 떨어진다. 부하 상태에서 스톨을 빨리 잡으려면
        논블로킹이어야 한다 (2026-09-08: 31Hz → 250Hz 개선).
        """
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recv(16)
        except (BlockingIOError, OSError):
            pass
        finally:
            self.sock.setblocking(True)
            self.sock.settimeout(0.3)

    def _send(self, motor_id, data8):
        self.sock.send(struct.pack(CAN_FRAME_FMT, motor_id, 8, bytes(data8)))

    def _ask(self, motor_id, cmd, window=0.3):
        self._drain()
        self.sock.settimeout(window)
        self._send(motor_id, [cmd, 0, 0, 0, 0, 0, 0, 0])
        deadline = time.time() + window
        while time.time() < deadline:
            try:
                raw = self.sock.recv(16)
            except OSError:
                break
            rid = struct.unpack("I", raw[:4])[0] & 0x1FFFFFFF
            data = raw[8:16]
            if rid == motor_id + 0x100 and data[0] == cmd:
                return data
        return None

    @staticmethod
    def _i32(data):
        return int.from_bytes(data[4:8], "little", signed=True)

    # ---- 읽기 -------------------------------------------------------------
    def angle(self, motor_id):
        """0x92 멀티턴 각도 (deg, 전원 인가 기준)"""
        d = self._ask(motor_id, CMD_MULTITURN_ANGLE)
        return None if d is None else self._i32(d) / 100.0

    def enc_single(self, motor_id):
        """0x61 mod ENC_CPR — 단회전 절대 위치 (전원 무관)"""
        d = self._ask(motor_id, CMD_ENC_ORIGINAL)
        return None if d is None else self._i32(d) % ENC_CPR

    def current(self, motor_id):
        s = self.status2(motor_id)
        return None if s is None else s[0]

    def status2(self, motor_id):
        """0x9C 1회 → (전류 A, 속도 dps, 단회전각 deg, 온도 C). 감시용 고속 경로."""
        d = self._ask(motor_id, CMD_STATUS2, window=0.08)
        if d is None:
            return None
        return (int.from_bytes(d[2:4], "little", signed=True) / 100.0,
                int.from_bytes(d[4:6], "little", signed=True),
                int.from_bytes(d[6:8], "little", signed=True),
                d[1])

    def status(self, motor_id):
        d = self._ask(motor_id, CMD_STATUS1)
        if d is None:
            return None
        return {"temp": d[1],
                "volt": int.from_bytes(d[4:6], "little") / 10.0,
                "err": int.from_bytes(d[6:8], "little")}

    # ---- 쓰기 -------------------------------------------------------------
    def _goto(self, motor_id, angle_deg, speed_dps):
        """0xA4 절대 위치 (raw 규약, 부호 반전 없음)"""
        data = bytearray(8)
        data[0] = CMD_POSITION
        data[1] = self.max_torque          # 0 이면 힘 제어 비활성 — 위 주석 참조
        data[2:4] = int(speed_dps).to_bytes(2, "little")
        data[4:8] = int(round(angle_deg * 100)).to_bytes(4, "little", signed=True)
        self._send(motor_id, data)

    def stop_all(self):
        """일반 정지. 속도 0 + 0x81 — 유지 토크는 남는다 (역구동 방지)."""
        for ax in self.axes:
            data = bytearray(8)
            data[0] = CMD_SPEED
            data[1] = self.max_torque      # 0 이면 능동 감속이 안 걸린다
            self._send(ax.motor_id, data)
        time.sleep(0.05)
        for ax in self.axes:
            self._send(ax.motor_id, [CMD_STOP, 0, 0, 0, 0, 0, 0, 0])

    def shutdown_all(self):
        """비상 차단. 0x80 만이 실제로 출력을 끊는다.

        0xA2 speed=0 과 0x81 은 모터를 여자 상태로 남긴다 (2026-09-08 실측:
        0x81 후에도 0.6~1.4A 가 계속 흘렀고, 0x80 에서만 0.00A 가 되었다).
        과전류·스톨 상황에서 0x81 을 쓰면 모터가 계속 힘을 쓰다가 소손된다.
        """
        for ax in self.axes:
            self._send(ax.motor_id, [CMD_SHUTDOWN, 0, 0, 0, 0, 0, 0, 0])
        time.sleep(0.05)
        for ax in self.axes:
            self._send(ax.motor_id, [CMD_SHUTDOWN, 0, 0, 0, 0, 0, 0, 0])

    # ---- 홈 계산 ----------------------------------------------------------
    def home_offset_deg(self, ax):
        """현재 위치에서 12시까지의 최단 각도 (deg). 0x61 절대 엔코더 기준."""
        cur = self.enc_single(ax.motor_id)
        if cur is None:
            return None
        d = ax.home_enc_single - cur
        if d > ENC_CPR / 2:
            d -= ENC_CPR
        elif d < -ENC_CPR / 2:
            d += ENC_CPR
        return d / ENC_CPR * 360.0

    def targets_for(self, turns):
        """각 축의 목표 각도(0x92 기준 절대) 계산. turns>0 = 좌측.

        출발점(12시)을 self._move_home 에 남긴다. 후퇴가 여기로 돌아가야 하기
        때문이다 — 자세한 이유는 _retreat 주석 참조.
        """
        out = {}
        home_of = {}
        for ax in self.axes:
            a = self.angle(ax.motor_id)
            off = self.home_offset_deg(ax)
            if a is None or off is None:
                return None
            home = a + off                       # 가장 가까운 12시 (0x92 기준)
            home_of[ax.motor_id] = home
            out[ax.motor_id] = home + ax.sign * turns * 360.0
        self._move_home = home_of
        return out

    # ---- 동작 -------------------------------------------------------------
    def move_turns(self, turns, speed_dps=None, timeout=None):
        """12시 정렬 후 동기 ±N회전. 성공 여부와 최종 각도를 돌려준다.

        timeout=None 이면 속도에서 계산한다:
          이론 소요시간 × 2 + 1.5초, 하한 DEFAULT_MOVE_TIMEOUT.
        12시 정렬 거리(최대 360°)까지 감안해 이론시간에 1회전을 더한다.
        """
        speed = speed_dps or self.speed_dps
        if timeout is None:
            expected = (abs(turns) + 1.0) * 360.0 / max(speed, 1.0)
            timeout = max(DEFAULT_MOVE_TIMEOUT, expected * 2.0 + 1.5)
        targets = self.targets_for(turns)
        if targets is None:
            self._warn("횡이동: 엔코더 읽기 실패 — 명령 취소")
            return False, {}

        side = "좌측" if turns > 0 else "우측"
        self._info(f"횡이동 {side} {abs(turns)*MM_PER_TURN:.0f}mm @ {speed} dps "
                   f"(제한 {timeout:.1f}s, 하드 {self.i_hard_a}A, I²t {I2T_BUDGET:.0f})")
        for ax in self.axes:
            self._info(f"횡이동 {side} {abs(turns)*MM_PER_TURN:.0f}mm | "
                       f"{ax.name} 0x{ax.motor_id:03X} → {targets[ax.motor_id]:+.2f}°")

        for mid, tgt in targets.items():
            self._goto(mid, tgt, speed)

        ok, last = self._wait(targets, timeout)
        if ok:
            self._info(f"횡이동 완료: " + ", ".join(
                f"0x{m:03X} {last.get(m, float('nan')):+.2f}°" for m in targets))
        return ok, last

    def _wait(self, targets, timeout):
        """도달 감시 + 전류/열/스톨 보호.

        0x9C 한 번에 전류·속도·각도가 오므로 축당 100Hz 이상으로 감시한다.
        도달 판정용 멀티턴 각도(0x92)는 100ms 간격으로만 읽는다 (감시 주기를
        떨어뜨리지 않기 위해).

        보호가 걸리면 stop 이 아니라 **shutdown(0x80)** 을 보낸다.
        """
        for pr in self.prot.values():
            pr.reset()

        t0 = time.time()
        last = {}
        last_angle_read = 0.0
        n_samples = 0

        while True:
            now = time.time()
            if now - t0 > timeout:
                return self._on_trip(None, f"이동 시간 제한 {timeout}s 초과",
                                     SEV_SOFT, last)

            # --- 고속 보호 감시 (매 루프) ---
            for mid in targets:
                st = self.status2(mid)
                if st is None:
                    continue
                cur, spd, ang, temp = st
                n_samples += 1
                reason, sev = self.prot[mid].update(cur, spd, temp, now)
                if self.recorder is not None:
                    # ang(단회전 raw)까지 넘긴다 — 위치별 전류 프로파일을 봐야
                    # "행정 어디서 얼마나 필요한가"를 외삽할 수 있다.
                    self.recorder(mid, now - t0, cur, spd, temp,
                                  self.prot[mid].i2t, ang)
                if reason:
                    return self._on_trip(mid, reason, sev, last)

            # --- 도달 판정 (100ms 간격) ---
            if now - last_angle_read >= 0.1:
                last_angle_read = now
                done = True
                for mid, tgt in targets.items():
                    a = self.angle(mid)
                    if a is None:
                        done = False
                        continue
                    last[mid] = a
                    if abs(a - tgt) > self.tolerance_deg:
                        done = False
                if done:
                    rate = n_samples / max(now - t0, 0.01)
                    self._info(f"보호 감시 {rate:.0f} Hz | " +
                               " | ".join(p.summary() for p in self.prot.values()))
                    return True, last

    # ---- 이상 대응 -------------------------------------------------------
    def _on_trip(self, mid, reason, sev, last):
        """보호 발동 처리.

        SEV_HARD(18A 초과) → 즉시 0x80. 그 외 → 12시로 후퇴 후 정지.
        후퇴가 즉시 차단보다 안전하다: 0x80 은 유지 토크를 없애기 때문에
        장비가 들린 상태였다면 그대로 떨어진다.
        """
        who = f"0x{mid:03X} " if mid is not None else ""
        if sev == SEV_HARD:
            self._warn(f"⛔ 즉시 차단: {who}{reason}")
            self.shutdown_all()
            for pr in self.prot.values():
                self._warn(f"   {pr.summary()}")
            return False, last

        self._warn(f"⚠️ 후퇴: {who}{reason} → 12시로 되돌립니다")
        for pr in self.prot.values():
            self._warn(f"   {pr.summary()}")
        ok = self._retreat()
        return False, last

    def _settle_before_retreat(self):
        """후퇴 전에 속도 0 을 보내고 전류가 정격 아래로 내려오기를 기다린다.

        스톨 직후에는 직전 부하 전류가 그대로 흐른다. 이를 기다리지 않고 후퇴
        감시를 시작하면 **첫 샘플이 곧바로 하드 리밋을 넘겨 0x80 으로 가버린다**
        (2026-09-15 실측: 후퇴 명령 5 ms 뒤 18.87A 로 차단). 그러면 소프트 트립의
        존재 이유인 "떨어뜨리지 않고 내려놓기" 가 그대로 무효가 된다.

        0xA2 speed=0 만 보낸다 — 여자 상태를 유지해 들린 장비를 잡고 있는다.
        """
        data = bytearray(8)
        data[0] = CMD_SPEED
        data[1] = self.max_torque
        for ax in self.axes:
            self._send(ax.motor_id, bytes(data))

        t0 = time.time()
        last_peak = 0.0
        while time.time() - t0 < RETREAT_SETTLE_T:
            last_peak = 0.0
            hot = False
            for ax in self.axes:
                st = self.status2(ax.motor_id)
                if st is None:
                    continue
                i = abs(st[0])
                last_peak = max(last_peak, i)
                if i > I_RATED_A:
                    hot = True
            if not hot:
                self._info(f"   후퇴 전 전류 안정화 완료 "
                           f"({time.time() - t0:.2f}s, {last_peak:.2f}A)")
                return True
        self._warn(f"   후퇴 전 전류가 {RETREAT_SETTLE_T}s 안에 정격 아래로 "
                   f"내려오지 않음 ({last_peak:.2f}A) — 그대로 후퇴 시도")
        return False

    def _retreat(self):
        """가장 가까운 12시로 저속 복귀. 장비를 통제된 속도로 내려놓기 위한 동작.

        후퇴 중에는 하드 리밋(self.i_hard_a)과 자체 타임아웃만 본다 —
        I²t 나 스톨로 후퇴를 또 중단시키면 들린 채로 멈춰버린다.
        """
        self._settle_before_retreat()

        # 후퇴 목표는 **이번 이동의 출발점**(12시)이다. '지금 위치에서 가장 가까운
        # 12시' 를 새로 찾으면 안 된다 — 한 바퀴 중 절반을 넘게 진행한 축은 진행
        # 방향 쪽 12시가 더 가까워서 **막힌 쪽으로 더 밀고 들어간다**.
        # 게다가 두 축은 부호 규약이 반대(+1/-1)라 각자 최근접을 고르면 물리적으로
        # 서로 반대 방향으로 갈 수 있다 (2026-09-15 실측: 0x143 -178.3°, 0x144
        # -132.2° → 좌우가 반대로 움직여 스텝이 어긋남).
        # 출발점으로 돌아가면 (1) 두 축이 항상 같은 물리 방향, (2) 같은 50mm 스텝
        # 유지, (3) 이미 지나온 저부하 구간을 되짚으므로 후퇴 자체가 가볍다.
        use_start = all(ax.motor_id in self._move_home for ax in self.axes)
        targets = {}
        for ax in self.axes:
            a = self.angle(ax.motor_id)
            if a is None:
                self._warn("후퇴 실패: 엔코더 읽기 불가 → 0x80 차단")
                self.shutdown_all()
                return False
            if use_start:
                tgt, why = self._move_home[ax.motor_id], "이동 시작점"
            else:
                off = self.home_offset_deg(ax)
                if off is None:
                    self._warn("후퇴 실패: 엔코더 읽기 불가 → 0x80 차단")
                    self.shutdown_all()
                    return False
                tgt, why = a + off, "최근접 12시"
            targets[ax.motor_id] = tgt
            self._info(f"   후퇴 0x{ax.motor_id:03X}: {a:+.1f}° → {tgt:+.1f}° "
                       f"({tgt - a:+.1f}°, {why})")

        for mid, tgt in targets.items():
            self._goto(mid, tgt, RETREAT_SPEED_DPS)

        t0 = time.time()
        while time.time() - t0 < RETREAT_TIMEOUT:
            now = time.time()
            for mid in targets:
                st = self.status2(mid)
                if st is None:
                    continue
                if abs(st[0]) > self.i_hard_a:
                    self._warn(f"후퇴 중 과전류 0x{mid:03X} {st[0]:+.2f}A "
                               f"> {self.i_hard_a}A → 0x80 차단")
                    self.shutdown_all()
                    return False
            if now - t0 > 0.2:
                done = True
                for mid, tgt in targets.items():
                    a = self.angle(mid)
                    if a is None or abs(a - tgt) > self.tolerance_deg * 3:
                        done = False
                if done:
                    self._info("   후퇴 완료 — 12시 복귀, 유지 토크 유지")
                    self.stop_all()
                    return True
        self._warn(f"후퇴 타임아웃 {RETREAT_TIMEOUT}s → 0x80 차단")
        self.shutdown_all()
        return False

    def close(self):
        try:
            self.stop_all()
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass
