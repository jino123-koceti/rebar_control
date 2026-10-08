#!/usr/bin/env python3
"""Pololu driver module for motor controllers using pyserial

This module handles the lower level logic of writing to a pololu motor
controller using python.

Example:
    Here is an example of usage. The first Daisy object sets the port and
    all the of the rest use the same port.
        motor0 = Daisy(0, port="/dev/ttyUSB0")  # only first one has to be set
        motor1 = Daisy(1)  # all following will use the first port
        motor2 = Daisy(2)

        motor1.forward(1600)  # drive forward the motor with device number 1

        # All motors are using port "/dev/ttyUSB0" to send commands
        # Each device is addressed by their device number
"""

from __future__ import division
import serial

BAUD_SYNC = b'\x80'  # Required to sync the baud rate for older devices
PROTOCOL = b'\xAA'   # First part of command to write using Pololu Protocol
FORWARD = b'\x05'    # Drive motor forward command
BACKWARD = b'\x06'   # Drive motor reverse command
START = b'\x03'      # Exit Safe-Start mode so the motor can run
STOP = b'\x60'       # Stops the motor and enters Safe-Start mode

# Compact Protocol commands (no device ID needed)
COMPACT_FORWARD = b'\x85'   # Drive motor forward (Compact)
COMPACT_BACKWARD = b'\x86'  # Drive motor reverse (Compact)
COMPACT_START = b'\x83'     # Exit Safe-Start (Compact)
COMPACT_STOP = b'\xE0'      # Stop motor (Compact)
GET_VAR = b'\x21'          # Get Variable (Pololu Protocol)
COMPACT_GET_VAR = b'\xA1'  # Get Variable (Compact)

# ⚠⚠ [2026-10-08] **보드 상태를 읽을 수 있어야 한다.**
#   예전에는 읽기가 없어서 "쓰기는 성공하는데 건이 안 움직인다" 를 더 파고들 수
#   없었다. Simple Motor Controller 는 에러가 걸리면 속도 명령을 **조용히 무시**
#   하므로, 보드에 물어보는 것이 유일한 확인 방법이다.
VAR_ERROR_STATUS = 0        # 지금 걸려 있는 에러 (이게 0 이 아니면 안 움직인다)
VAR_ERRORS_OCCURRED = 1     # 걸렸던 적이 있는 에러 (읽으면 지워진다)
VAR_LIMIT_STATUS = 3        # 리미트·킬 스위치 상태
# ⚠ 아래 네 개가 "보드가 무시하는가 / 보드는 돌리는데 바깥이 안 도는가" 를 가른다.
#   Target Speed 가 0 이면 보드가 우리 명령을 **안 받아들인 것**이고(입력 모드가
#   Serial/USB 가 아니거나 safe-start), ±3200 인데 안 움직이면 **보드 바깥**이다.
VAR_TARGET_SPEED = 20       # 목표 속도 (-3200~3200, 부호있음)
VAR_SPEED = 21              # 현재 속도 (-3200~3200, 부호있음)
VAR_INPUT_VOLTAGE = 23      # VIN (mV)
VAR_TEMPERATURE = 24        # 온도 (0.1°C)

# Error Status / Errors Occurred 비트 (Pololu 0J44 문서)
ERROR_FLAGS = (
    (0x0001, 'Safe Start Violation'),
    (0x0002, 'Required Channel Invalid'),
    (0x0004, 'Serial Error'),
    (0x0008, 'Command Timeout'),
    (0x0010, 'Limit/Kill Switch'),
    (0x0020, 'Low VIN'),
    (0x0040, 'High VIN'),
    (0x0080, 'Over Temperature'),
    (0x0100, 'Motor Driver Error'),
    (0x0200, 'ERR Line High'),
)


def decode_errors(bits):
    """에러 비트 → 사람이 읽는 문자열."""
    if not bits:
        return '없음'
    hit = [m for b, m in ERROR_FLAGS if bits & b]
    return ' · '.join(hit) or f'알 수 없는 비트 0x{bits:04X}'


class Daisy(object):
    """Represents a single Pololu Simple Motor Controller in a daisy chain

    This class offers an interface to daisy chain several Pololu Simple
    Motor Controllers. The daisy chained modules all share the same serial
    connection. To target specific devices, every command is sent with the
    device number of the device. You must configure the devices to have
    different numbers. This must be done using the Simple Motor Controller
    setup software (https://www.pololu.com/docs/0J44/3).

    Initialize every separate board you want to talk to with its
    set device number.

    Attributes:
        dev_num (bytes): Device number of Pololu board to be commanded
    """

    count = 0  # static variable to keep track of number of Daisys open
    ser = None  # initialize static variable ser to None
    port = None       # 다시 열 때 쓰는 포트 (아래 `_open` 참고)
    baudrate = 9600

    def __init__(self, dev_num, port="/dev/ttyUSB0", baudrate=9600, use_compact=True):
        """Set motor id and open serial connection if not already open

        Args:
            dev_num: Device number (0-127) for daisy chaining
            port: Serial port
            baudrate: Baud rate
            use_compact: Use Compact Protocol (True) or Pololu Protocol (False)
        """

        if dev_num < 0 or dev_num > 127:
            raise Exception("Invalid motor id, must set to id of motor (0-127) for daisy chaining")

        Daisy.count += 1  # increment count of controllers
        self.dev_num = bytes([dev_num])  # set device number to use in other commands
        self.use_compact = use_compact  # protocol mode

        # ⚠⚠ [2026-10-08] **포트 설정을 기억해 둔다 — 다시 열어야 할 때가 있다.**
        #   이 핸들은 클래스 변수에 캐시되는데, USB 허브가 재열거되면 (상부 전원,
        #   카메라 재삽입 등) 장치 노드가 ttyACM0 ↔ ttyACM1 로 바뀌고 **열어 둔
        #   핸들이 죽는다.** 그러면 이후 모든 쓰기가 `Errno 5 I/O error` 로 실패하는데
        #   예전에는 다시 열지 않아 **영구히 죽은 채** 남았다. udev 심링크
        #   (`/dev/pololu_trigger`)는 경로만 고정해 주고 핸들은 지켜 주지 않는다.
        #   실측 2026-10-08: 결속 미션 8점에서 `[fire] 결속건 작동` 이 전부 찍혔는데
        #   Pololu 쓰기는 88번 모두 실패했다 — **한 점도 안 당겨졌고 미션은 성공으로
        #   보고했다.** 시연에서 이보다 나쁜 실패 양상은 없다.
        Daisy.port, Daisy.baudrate = port, baudrate
        self._reopening = False              # `_write` 재진입 방지 (아래 참고)
        # 데이지체인은 포트를 공유한다 — 이미 열려 있으면 그대로 쓴다
        if Daisy.ser is None or not Daisy.ser.isOpen():
            self._open()
        self._exit_safe_start()  # make it so pololu reacts to commands

    @classmethod
    def _open(cls):
        """직렬 포트를 (필요하면 다시) 연다.

        ⚠⚠ [2026-10-08] **새 핸들이 열린 뒤에만 바꿔 넣는다.**
        전에는 먼저 `cls.ser = None` 으로 비우고 열었다. 그래서 장치가 USB 에서
        사라진 동안(상부 허브 가지가 통째로 떨어져 `/dev/pololu_trigger` 자체가
        없어졌다) 열기가 실패하면 `ser` 가 **None 으로 남고**, 그 뒤 모든 쓰기가
        `'NoneType' object has no attribute 'write'` 로 죽었다 — 장치가 돌아와도
        복구되지 않았다. 실측 2026-10-08 10:59 에 그 상태로 트리거가 멈췄다.
        """
        new = serial.Serial(cls.port, baudrate=cls.baudrate, timeout=1)
        old, cls.ser = cls.ser, new         # 성공한 뒤에 교체한다
        if old is not None:
            try:
                old.close()
            except Exception:
                pass                        # 이미 죽은 핸들이면 닫기도 실패한다
        new.write(BAUD_SYNC)                # sync old devices by writing 0x80

    def _write(self, cmd):
        """쓰고, 실패하면 **포트를 다시 열어 한 번 더** 시도한다.

        ⚠ 한 번만 재시도한다. 계속 매달리면 토픽 콜백이 막히고, 실패가 계속되면
          그건 배선·전원 문제이므로 사람이 봐야 한다 (노드가 ERROR 로 올린다).
        """
        # ⚠ `ser` 가 None 일 수 있다 — 열기에 실패한 적이 있으면 그렇다.
        #   그때도 **재연결 경로로 간다** (예전에는 AttributeError 로 죽었다).
        try:
            if Daisy.ser is None:
                raise OSError('포트가 열려 있지 않다')
            Daisy.ser.write(cmd)
            Daisy.ser.flush()
            return
        except (OSError, serial.SerialException, AttributeError) as e:
            if self._reopening:
                raise                        # 재연결 중의 실패는 그대로 올린다
            print(f"Pololu 쓰기 실패 ({e}) — 포트를 다시 엽니다: {Daisy.port}")
        # ⚠ `_exit_safe_start` 가 다시 `_write` 를 타므로 **재진입을 막아야 한다** —
        #   안 막으면 포트가 계속 죽어 있을 때 무한 재귀로 노드가 죽는다
        self._reopening = True
        try:
            self._open()                     # 실패하면 예외가 호출부로 올라간다
            Daisy.ser.write(cmd)
            Daisy.ser.flush()
            self._exit_safe_start()          # 재연결된 보드는 safe-start 상태다
        finally:
            self._reopening = False

    def __del__(self):
        """Decrement count, stop motor, and close port if it's the last connection"""
        Daisy.count -= 1  # decrement count of controllers
        self._stop_motor()  # safely stop current motor
        
        # if this is the last controller open
        if Daisy.count <= 0:
            if Daisy.ser is not None and Daisy.ser.isOpen():
                Daisy.ser.close()
                print("Serial connection closed")

    def _send_command(self, command, compact_command, databyte3, databyte4):
        """Sends a two-byte command using the appropriate protocol."""
        if self.use_compact:
            cmd = compact_command + bytes([databyte3, databyte4])
            print(f"DEBUG: Sending compact command: {cmd.hex()} (data3={databyte3}, data4={databyte4})")
        else:
            cmd = PROTOCOL + self.dev_num + command + bytes([databyte3, databyte4])
            print(f"DEBUG: Sending command: {cmd.hex()} (dev={self.dev_num[0]}, cmd={command.hex()}, data3={databyte3}, data4={databyte4})")
        self._write(cmd)  # 실패하면 포트를 다시 열고 재시도한다

    def _send_command_single(self, command, compact_command):
        """Sends a one-byte command using the appropriate protocol."""
        if self.use_compact:
            cmd = compact_command
            print(f"DEBUG: Sending compact single command: {cmd.hex()}")
        else:
            cmd = PROTOCOL + self.dev_num + command
            print(f"DEBUG: Sending single command: {cmd.hex()} (dev={self.dev_num[0]}, cmd={command.hex()})")
        self._write(cmd)  # 실패하면 포트를 다시 열고 재시도한다

    def _exit_safe_start(self):
        """Exit safe start so you can freely send commands to Pololu.
        This must be run before run other commands."""
        self._send_command_single(START, COMPACT_START)

    def _stop_motor(self):
        """Immediately stops the motor and enters safe start mode"""
        self._send_command_single(STOP, COMPACT_STOP)

    # USER METHODS

    def forward(self, speed):
        """Drive motor forward at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds
        self._exit_safe_start()

        # Pololu Protocol uses 5-bit + 7-bit format for 12-bit data
        # databyte3 contains the lower 5 bits
        # databyte4 contains the upper 7 bits
        self._send_command(FORWARD, COMPACT_FORWARD, speed & 0x1F, (speed >> 5) & 0x7F)

    def backward(self, speed):
        """Drive motor backward (reverse) at specified speed (0 to 3200)"""
        speed = max(min(3200, speed), 0)  # enforce bounds
        self._exit_safe_start()
        self._send_command(BACKWARD, COMPACT_BACKWARD, speed & 0x1F, (speed >> 5) & 0x7F)

    def get_variable(self, var_id):
        """Get Variable 로 16비트 값을 읽는다. 못 읽으면 None.

        ⚠ 응답이 안 오면 **None 을 돌려주고 끝낸다** — 진단용이라 여기서 예외를
          올리면 멀쩡한 구동까지 막는다. 포트 timeout 은 1초로 열려 있다.
        """
        try:
            Daisy.ser.reset_input_buffer()
            if self.use_compact:
                self._write(COMPACT_GET_VAR + bytes([var_id]))
            else:
                self._write(PROTOCOL + self.dev_num + GET_VAR + bytes([var_id]))
            raw = Daisy.ser.read(2)
            if len(raw) != 2:
                return None
            return raw[0] | (raw[1] << 8)
        except Exception:
            return None

    def error_status(self):
        """지금 걸려 있는 에러 비트. 0 이면 정상, None 이면 못 읽음."""
        return self.get_variable(VAR_ERROR_STATUS)

    def errors_occurred(self):
        """걸렸던 적 있는 에러 비트. **읽으면 보드에서 지워진다.**"""
        return self.get_variable(VAR_ERRORS_OCCURRED)

    @staticmethod
    def _signed(v):
        """16비트 무부호 → 부호있는 값."""
        return None if v is None else (v - 65536 if v >= 32768 else v)

    def diagnostics(self):
        """보드가 **지금 무엇을 하고 있다고 생각하는지**. 못 읽으면 값이 None.

        ⚠ 변수 번호는 Pololu 0J44 문서 기준이다 (20 목표속도 / 21 현재속도 /
          23 VIN mV / 24 온도 0.1°C). 번호가 틀리면 값이 엉뚱하게 나오므로
          **VIN 을 실제 공급전압과 대조해 번호가 맞는지 먼저 확인할 것.**
        """
        return {
            'target': self._signed(self.get_variable(VAR_TARGET_SPEED)),
            'speed': self._signed(self.get_variable(VAR_SPEED)),
            'vin_mv': self.get_variable(VAR_INPUT_VOLTAGE),
            'temp_c': (lambda t: None if t is None else t / 10.0)(
                self.get_variable(VAR_TEMPERATURE)),
            'limit': self.get_variable(VAR_LIMIT_STATUS),
        }

    def drive(self, speed):
        """Drive motor in direction based on speed (-3200, 3200)

        ⚠⚠ [2026-10-08] **매번 safe-start 를 먼저 푼다.**
        Simple Motor Controller 는 **어떤 에러든** 걸리면 모터를 세우고
        `Safe Start Violation` 을 걸어 두며, 그 뒤에는 "Exit safe start" 를 다시
        받기 전까지 **속도 명령을 조용히 무시한다.** 에러 원인은 직렬 타임아웃,
        저전압(VIN), 과온도, 리미트 입력 등이고 전부 일시적일 수 있다.
        예전에는 `__init__` 에서 한 번만 풀었다 — 그래서 한 번 에러가 걸리면
        쓰기는 계속 성공하는데 건은 영구히 안 움직였고, 로그에는 아무 흔적이
        남지 않았다 (쓰기 실패가 아니므로). Pololu 공식 예제도 매 명령 전에
        보낸다. 1바이트라 비용이 없다.
        """
        self._exit_safe_start()
        if speed < 0:
            self.backward(-speed)
        else:
            self.forward(speed)

    def stop(self):
        """Stop the motor"""
        self._stop_motor()
