#!/usr/bin/env python3

import struct
import time
from typing import Optional, Dict, Any, Tuple
from enum import IntEnum
import logging

class ControlMode(IntEnum):
    """RMD-X4 제어 모드"""
    POSITION_CONTROL = 0x01
    VELOCITY_CONTROL = 0x02
    CURRENT_CONTROL = 0x03
    TORQUE_CONTROL = 0x04
    FORCE_CONTROL = 0x05
    INCREMENTAL_CONTROL = 0x06

class CommandType(IntEnum):
    """RMD-X4 명령 타입"""
    # 시스템 제어 명령
    BRAKE_RELEASE = 0x77
    BRAKE_LOCK = 0x78
    MOTOR_STOP = 0x81
    MOTOR_SHUTDOWN = 0x80
    MOTOR_ENABLE = 0x88
    # ⚠ **에러 복구는 이것이다.** V4.3 §2.26 System Reset — 모터 프로그램을
    #   재시작한다(전원 재투입과 같은 효과, 응답 없음). 멀티턴이 날아가므로
    #   복구 뒤에는 **전체 호밍이 필수**다.
    SYSTEM_RESET = 0x76

    # PID 및 가속도 명령
    READ_PID_PARAMETERS = 0x30
    WRITE_PID_PARAMETERS = 0x31
    READ_ACCELERATION = 0x42
    WRITE_ACCELERATION = 0x43

    # 엔코더 및 위치 명령
    READ_ENCODER_DATA = 0x90
    WRITE_ENCODER_OFFSET = 0x91
    READ_MULTI_TURN_ANGLE = 0x92
    READ_SINGLE_TURN_ANGLE = 0x94

    # 상태 읽기 명령
    READ_MOTOR_STATUS = 0x9C
    # ⚠⚠ **V4.3 에 `0x9B` 라는 명령은 없다.** 2026-10-07 에 Z 의 엔코더 데이터
    #   오류(0x4000)에 네 번 보냈지만 아무 효과가 없었다 — 당연하다. 쓰지 말 것.
    #   리셋은 `SYSTEM_RESET`(0x76) 이다.
    CLEAR_MOTOR_FAULT = 0x9B        # [무효 — V4.3 에 없음]
    READ_MOTOR_ERROR = 0x9D
    READ_MOTOR_VOLTAGE = 0x9E
    READ_MOTOR_TEMPERATURE = 0x9F

    # 제어 명령 (중요)
    SET_MOTOR_TORQUE = 0xA1        # 전류/토크 제어
    SET_MOTOR_SPEED = 0xA2          # 속도 제어
    SET_MOTOR_POSITION = 0xA4       # 위치 제어
    SET_MOTOR_INCREMENTAL_POSITION = 0xA8  # 증분 위치 제어

class RMDX4Protocol:
    """
    RMD-X4 모터 프로토콜 처리 클래스
    CAN 통신을 통한 모터 제어 명령 생성 및 응답 파싱
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        # 프로토콜 상수
        self.MOTOR_CMD_ID = 0x140
        self.MOTOR_RESP_ID = 0x141
        self.MOTOR_BROADCAST_ID = 0x140
        
        # 제어 모드별 명령
        self.CONTROL_COMMANDS = {
            ControlMode.POSITION_CONTROL: 0x01,
            ControlMode.VELOCITY_CONTROL: 0x02,
            ControlMode.CURRENT_CONTROL: 0x03,
            ControlMode.TORQUE_CONTROL: 0x04,
            ControlMode.FORCE_CONTROL: 0x05,
            ControlMode.INCREMENTAL_CONTROL: 0x06
        }
    
    def create_position_control_command(self, position: float, velocity: float = 0.0, 
                                      acceleration: float = 0.0, mode: int = 0) -> bytes:
        """위치 제어 명령 생성"""
        try:
            # 위치값을 32비트 정수로 변환 (라디안 -> 0.01도 단위)
            position_int = int(position * 180.0 / 3.14159 * 100)
            
            # 속도값을 32비트 정수로 변환 (rad/s -> rpm 단위)
            velocity_int = int(velocity * 60.0 / (2 * 3.14159))
            
            # 가속도값을 32비트 정수로 변환 (rad/s² -> rpm/s 단위)
            acceleration_int = int(acceleration * 60.0 / (2 * 3.14159))
            
            # 명령 데이터 구성
            data = struct.pack('<B', 0xA4)  # 명령 타입
            data += struct.pack('<i', position_int)  # 위치 (4바이트)
            data += struct.pack('<i', velocity_int)  # 속도 (4바이트)
            data += struct.pack('<i', acceleration_int)  # 가속도 (4바이트)
            data += struct.pack('<B', mode)  # 모드
            
            return data
            
        except Exception as e:
            self.logger.error(f"위치 제어 명령 생성 오류: {e}")
            return b''
    
    def create_velocity_control_command(self, velocity: float, acceleration: float = 0.0) -> bytes:
        """속도 제어 명령 생성"""
        try:
            # 속도값을 32비트 정수로 변환 (rad/s -> rpm 단위)
            velocity_int = int(velocity * 60.0 / (2 * 3.14159))
            
            # 가속도값을 32비트 정수로 변환 (rad/s² -> rpm/s 단위)
            acceleration_int = int(acceleration * 60.0 / (2 * 3.14159))
            
            # 명령 데이터 구성
            data = struct.pack('<B', 0xA2)  # 명령 타입
            data += struct.pack('<i', velocity_int)  # 속도 (4바이트)
            data += struct.pack('<i', acceleration_int)  # 가속도 (4바이트)
            data += b'\x00\x00'  # 패딩
            
            return data
            
        except Exception as e:
            self.logger.error(f"속도 제어 명령 생성 오류: {e}")
            return b''
    
    def create_current_control_command(self, current: float) -> bytes:
        """전류 제어 명령 생성"""
        try:
            # 전류값을 16비트 정수로 변환 (A -> 0.01A 단위)
            current_int = int(current * 100)
            
            # 명령 데이터 구성
            data = struct.pack('<B', 0xA1)  # 명령 타입
            data += struct.pack('<h', current_int)  # 전류 (2바이트)
            data += b'\x00\x00\x00\x00\x00'  # 패딩
            
            return data
            
        except Exception as e:
            self.logger.error(f"전류 제어 명령 생성 오류: {e}")
            return b''
    
    def create_torque_control_command(self, torque: float) -> bytes:
        """토크 제어 명령 생성"""
        try:
            # 토크값을 16비트 정수로 변환 (Nm -> 0.01Nm 단위)
            torque_int = int(torque * 100)
            
            # 명령 데이터 구성
            data = struct.pack('<B', 0xA3)  # 명령 타입
            data += struct.pack('<h', torque_int)  # 토크 (2바이트)
            data += b'\x00\x00\x00\x00\x00'  # 패딩
            
            return data
            
        except Exception as e:
            self.logger.error(f"토크 제어 명령 생성 오류: {e}")
            return b''
    
    def create_system_command(self, command_type: CommandType, data: bytes = b'') -> bytes:
        """시스템 명령 생성"""
        try:
            # 명령 데이터 구성
            cmd_data = struct.pack('<B', command_type.value)
            cmd_data += data
            
            # 8바이트까지 패딩
            while len(cmd_data) < 8:
                cmd_data += b'\x00'
            
            return cmd_data[:8]  # 8바이트로 제한
            
        except Exception as e:
            self.logger.error(f"시스템 명령 생성 오류: {e}")
            return b''
    
    def parse_motor_response(self, can_id: int, data: bytes) -> Optional[Dict[str, Any]]:
        """모터 응답 파싱"""
        try:
            if len(data) < 1:
                return None
            
            command_type = data[0]
            response = {
                'command_type': command_type,
                'timestamp': time.time(),
                'raw_data': data
            }
            
            # 명령 타입별 응답 파싱
            if command_type == 0xA4:  # 위치 제어 응답
                response.update(self._parse_position_response(data))
            elif command_type == 0xA2:  # 속도 제어 응답
                response.update(self._parse_velocity_response(data))
            elif command_type == 0xA1:  # 전류 제어 응답
                response.update(self._parse_current_response(data))
            elif command_type == 0xA3:  # 토크 제어 응답
                response.update(self._parse_torque_response(data))
            elif command_type == 0x90:  # 엔코더 데이터 응답
                response.update(self._parse_encoder_response(data))
            elif command_type == 0x9A:  # 모터 상태 응답
                response.update(self._parse_status_response(data))
            else:
                response['parsed'] = False
                response['message'] = f"알 수 없는 명령 타입: 0x{command_type:02X}"
            
            return response
            
        except Exception as e:
            self.logger.error(f"모터 응답 파싱 오류: {e}")
            return None
    
    def _parse_position_response(self, data: bytes) -> Dict[str, Any]:
        """위치 제어 응답 파싱"""
        if len(data) < 9:
            return {'error': '데이터 길이 부족'}
        
        try:
            position_int = struct.unpack('<i', data[1:5])[0]
            velocity_int = struct.unpack('<i', data[5:9])[0]
            
            # 단위 변환
            position = position_int * 3.14159 / 180.0 / 100.0  # 0.01도 -> 라디안
            velocity = velocity_int * 2 * 3.14159 / 60.0  # rpm -> rad/s
            
            return {
                'parsed': True,
                'position': position,
                'velocity': velocity,
                'position_raw': position_int,
                'velocity_raw': velocity_int
            }
        except Exception as e:
            return {'error': f'위치 응답 파싱 오류: {e}'}
    
    def _parse_velocity_response(self, data: bytes) -> Dict[str, Any]:
        """속도 제어 응답 파싱"""
        if len(data) < 5:
            return {'error': '데이터 길이 부족'}
        
        try:
            velocity_int = struct.unpack('<i', data[1:5])[0]
            velocity = velocity_int * 2 * 3.14159 / 60.0  # rpm -> rad/s
            
            return {
                'parsed': True,
                'velocity': velocity,
                'velocity_raw': velocity_int
            }
        except Exception as e:
            return {'error': f'속도 응답 파싱 오류: {e}'}
    
    def _parse_current_response(self, data: bytes) -> Dict[str, Any]:
        """전류 제어 응답 파싱"""
        if len(data) < 3:
            return {'error': '데이터 길이 부족'}
        
        try:
            current_int = struct.unpack('<h', data[1:3])[0]
            current = current_int / 100.0  # 0.01A -> A
            
            return {
                'parsed': True,
                'current': current,
                'current_raw': current_int
            }
        except Exception as e:
            return {'error': f'전류 응답 파싱 오류: {e}'}
    
    def _parse_torque_response(self, data: bytes) -> Dict[str, Any]:
        """토크 제어 응답 파싱"""
        if len(data) < 3:
            return {'error': '데이터 길이 부족'}
        
        try:
            torque_int = struct.unpack('<h', data[1:3])[0]
            torque = torque_int / 100.0  # 0.01Nm -> Nm
            
            return {
                'parsed': True,
                'torque': torque,
                'torque_raw': torque_int
            }
        except Exception as e:
            return {'error': f'토크 응답 파싱 오류: {e}'}
    
    def _parse_encoder_response(self, data: bytes) -> Dict[str, Any]:
        """엔코더 응답 파싱"""
        if len(data) < 9:
            return {'error': '데이터 길이 부족'}
        
        try:
            # 듀얼 엔코더 데이터 파싱
            encoder1 = struct.unpack('<i', data[1:5])[0]
            encoder2 = struct.unpack('<i', data[5:9])[0]
            
            return {
                'parsed': True,
                'encoder1': encoder1,
                'encoder2': encoder2,
                'encoder1_angle': encoder1 * 360.0 / (2**18),  # 18비트 엔코더
                'encoder2_angle': encoder2 * 360.0 / (2**17)   # 17비트 엔코더
            }
        except Exception as e:
            return {'error': f'엔코더 응답 파싱 오류: {e}'}
    
    def _parse_status_response(self, data: bytes) -> Dict[str, Any]:
        """모터 상태 응답 파싱"""
        if len(data) < 8:
            return {'error': '데이터 길이 부족'}
        
        try:
            # 상태 정보 파싱
            motor_status = struct.unpack('<B', data[1:2])[0]
            error_code = struct.unpack('<H', data[2:4])[0]
            temperature = struct.unpack('<h', data[4:6])[0]
            voltage = struct.unpack('<H', data[6:8])[0]
            
            return {
                'parsed': True,
                'motor_status': motor_status,
                'error_code': error_code,
                'temperature': temperature / 10.0,  # 0.1도 단위
                'voltage': voltage / 10.0,  # 0.1V 단위
                'has_error': error_code != 0
            }
        except Exception as e:
            return {'error': f'상태 응답 파싱 오류: {e}'}
