#!/usr/bin/env python3
"""
SocketCAN을 사용하여 연결된 RMD 모터의 CAN ID를 스캔하는 도구
사용법: sudo python3 scan_motors.py
"""

import can
import time
import struct
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('motor_scanner')

class MotorScanner:
    """RMD 모터 CAN ID 스캐너"""
    
    def __init__(self, interface='can2', bitrate=1000000):
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        
        # 스캔할 CAN ID 범위 (RMD 모터는 일반적으로 0x140~0x147 범위)
        self.scan_ids = [0x141, 0x142, 0x143, 0x144, 0x145, 0x146, 0x147]
        
        # 응답 ID는 명령 ID + 0x100
        self.response_ids = {cmd_id: cmd_id + 0x100 for cmd_id in self.scan_ids}
        
        # RMD-X4 명령 코드
        self.CMD_READ_STATUS = 0x9C        # 모터 상태 읽기
        self.CMD_READ_MULTI_TURN = 0x92    # 멀티턴 각도 읽기
        
    def connect(self):
        """CAN 버스에 연결"""
        try:
            self.bus = can.interface.Bus(
                channel=self.interface,
                bustype='socketcan',
                bitrate=self.bitrate
            )
            logger.info(f"✅ {self.interface} 연결 성공 ({self.bitrate} bps)")
            return True
        except Exception as e:
            logger.error(f"❌ {self.interface} 연결 실패: {e}")
            return False
    
    def send_command(self, motor_id, command_byte):
        """모터에 명령 전송"""
        try:
            data = bytes([command_byte]) + b'\x00' * 7
            msg = can.Message(
                arbitration_id=motor_id,
                data=data,
                is_extended_id=False
            )
            self.bus.send(msg)
            logger.debug(f"📤 전송: ID=0x{motor_id:03X}, CMD=0x{command_byte:02X}")
            return True
        except Exception as e:
            logger.error(f"❌ 전송 실패 (ID=0x{motor_id:03X}): {e}")
            return False
    
    def wait_for_response(self, expected_id, timeout=0.5):
        """응답 대기"""
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is not None:
                    if msg.arbitration_id == expected_id:
                        logger.debug(f"📥 응답: ID=0x{msg.arbitration_id:03X}, 데이터={msg.data.hex().upper()}")
                        return msg
                    else:
                        logger.debug(f"📥 다른 ID 응답: 0x{msg.arbitration_id:03X} (예상: 0x{expected_id:03X})")
            except can.CanError:
                pass
        
        return None
    
    def parse_status_response(self, data):
        """0x9C 상태 응답 파싱"""
        if len(data) < 8 or data[0] != self.CMD_READ_STATUS:
            return None
        
        try:
            temperature = struct.unpack('<b', data[1:2])[0]  # 온도 (°C)
            current = struct.unpack('<h', data[2:4])[0] * 0.01  # 전류 (A)
            speed = struct.unpack('<h', data[4:6])[0]  # 속도 (dps)
            angle = struct.unpack('<h', data[6:8])[0]  # 각도 (도)
            
            return {
                'temperature': temperature,
                'current': current,
                'speed': speed,
                'angle': angle
            }
        except Exception as e:
            logger.error(f"응답 파싱 오류: {e}")
            return None
    
    def parse_multi_turn_response(self, data):
        """0x92 멀티턴 각도 응답 파싱"""
        if len(data) < 8 or data[0] != self.CMD_READ_MULTI_TURN:
            return None
        
        try:
            angle_raw = struct.unpack('<i', data[4:8])[0]  # int32, 0.01도 단위
            angle_degrees = angle_raw * 0.01
            
            return {
                'angle_degrees': angle_degrees,
                'angle_raw': angle_raw
            }
        except Exception as e:
            logger.error(f"응답 파싱 오류: {e}")
            return None
    
    def scan_motor(self, motor_id):
        """단일 모터 스캔"""
        logger.info(f"\n🔍 모터 ID 0x{motor_id:03X} 스캔 중...")
        
        # 1. 모터 상태 읽기 (0x9C)
        if self.send_command(motor_id, self.CMD_READ_STATUS):
            response = self.wait_for_response(self.response_ids[motor_id], timeout=0.5)
            
            if response:
                logger.info(f"✅ 모터 0x{motor_id:03X} 발견! (응답 ID: 0x{response.arbitration_id:03X})")
                
                # 응답 데이터 파싱
                status = self.parse_status_response(response.data)
                if status:
                    logger.info(f"   📊 상태: 온도={status['temperature']}°C, "
                              f"전류={status['current']:.2f}A, "
                              f"속도={status['speed']}dps, "
                              f"각도={status['angle']}°")
                
                # 2. 멀티턴 각도 읽기 (0x92)
                time.sleep(0.1)
                if self.send_command(motor_id, self.CMD_READ_MULTI_TURN):
                    response2 = self.wait_for_response(self.response_ids[motor_id], timeout=0.5)
                    
                    if response2:
                        multi_turn = self.parse_multi_turn_response(response2.data)
                        if multi_turn:
                            logger.info(f"   📐 멀티턴 각도: {multi_turn['angle_degrees']:.2f}° "
                                      f"(RAW: {multi_turn['angle_raw']})")
                
                return True
            else:
                logger.warning(f"⚠️  모터 0x{motor_id:03X} 응답 없음")
                return False
        
        return False
    
    def scan_all_motors(self):
        """모든 모터 스캔"""
        logger.info("="*60)
        logger.info("RMD 모터 CAN ID 스캔 시작")
        logger.info(f"인터페이스: {self.interface}")
        logger.info(f"스캔 범위: {[f'0x{id:03X}' for id in self.scan_ids]}")
        logger.info("="*60)
        
        found_motors = []
        
        for motor_id in self.scan_ids:
            if self.scan_motor(motor_id):
                found_motors.append(motor_id)
            time.sleep(0.2)  # 모터 간 간격
        
        # 결과 출력
        logger.info("\n" + "="*60)
        logger.info("스캔 결과")
        logger.info("="*60)
        
        if found_motors:
            logger.info(f"✅ 발견된 모터: {len(found_motors)}개")
            for motor_id in found_motors:
                logger.info(f"   - 0x{motor_id:03X} (응답 ID: 0x{self.response_ids[motor_id]:03X})")
        else:
            logger.warning("⚠️  발견된 모터 없음")
            logger.info("\n💡 확인사항:")
            logger.info("   1. 모터 전원이 켜져 있나요?")
            logger.info("   2. CAN 케이블이 올바르게 연결되어 있나요?")
            logger.info("   3. CAN 인터페이스가 활성화되어 있나요? (ip link show can2)")
            logger.info("   4. 다른 프로그램이 CAN 버스를 사용 중인가요?")
        
        logger.info("="*60)
        
        return found_motors
    
    def cleanup(self):
        """정리"""
        if self.bus:
            try:
                self.bus.shutdown()
                logger.info("CAN 버스 연결 해제")
            except Exception as e:
                logger.error(f"연결 해제 오류: {e}")

def main():
    """메인 함수"""
    import argparse
    
    parser = argparse.ArgumentParser(description='RMD 모터 CAN ID 스캐너')
    parser.add_argument('--interface', '-i', default='can2', help='CAN 인터페이스 (기본값: can2)')
    parser.add_argument('--bitrate', '-b', type=int, default=1000000, help='CAN 비트레이트 (기본값: 1000000)')
    parser.add_argument('--verbose', '-v', action='store_true', help='상세 로그 출력')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    scanner = MotorScanner(interface=args.interface, bitrate=args.bitrate)
    
    try:
        if not scanner.connect():
            logger.error("CAN 버스 연결 실패")
            return 1
        
        scanner.scan_all_motors()
        
        return 0
        
    except KeyboardInterrupt:
        logger.info("\n사용자에 의해 중단됨")
        return 0
    except Exception as e:
        logger.error(f"오류 발생: {e}", exc_info=True)
        return 1
    finally:
        scanner.cleanup()

if __name__ == '__main__':
    exit(main())


