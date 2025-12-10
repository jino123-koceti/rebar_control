#!/usr/bin/env python3
"""
정확한 속도 계산을 위한 정밀 로그 기록 스크립트

이 스크립트는 ROS2 토픽을 구독하여 다음 이벤트를 나노초 단위 타임스탬프와 함께 기록합니다:
- 회전 명령 발행 시점
- 위치 업데이트 시점 (0x141, 0x142)
- 목표 근접/도달 시점
- 각 이벤트의 정확한 위치 값

로그 파일: /tmp/precise_speed_analysis.log
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32, Int32
import time
import os
from datetime import datetime


class PreciseSpeedLogger(Node):
    """정밀 속도 분석을 위한 로그 기록 노드"""
    
    def __init__(self):
        super().__init__('precise_speed_logger')
        
        # 로그 파일 설정
        self.log_file = '/tmp/precise_speed_analysis.log'
        self.log_fd = open(self.log_file, 'a', encoding='utf-8', buffering=1)  # line buffering
        
        # 회전 명령 추적
        self.rotation_commands = {}  # {rotation_id: {left_target, right_target, speed, timestamp_ns}}
        self.rotation_counter = 0
        
        # 현재 위치 추적
        self.current_positions = {
            'left': None,
            'right': None
        }
        
        # 목표 근접/도달 추적
        self.target_reached = {
            'left': None,
            'right': None
        }
        
        # 로그 시작 메시지
        self.log_event("=" * 80)
        self.log_event("정밀 속도 분석 로거 시작")
        self.log_event(f"로그 파일: {self.log_file}")
        self.log_event("=" * 80)
        
        # 토픽 구독
        # 1. 회전 명령 구독 (iron_md_teleop_node에서 발행하는 위치 제어 명령 토픽 구독)
        self.left_wheel_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/motor_0x141/position',
            self.left_wheel_command_callback,
            10
        )
        
        self.right_wheel_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/motor_0x142/position',
            self.right_wheel_command_callback,
            10
        )
        
        # 2. 위치 업데이트 구독 (position_control_node에서 발행하는 위치 정보)
        self.left_wheel_pos_sub = self.create_subscription(
            Float32,
            'motor_0x141_position',
            self.left_wheel_position_callback,
            10
        )
        
        self.right_wheel_pos_sub = self.create_subscription(
            Float32,
            'motor_0x142_position',
            self.right_wheel_position_callback,
            10
        )
        
        # 3. 목표 도달 구독
        self.goal_reached_sub = self.create_subscription(
            Int32,
            '/motor_goal_reached',
            self.goal_reached_callback,
            10
        )
        
        self.get_logger().info(f"정밀 속도 분석 로거 시작됨 (로그 파일: {self.log_file})")
    
    def get_timestamp_ns(self):
        """나노초 단위 타임스탬프 반환"""
        return time.time_ns()
    
    def format_timestamp(self, timestamp_ns):
        """타임스탬프를 읽기 쉬운 형식으로 변환"""
        timestamp_sec = timestamp_ns / 1e9
        dt = datetime.fromtimestamp(timestamp_sec)
        return f"{dt.strftime('%Y-%m-%d %H:%M:%S')}.{int((timestamp_ns % 1_000_000_000) / 1_000_000):06d}"
    
    def log_event(self, message):
        """이벤트를 로그 파일에 기록"""
        timestamp_ns = self.get_timestamp_ns()
        formatted_time = self.format_timestamp(timestamp_ns)
        log_line = f"[{timestamp_ns}] [{formatted_time}] {message}\n"
        self.log_fd.write(log_line)
        self.log_fd.flush()  # 즉시 디스크에 기록
    
    def left_wheel_command_callback(self, msg: Float64MultiArray):
        """좌측 주행 모터 위치 제어 명령 콜백"""
        if len(msg.data) >= 2:
            target = float(msg.data[0])
            speed = float(msg.data[1])
            timestamp_ns = self.get_timestamp_ns()
            
            # 회전 명령인지 확인 (360도 또는 1200도 단위)
            if self.current_positions['left'] is not None:
                delta = abs(target - self.current_positions['left'])
                rotation_angle = None
                
                # 360도 회전 감지
                if abs(delta - 360.0) < 5.0 or abs(delta + 360.0) < 5.0:
                    rotation_angle = 360.0
                # 1200도 회전 감지 (약 3.33회전 = 600mm)
                elif abs(delta - 1200.0) < 10.0 or abs(delta + 1200.0) < 10.0:
                    rotation_angle = 1200.0
                
                if rotation_angle is not None:
                    # 회전 명령 감지
                    self.rotation_counter += 1
                    rotation_id = self.rotation_counter
                    
                    if rotation_id not in self.rotation_commands:
                        self.rotation_commands[rotation_id] = {
                            'left_target': target,
                            'right_target': None,
                            'speed': speed,
                            'left_timestamp_ns': timestamp_ns,
                            'right_timestamp_ns': None,
                            'start_left_pos': self.current_positions['left'],
                            'start_right_pos': self.current_positions['right'],
                            'rotation_angle': rotation_angle
                        }
                    else:
                        self.rotation_commands[rotation_id]['left_target'] = target
                        self.rotation_commands[rotation_id]['speed'] = speed
                        self.rotation_commands[rotation_id]['left_timestamp_ns'] = timestamp_ns
                        self.rotation_commands[rotation_id]['start_left_pos'] = self.current_positions['left']
                        self.rotation_commands[rotation_id]['rotation_angle'] = rotation_angle
                    
                    direction = f"+{rotation_angle:.0f}°" if (target - self.current_positions['left']) > 0 else f"-{rotation_angle:.0f}°"
                    self.log_event(
                        f"[ROTATION_CMD] ID={rotation_id} LEFT 0x141: "
                        f"시작={self.current_positions['left']:.3f}° → 목표={target:.3f}° ({direction}), "
                        f"속도={speed:.1f}dps"
                    )
    
    def right_wheel_command_callback(self, msg: Float64MultiArray):
        """우측 주행 모터 위치 제어 명령 콜백"""
        if len(msg.data) >= 2:
            target = float(msg.data[0])
            speed = float(msg.data[1])
            timestamp_ns = self.get_timestamp_ns()
            
            # 회전 명령인지 확인 (360도 또는 1200도 단위)
            if self.current_positions['right'] is not None:
                delta = abs(target - self.current_positions['right'])
                rotation_angle = None
                
                # 360도 회전 감지
                if abs(delta - 360.0) < 5.0 or abs(delta + 360.0) < 5.0:
                    rotation_angle = 360.0
                # 1200도 회전 감지 (약 3.33회전 = 600mm)
                elif abs(delta - 1200.0) < 10.0 or abs(delta + 1200.0) < 10.0:
                    rotation_angle = 1200.0
                
                if rotation_angle is not None:
                    # 가장 최근 회전 명령에 우측 정보 추가
                    if self.rotation_counter > 0 and self.rotation_counter in self.rotation_commands:
                        rotation_id = self.rotation_counter
                        self.rotation_commands[rotation_id]['right_target'] = target
                        self.rotation_commands[rotation_id]['right_timestamp_ns'] = timestamp_ns
                        self.rotation_commands[rotation_id]['start_right_pos'] = self.current_positions['right']
                        self.rotation_commands[rotation_id]['rotation_angle'] = rotation_angle
                        
                        direction = f"+{rotation_angle:.0f}°" if (target - self.current_positions['right']) > 0 else f"-{rotation_angle:.0f}°"
                        self.log_event(
                            f"[ROTATION_CMD] ID={rotation_id} RIGHT 0x142: "
                            f"시작={self.current_positions['right']:.3f}° → 목표={target:.3f}° ({direction}), "
                            f"속도={speed:.1f}dps"
                        )
                        
                        # 양쪽 명령이 모두 들어왔으면 회전 시작 로그
                        if (self.rotation_commands[rotation_id]['left_timestamp_ns'] and 
                            self.rotation_commands[rotation_id]['right_timestamp_ns']):
                            left_delay = (self.rotation_commands[rotation_id]['right_timestamp_ns'] - 
                                        self.rotation_commands[rotation_id]['left_timestamp_ns']) / 1e9
                            self.log_event(
                                f"[ROTATION_START] ID={rotation_id} 양쪽 명령 완료, "
                                f"명령 간격={left_delay*1000:.2f}ms, "
                                f"속도={speed:.1f}dps, 회전각={rotation_angle:.0f}°"
                            )
    
    def left_wheel_position_callback(self, msg: Float32):
        """좌측 주행 모터 위치 업데이트 콜백"""
        current_pos = float(msg.data)
        timestamp_ns = self.get_timestamp_ns()
        
        prev_pos = self.current_positions['left']
        self.current_positions['left'] = current_pos
        
        if prev_pos is not None:
            delta = current_pos - prev_pos
            self.log_event(
                f"[POS_UPDATE] LEFT 0x141: {prev_pos:.3f}° → {current_pos:.3f}° "
                f"(변화: {delta:+.3f}°)"
            )
            
            # 진행 중인 회전 명령에 대한 위치 업데이트
            for rotation_id, cmd in self.rotation_commands.items():
                if cmd['left_target'] is not None:
                    error = abs(current_pos - cmd['left_target'])
                    # 회전 각도에 따라 허용 오차 조정
                    rotation_angle = cmd.get('rotation_angle', 360.0)
                    tolerance = 10.0 if rotation_angle >= 1200.0 else 5.0
                    
                    if error < tolerance:  # 목표 근접
                        if cmd.get('left_near_timestamp_ns') is None:
                            cmd['left_near_timestamp_ns'] = timestamp_ns
                            duration_ns = timestamp_ns - cmd['left_timestamp_ns']
                            duration_sec = duration_ns / 1e9
                            distance = abs(current_pos - cmd['start_left_pos'])
                            self.log_event(
                                f"[TARGET_NEAR] ID={rotation_id} LEFT 0x141: "
                                f"목표 근접! 위치={current_pos:.3f}° (목표: {cmd['left_target']:.3f}°, "
                                f"오차: {error:.3f}°), "
                                f"이동 거리: {distance:.3f}°, "
                                f"소요 시간: {duration_sec:.6f}초 ({duration_ns}ns)"
                            )
        else:
            self.log_event(f"[POS_UPDATE] LEFT 0x141: 초기 위치={current_pos:.3f}°")
    
    def right_wheel_position_callback(self, msg: Float32):
        """우측 주행 모터 위치 업데이트 콜백"""
        current_pos = float(msg.data)
        timestamp_ns = self.get_timestamp_ns()
        
        prev_pos = self.current_positions['right']
        self.current_positions['right'] = current_pos
        
        if prev_pos is not None:
            delta = current_pos - prev_pos
            self.log_event(
                f"[POS_UPDATE] RIGHT 0x142: {prev_pos:.3f}° → {current_pos:.3f}° "
                f"(변화: {delta:+.3f}°)"
            )
            
            # 진행 중인 회전 명령에 대한 위치 업데이트
            for rotation_id, cmd in self.rotation_commands.items():
                if cmd['right_target'] is not None:
                    error = abs(current_pos - cmd['right_target'])
                    # 회전 각도에 따라 허용 오차 조정
                    rotation_angle = cmd.get('rotation_angle', 360.0)
                    tolerance = 10.0 if rotation_angle >= 1200.0 else 5.0
                    
                    if error < tolerance:  # 목표 근접
                        if cmd.get('right_near_timestamp_ns') is None:
                            cmd['right_near_timestamp_ns'] = timestamp_ns
                            duration_ns = timestamp_ns - cmd['right_timestamp_ns']
                            duration_sec = duration_ns / 1e9
                            distance = abs(current_pos - cmd['start_right_pos'])
                            self.log_event(
                                f"[TARGET_NEAR] ID={rotation_id} RIGHT 0x142: "
                                f"목표 근접! 위치={current_pos:.3f}° (목표: {cmd['right_target']:.3f}°, "
                                f"오차: {error:.3f}°), "
                                f"이동 거리: {distance:.3f}°, "
                                f"소요 시간: {duration_sec:.6f}초 ({duration_ns}ns)"
                            )
                            
                            # 양쪽 모두 목표 근접했는지 확인
                            if (cmd.get('left_near_timestamp_ns') and 
                                cmd.get('right_near_timestamp_ns')):
                                # 양쪽 모두 완료
                                left_duration_ns = cmd['left_near_timestamp_ns'] - cmd['left_timestamp_ns']
                                right_duration_ns = cmd['right_near_timestamp_ns'] - cmd['right_timestamp_ns']
                                max_duration_ns = max(left_duration_ns, right_duration_ns)
                                max_duration_sec = max_duration_ns / 1e9
                                
                                # 회전 각도에 따라 이동 거리 계산
                                # 360도 = 18cm = 180mm
                                # 1200도 = 60cm = 600mm
                                if rotation_angle >= 1200.0:
                                    distance_cm = 60.0  # 1200도 = 600mm
                                    distance_mm = 600.0
                                else:
                                    distance_cm = 18.0  # 360도 = 180mm
                                    distance_mm = 180.0
                                
                                speed_mm_per_sec = distance_mm / max_duration_sec
                                
                                self.log_event(
                                    f"[ROTATION_COMPLETE] ID={rotation_id} 양쪽 모두 목표 근접 완료! "
                                    f"회전각: {rotation_angle:.0f}°, "
                                    f"왼쪽 소요: {left_duration_ns/1e9:.6f}초, "
                                    f"오른쪽 소요: {right_duration_ns/1e9:.6f}초, "
                                    f"최종 소요: {max_duration_sec:.6f}초, "
                                    f"이동 거리: {distance_cm}cm ({distance_mm}mm), "
                                    f"속도: {speed_mm_per_sec:.2f}mm/sec"
                                )
        else:
            self.log_event(f"[POS_UPDATE] RIGHT 0x142: 초기 위치={current_pos:.3f}°")
    
    def goal_reached_callback(self, msg: Int32):
        """목표 도달 콜백"""
        motor_id = msg.data
        timestamp_ns = self.get_timestamp_ns()
        
        if motor_id == 0x141:
            self.log_event(f"[GOAL_REACHED] LEFT 0x141 목표 도달")
        elif motor_id == 0x142:
            self.log_event(f"[GOAL_REACHED] RIGHT 0x142 목표 도달")
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.log_event("=" * 80)
        self.log_event("정밀 속도 분석 로거 종료")
        self.log_event("=" * 80)
        if self.log_fd:
            self.log_fd.close()
        super().destroy_node()


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    node = PreciseSpeedLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    except Exception as e:
        node.get_logger().error(f"오류 발생: {e}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

