#!/usr/bin/env python3
"""
Velocity Profiler
사다리꼴 속도 프로파일 생성기 (Tire Roller 방식)

실시간 피드백 기반으로 가감속 프로파일 생성
"""

import numpy as np


class VelocityProfiler:
    """
    사다리꼴 속도 프로파일 생성기

    Tire Roller의 VelocityProfiler를 참조하여 단순화
    """

    def __init__(self, v_max):
        """
        Parameters:
        - v_max: 최대 속도 (단위: dps 또는 m/s)
        """
        self.acc_time = 0.5  # 가속 시간 (초) - 0x143: 150dps 도달 시 0.5초
        self.dec_time = 1.0  # 감속 시간 (초) - 정밀 정지를 위해 천천히 감속
        self.v_min = 0.1     # 최소 속도 (단위: dps 또는 m/s)
        self.current_velocity = 0.0
        self.v_max = abs(v_max)

        # 가속도 계산
        self.acceleration = self.v_max / self.acc_time
        self.deceleration = self.v_max / self.dec_time

        # 가속/감속 거리 계산
        power_vmax = self.v_max ** 2
        self.acc_dist = 0.5 * power_vmax / self.acceleration
        # 감속 거리에 안전 마진 20% 추가 (관성 + 제어 지연)
        self.dec_dist = 0.5 * power_vmax / self.deceleration * 1.2

    def set_maxvel(self, v_max):
        """최대 속도 변경"""
        self.v_max = abs(v_max)
        self.acceleration = self.v_max / self.acc_time
        self.deceleration = self.v_max / self.dec_time
        power_vmax = self.v_max ** 2
        self.acc_dist = 0.5 * power_vmax / self.acceleration
        # 감속 거리에 안전 마진 20% 추가 (관성 + 제어 지연)
        self.dec_dist = 0.5 * power_vmax / self.deceleration * 1.2

    def get_simple_trapezoidal_profile_velocity(self, distance_moved, distance_togo):
        """
        사다리꼴 프로파일 속도 계산 (Tire Roller 방식)

        매 제어 주기마다 호출하여 현재 목표 속도를 얻음

        Parameters:
        - distance_moved: 이미 이동한 거리 (현재는 미사용)
        - distance_togo: 남은 거리 (단위: degree 또는 m)

        Returns:
        - vel: 현재 목표 속도 (단위: dps 또는 m/s)
        """
        # 감속 구간: 남은 거리가 감속 거리보다 작으면 감속
        if distance_togo <= self.dec_dist:
            vel = np.sqrt(2 * self.deceleration * np.abs(distance_togo)) - 0.25

        # 가속 구간: 현재 속도가 최대 속도보다 작으면 가속
        elif self.current_velocity < self.v_max:
            vel = self.current_velocity + self.acceleration * 0.1  # dt = 0.1초 가정
            if vel > self.v_max:
                vel = self.v_max

        # 감속 구간: 현재 속도가 최대 속도보다 크면 감속
        elif self.current_velocity > self.v_max:
            vel = self.current_velocity - self.deceleration * 0.1

        # 등속 구간
        else:
            vel = self.v_max

        # 최소 속도 제한
        if vel < self.v_min:
            vel = self.v_min

        # 상태 업데이트
        self.current_velocity = vel

        return vel
