#!/usr/bin/env python3
"""
결속 오케스트레이터 공통 데이터 클래스

ActionType, Action, TyingState 등 모듈 간 공유 데이터 정의.
tying_orchestrator_node, detection_manager, action_builder, z_torque_monitor에서 import.
"""

from enum import Enum, auto
from dataclasses import dataclass


class ActionType(Enum):
    """액션 큐 항목 유형"""
    MOVE_XY = auto()             # XY 스테이지 절대위치 이동 (mm)
    MOVE_YAW = auto()            # Yaw 절대각도 이동 (deg, 0x92 멀티턴)
    MOVE_Z_DOWN = auto()         # Z축 하강 (상대 이동, mm)
    MOVE_Z_UP = auto()           # Z축 상승 (상대 이동, mm)
    MOVE_Z_UP_WITH_XY = auto()   # Z상승 + 조기 XY이동 (연속 동작)
    TRIGGER = auto()             # 트리거 동작 (발사 → 정지 → 원복 → 정지)
    LOG = auto()                 # 로그 출력


@dataclass
class Action:
    """실행할 액션 정의"""
    action_type: ActionType
    x_mm: float = 0.0
    y_mm: float = 0.0
    yaw_deg: float = 0.0
    z_mm: float = 0.0            # Z축 이동 거리 (mm)
    message: str = ''
    point_label: str = ''


class TyingState(Enum):
    """오케스트레이터 상태"""
    IDLE = auto()
    DETECTING = auto()
    EXECUTING_ACTION = auto()
    WAITING_SETTLE = auto()
    WAITING_Z = auto()           # Z축 이동 대기 (계산된 시간)
    WAITING_Z_XY = auto()        # Z상승 중 + XY 병렬 이동 대기
    WAITING_TRIGGER = auto()     # 트리거 동작 대기 (다단계)
    PAUSED = auto()
    COMPLETE = auto()
    ERROR = auto()
