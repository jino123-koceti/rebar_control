#!/usr/bin/env python3
"""
액션 큐 생성 모듈

결속 포인트 목록으로부터 실행할 액션 큐를 생성.
자세변경 시퀀스, Z↑+XY 최적화 포함.
"""

from rebar_vision.tying_common import ActionType, Action


class ActionBuilder:
    """결속 액션 큐 생성기.

    사용법:
        builder = ActionBuilder(params)
        actions, new_pose = builder.build(points, current_pose, tying_direction)
    """

    def __init__(self, params):
        """
        Args:
            params: dict with keys:
                tying_z_down_mm, x_home_mm, pose_change_y_near, pose_change_y_far,
                yaw_right_cam, yaw_left_cam, yaw_left_cam_approach, yaw_mid,
                home_ref_yaw_deg
        """
        self._p = params

    def update_params(self, **kwargs):
        """런타임에 파라미터 업데이트 (예: home_ref_yaw_deg 변경 시)."""
        self._p.update(kwargs)

    def build(self, points, current_pose, tying_direction, logger=None):
        """포인트 목록 → 액션 큐 생성.

        Args:
            points: list of (label, x_mm, y_mm, side)
            current_pose: 'right' or 'left'
            tying_direction: 'forward' or 'reverse'
            logger: ROS2 logger (optional, for info logging)

        Returns:
            (action_queue, final_pose_state)
        """
        queue = []
        pose = current_pose

        right_pts = [(l, x, y) for l, x, y, s in points if s == 'right']
        left_pts = [(l, x, y) for l, x, y, s in points if s == 'left']

        if logger:
            logger.info(
                f'자세 상태: {current_pose} | '
                f'방향: {tying_direction} | '
                f'Right: {len(right_pts)}개, Left: {len(left_pts)}개')

        if tying_direction == 'reverse':
            left_reversed = list(reversed(left_pts))
            right_reversed = list(reversed(right_pts))

            queue.append(Action(
                action_type=ActionType.LOG,
                message='=== reverse: Left(P6→P4) → 자세변경 → Right(P3→P1) ===',
            ))

            if left_reversed and pose != 'left':
                queue.extend(self._pose_change_right_to_left())
                pose = 'left'

            for label, x, y in left_reversed:
                queue.extend(self._tying_actions(label, x, y))

            if right_reversed and pose == 'left':
                queue.extend(self._pose_change_left_to_right())
                pose = 'right'

            for label, x, y in right_reversed:
                queue.extend(self._tying_actions(label, x, y))

            if right_reversed:
                pose = 'right'
            elif left_reversed:
                pose = 'left'

        else:
            # forward (기본)
            queue.append(Action(
                action_type=ActionType.LOG,
                message='=== forward: Right(P1→P3) → 자세변경 → Left(P4→P6) ===',
            ))

            if right_pts and pose == 'left':
                queue.extend(self._pose_change_left_to_right())
                pose = 'right'

            for label, x, y in right_pts:
                queue.extend(self._tying_actions(label, x, y))

            if left_pts and pose != 'left':
                queue.extend(self._pose_change_right_to_left())
                pose = 'left'

            for label, x, y in left_pts:
                queue.extend(self._tying_actions(label, x, y))

            if left_pts:
                pose = 'left'
            elif right_pts:
                pose = 'right'

        # Z↑+XY 최적화
        queue = self._optimize_z_xy_overlap(queue, logger)

        return queue, pose

    def build_pose_change(self, from_pose, to_pose):
        """자세변경 액션만 생성.

        Returns:
            list of Action
        """
        if from_pose == 'right' and to_pose == 'left':
            return self._pose_change_right_to_left()
        elif from_pose == 'left' and to_pose == 'right':
            return self._pose_change_left_to_right()
        return []

    def _tying_actions(self, label, x, y):
        """한 포인트 결속 액션 시퀀스: XY이동 → Z하강 → 트리거 → Z상승"""
        z_mm = self._p['tying_z_down_mm']
        return [
            Action(action_type=ActionType.MOVE_XY,
                   x_mm=x, y_mm=y, point_label=label,
                   message=f'{label}: X={x:.1f}mm, Y={y:.1f}mm'),
            Action(action_type=ActionType.MOVE_Z_DOWN,
                   z_mm=z_mm, point_label=label,
                   message=f'{label}: Z {z_mm}mm 하강'),
            Action(action_type=ActionType.TRIGGER,
                   point_label=label,
                   message=f'{label}: 트리거'),
            Action(action_type=ActionType.MOVE_Z_UP,
                   z_mm=z_mm, point_label=label,
                   message=f'{label}: Z {z_mm}mm 상승'),
        ]

    def _pose_change_right_to_left(self):
        """Right → Left 자세변경 시퀀스 (5단계)."""
        p = self._p
        return [
            Action(action_type=ActionType.LOG,
                   message='=== 자세 변경: Right → Left ==='),
            Action(action_type=ActionType.MOVE_XY,
                   x_mm=p['x_home_mm'], y_mm=p['pose_change_y_near'],
                   message=f'자세변경 1/5: X={p["x_home_mm"]:.1f}mm(홈), Y={p["pose_change_y_near"]}mm'),
            Action(action_type=ActionType.MOVE_YAW,
                   yaw_deg=p['yaw_mid'],
                   message=f'자세변경 2/5: Yaw → {p["yaw_mid"]}°'),
            Action(action_type=ActionType.MOVE_XY,
                   x_mm=p['x_home_mm'], y_mm=p['pose_change_y_far'],
                   message=f'자세변경 3/5: X={p["x_home_mm"]:.1f}mm(홈), Y → {p["pose_change_y_far"]}mm'),
            Action(action_type=ActionType.MOVE_YAW,
                   yaw_deg=p['yaw_left_cam_approach'],
                   message=f'자세변경 4/5: Yaw → {p["yaw_left_cam_approach"]}° (접근)'),
            Action(action_type=ActionType.MOVE_YAW,
                   yaw_deg=p['yaw_left_cam'],
                   message=f'자세변경 5/5: Yaw → {p["yaw_left_cam"]}° (최종)'),
        ]

    def _pose_change_left_to_right(self):
        """Left → Right 자세복귀 시퀀스 (5단계)."""
        p = self._p
        yaw_near_home = p['home_ref_yaw_deg'] + 5.0
        return [
            Action(action_type=ActionType.LOG,
                   message='=== 자세 복귀: Left → Right ==='),
            Action(action_type=ActionType.MOVE_XY,
                   x_mm=p['x_home_mm'], y_mm=p['pose_change_y_far'],
                   message=f'자세복귀 1/5: X={p["x_home_mm"]:.1f}mm(홈), Y={p["pose_change_y_far"]}mm'),
            Action(action_type=ActionType.MOVE_YAW,
                   yaw_deg=p['yaw_mid'],
                   message=f'자세복귀 2/5: Yaw → {p["yaw_mid"]}°'),
            Action(action_type=ActionType.MOVE_XY,
                   x_mm=p['x_home_mm'], y_mm=p['pose_change_y_near'],
                   message=f'자세복귀 3/5: X={p["x_home_mm"]:.1f}mm(홈), Y → {p["pose_change_y_near"]}mm'),
            Action(action_type=ActionType.MOVE_YAW,
                   yaw_deg=yaw_near_home,
                   message=f'자세복귀 4/5: Yaw → {yaw_near_home:.1f}° (홈+5°)'),
            Action(action_type=ActionType.MOVE_YAW,
                   yaw_deg=p['yaw_right_cam'],
                   message=f'자세복귀 5/5: Yaw → {p["yaw_right_cam"]}°'),
        ]

    def _optimize_z_xy_overlap(self, queue, logger=None):
        """Z_UP 직후 MOVE_XY → MOVE_Z_UP_WITH_XY로 합침."""
        optimized = []
        i = 0
        merged_count = 0
        while i < len(queue):
            action = queue[i]
            if (action.action_type == ActionType.MOVE_Z_UP
                    and i + 1 < len(queue)
                    and queue[i + 1].action_type == ActionType.MOVE_XY):
                next_action = queue[i + 1]
                optimized.append(Action(
                    action_type=ActionType.MOVE_Z_UP_WITH_XY,
                    z_mm=action.z_mm,
                    x_mm=next_action.x_mm,
                    y_mm=next_action.y_mm,
                    point_label=action.point_label,
                    message=f'{action.point_label}: Z↑{action.z_mm}mm + '
                            f'{next_action.point_label} XY({next_action.x_mm:.1f},{next_action.y_mm:.1f})',
                ))
                i += 2
                merged_count += 1
            else:
                optimized.append(action)
                i += 1

        if merged_count > 0 and logger:
            logger.info(
                f'  [최적화] Z↑+XY 병합: {merged_count}개 (액션 {len(optimized)}개)')

        return optimized
