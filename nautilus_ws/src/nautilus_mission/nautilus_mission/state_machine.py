#!/usr/bin/env python3

import time
from typing import Optional

from rclpy.node import Node
from transitions import Machine


from enums.ObjectID import ObjectID
from enums.VisionAction import VisionAction
from enums.DetectionIndex import DetectionIndex
from .detection_store import DetectionStore

from .mission_objectives import mission_list, Objective, ActionType


class StateMachine:
    """Mission decision logic only. No ROS subscriptions and no vision error calculation."""

    def __init__(self, node: Node, detection_store: DetectionStore):
        self.node = node
        self.detection_store = detection_store

        self.objectives : list[Objective] = mission_list

        self.objective_index = 0
        self.current_objective: Optional[Objective] = None
        self.target_ids: Optional[list[ObjectID]] = None
        self.vision_action = VisionAction.IDLE
        self.mean_depth_forward_cam: Optional[int] = None

        self.target_missing_count = 0
        self.target_missing_limit = 5
        self.state_start_time = time.monotonic()

        states = [
            'IDLE',
            'LOAD_OBJECTIVE',
            'SEARCH_TARGET',
            'CENTER_TARGET',
            'APPROACH_TARGET',
            'EXECUTE_ACTION',
            'MISSION_COMPLETE',
        ]

        transitions = [
            {'trigger': 'start_mission', 'source': 'IDLE', 'dest': 'LOAD_OBJECTIVE'},
            {'trigger': 'objective_loaded', 'source': 'LOAD_OBJECTIVE', 'dest': 'SEARCH_TARGET'},
            {'trigger': 'no_more_objectives', 'source': 'LOAD_OBJECTIVE', 'dest': 'MISSION_COMPLETE'},
            {'trigger': 'target_found', 'source': 'SEARCH_TARGET', 'dest': 'CENTER_TARGET'},
            {'trigger': 'target_lost', 'source': ['CENTER_TARGET', 'APPROACH_TARGET'], 'dest': 'SEARCH_TARGET'},
            {'trigger': 'target_centered_event', 'source': 'CENTER_TARGET', 'dest': 'APPROACH_TARGET'},
            {'trigger': 'target_reached', 'source': 'APPROACH_TARGET', 'dest': 'EXECUTE_ACTION'},
            {'trigger': 'no_target_to_be_reached', 'source': '*', 'dest': 'EXECUTE_ACTION'},
            {'trigger': 'action_done', 'source': 'EXECUTE_ACTION', 'dest': 'LOAD_OBJECTIVE'},
            {'trigger': 'finish_mission', 'source': '*', 'dest': 'MISSION_COMPLETE'},
        ]

        self.machine = Machine(
            model=self,
            states=states,
            initial='IDLE',
            transitions=transitions,
            after_state_change='state_changed',
            ignore_invalid_triggers=True,
        )

    def tick(self):
        if self.target_ids is None:
            self.no_target_to_be_reached()

        if self.state == 'SEARCH_TARGET':
            self.vision_action = VisionAction.IDLE
            if self.is_target_present():
                self.target_found()

        elif self.state == 'CENTER_TARGET':
            self.vision_action = VisionAction.CENTER_TARGET
            if self.is_target_lost_filtered():
                self.target_lost()
            elif self.is_target_centered() and self.is_target_perpendicular() and self.state_lifespan > 5.0:
                self.target_centered_event()

        elif self.state == 'APPROACH_TARGET':
            self.vision_action = VisionAction.APPROACH_TARGET
            if self.is_target_lost_filtered():
                self.target_lost()
            elif self.is_target_approached():
                self.target_reached()

        elif self.state == 'EXECUTE_ACTION':
            self.vision_action = self.get_vision_action_for_current_objective()
            self.run_current_action()
            if self.is_current_action_done():
                self.objective_index += 1
                self.action_done()

        else:
            self.vision_action = VisionAction.IDLE

    def on_enter_LOAD_OBJECTIVE(self):
        self.vision_action = VisionAction.IDLE
        self.node.publish_forward_cmd(1500)

        if self.objective_index >= len(self.objectives):
            self.no_more_objectives()
            return

        self.current_objective = self.objectives[self.objective_index]
        self.target_ids = self.current_objective.target_ids

        self.node.get_logger().info(
            f'Loaded objective {self.objective_index + 1}/{len(self.objectives)}: '
            f'{self.current_objective.name}'
        )

        if self.target_ids is not None:
            self.node.get_logger().info('Target IDs: ' + ', '.join(target.name for target in self.target_ids))
        else:
            self.node.get_logger().info('Target IDs: None')

        if self.current_objective.depth_threshold is not None:
            self.node.publish_depth_threshold(self.current_objective.depth_threshold)

        self.objective_loaded()

    def on_enter_CENTER_TARGET(self):
        self.target_missing_count = 0

    def on_enter_APPROACH_TARGET(self):
        self.target_missing_count = 0

    def on_enter_EXECUTE_ACTION(self):
        if self.current_objective is None:
            self.finish_mission()
            return

        self.node.get_logger().info(
            f'Executing action {self.current_objective.action_type.name} '
            f'for objective {self.current_objective.name}'
        )

    def on_enter_MISSION_COMPLETE(self):
        self.target_ids = None
        self.vision_action = VisionAction.IDLE
        self.node.publish_forward_cmd(1500)
        self.node.get_logger().info('Mission complete')

    def run_current_action(self):
        if self.state != 'EXECUTE_ACTION' or self.current_objective is None:
            return

        if self.current_objective.action_type == ActionType.FORWARD:
            self.node.publish_forward_cmd(self.current_objective.action_forward_pwm)

    def get_vision_action_for_current_objective(self) -> VisionAction:
        if self.current_objective is None:
            return VisionAction.IDLE

        if self.current_objective.action_type == ActionType.CIRCLE_MARKER:
            return VisionAction.CIRCLE_MARKER

        return VisionAction.IDLE

    def is_current_action_done(self) -> bool:
        if self.current_objective is None:
            return True

        action = self.current_objective.action_type

        if action == ActionType.NONE:
            return True

        if action == ActionType.FORWARD:
            required_time = max(self.current_objective.action_duration, self.current_objective.min_action_lifespan)
            return self.state_lifespan >= required_time

        if action == ActionType.CIRCLE_MARKER:
            if self.current_objective.mean_depth_target is None:
                return False

            return (
                self.mean_depth_forward_cam == self.current_objective.mean_depth_target
                and self.state_lifespan >= self.current_objective.min_action_lifespan
            )

        return False

    def is_target_present(self) -> bool:
        return self.detection_store.get_detection(self.target_ids) is not None

    def is_target_lost_filtered(self) -> bool:
        if self.is_target_present():
            self.target_missing_count = 0
            return False

        self.target_missing_count += 1
        return self.target_missing_count >= self.target_missing_limit

    def is_target_centered(self) -> bool:
        if self.current_objective is None:
            return False

        target = self.detection_store.get_detection(self.target_ids)
        if target is None:
            return False

        px = target[DetectionIndex.CENTER_PX]
        return abs(px) < self.current_objective.center_tolerance_px

    def is_target_approached(self) -> bool:
        if self.current_objective is None:
            return False

        target = self.detection_store.get_detection(self.target_ids)
        if target is None:
            return False

        depth = target[DetectionIndex.DEPTH_MM]
        return depth < self.current_objective.approach_distance

    def is_target_perpendicular(self) -> bool:
        if self.current_objective is None:
            return False

        target = self.detection_store.get_detection(self.target_ids)
        if target is None:
            return False

        angle = target[DetectionIndex.ANGLE_DEG]
        return abs(angle) < self.current_objective.angle_tolerance_deg

    def state_changed(self):
        self.state_start_time = time.monotonic()
        self.node.get_logger().info(f'Entered state {self.state}')

    @property
    def state_lifespan(self):
        return time.monotonic() - self.state_start_time
