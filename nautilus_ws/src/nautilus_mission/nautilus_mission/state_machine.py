#!/usr/bin/env python3

import time
from typing import Optional

from rclpy.node import Node
from transitions import Machine

from enums.ObjectID import ObjectID
from enums.VisionAction import VisionAction
from enums.DetectionIndex import DetectionIndex
from enums.ServoEnum import ServoEnum

from .detection_store import DetectionStore
from .mission_objectives import mission_list, Objective, ActionType
from .ekf_reset import reset_ekf_pose
from nautilus_services import request_depth_change, reset_pids


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
        self.forward_position = 0.0
        self.lateral_position = 0.0
        self.ekf_resetted = False

        self.target_missing_count = 0
        self.target_missing_limit = 500
        self.state_start_time = time.monotonic()
        self.execute_action_start_time = None

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
            {'trigger': 'load_next_objective', 'source': 'LOAD_OBJECTIVE', 'dest': 'SEARCH_TARGET', 'conditions': 'has_more_objectives', 'after': 'load_current_objective'},
            {'trigger': 'load_next_objective', 'source': 'LOAD_OBJECTIVE', 'dest': 'MISSION_COMPLETE', 'unless': 'has_more_objectives'},
            {'trigger': 'target_found', 'source': 'SEARCH_TARGET', 'dest': 'CENTER_TARGET'},
            {'trigger': 'target_lost', 'source': ['CENTER_TARGET', 'APPROACH_TARGET'], 'dest': 'SEARCH_TARGET'},
            {'trigger': 'target_centered_event', 'source': 'CENTER_TARGET', 'dest': 'APPROACH_TARGET'},
            {'trigger': 'target_reached', 'source': 'APPROACH_TARGET', 'dest': 'EXECUTE_ACTION', 'conditions': 'ekf_reset_done'},
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
            send_event=True,
        )

    def tick(self):
        if self.state == 'LOAD_OBJECTIVE':
            self.load_next_objective()

        elif self.target_ids is None and self.state == 'SEARCH_TARGET':
            self.no_target_to_be_reached()

        elif self.state == 'SEARCH_TARGET':
            self.spin_search()
            self.vision_action = VisionAction.IDLE
            if self.is_target_present():
                self.target_found()

        elif self.state == 'CENTER_TARGET':
            if self.current_objective.center.full_centering:
                self.vision_action = VisionAction.CENTER_TARGET
            else:
                self.vision_action = VisionAction.CENTER_FOV
            if self.is_target_lost_filtered():
                self.target_lost()
            if self.is_target_centered():
                if self.is_target_perpendicular() and self.current_objective.center.full_centering:
                    self.target_centered_event()
                elif not self.current_objective.center.full_centering:
                    self.target_centered_event()

        elif self.state == 'APPROACH_TARGET':
            self.vision_action = VisionAction.APPROACH_TARGET
            if self.is_target_lost_filtered():
                self.target_lost()
            if self.is_target_approached():
                self.target_reached()

        elif self.state == 'EXECUTE_ACTION':
            self.vision_action = self.get_vision_action_for_current_objective()
            self.run_current_action()
            if self.is_current_action_done():
                self.objective_index += 1
                self.action_done()

        elif self.state == 'MISSION_COMPLETE':
            return

        else:
            self.vision_action = VisionAction.IDLE

    def on_enter_LOAD_OBJECTIVE(self, event):
        self.vision_action = VisionAction.IDLE
        self.node.publish_forward_cmd(1500)

    def load_current_objective(self, event):
        self.current_objective = self.objectives[self.objective_index]
        self.target_ids = self.current_objective.target_ids

        self.node.get_logger().info('\n')
        self.node.get_logger().info(f'Loaded objective {self.objective_index + 1}/{len(self.objectives)}: 'f'{self.current_objective.name}')

        if self.target_ids is not None:
            self.node.get_logger().info('Target IDs: ' + ', '.join(target.name for target in self.target_ids))
        else:
            self.node.get_logger().info('Target IDs: None')

        if self.current_objective.detections_depth_filter_mm is not None:
            self.node.publish_detections_depth_filter_mm(self.current_objective.detections_depth_filter_mm)

        if self.current_objective.target_auv_depth_m is not None:
            request_depth_change(self.node, self.current_objective.target_auv_depth_m)
        if self.current_objective.action.type == ActionType.FIRE_TORPEDO:
            self.current_objective.action.fired = False

    def on_enter_CENTER_TARGET(self, event):
        reset_pids(self.node)
        self.target_missing_count = 0

    def on_enter_APPROACH_TARGET(self, event):
        reset_pids(self.node)
        self.target_missing_count = 0
        self.ekf_resetted = False
        
    def on_exit_APPROACH_TARGET(self, event):
        self.ekf_resetted = reset_ekf_pose(self.node)

    def on_enter_EXECUTE_ACTION(self, event):
        reset_pids(self.node)

        if self.current_objective is None:
            self.finish_mission()
            return
    
        self.execute_action_start_time = time.monotonic()

        self.node.get_logger().info(
            f'Executing action {self.current_objective.action.type.name} '
            f'for objective {self.current_objective.name}'
        )

    def on_enter_MISSION_COMPLETE(self, event):
        self.target_ids = None
        self.vision_action = VisionAction.IDLE
        self.node.publish_forward_cmd(1500)
        self.node.get_logger().info('Mission complete')

    def has_more_objectives(self, event):
        return self.objective_index < len(self.objectives)

    def run_current_action(self):
        if self.state != 'EXECUTE_ACTION' or self.current_objective is None:
            return  

        if self.current_objective.action.type == ActionType.FIRE_TORPEDO:
            if not self.current_objective.action.fired:
                # self.node.fire_torpedo()
                self.node.get_logger().info('Launching torpedo no 1!')
                self.node.publish_servo_cmd(ServoEnum.TORPEDO_ID, ServoEnum.TORPEDO_R_PWM)  
                self.node.publish_servo_cmd(ServoEnum.TORPEDO_ID, ServoEnum.TORPEDO_INIT_PWM)
        if self.current_objective.action.type == ActionType.FORWARD:
            error_ekf_fwd_position = self.current_objective.action.forward_distance_m - self.forward_position
            self.node.publish_forward_ekf_error(error_ekf_fwd_position)

    def spin_search(self):
        cmd = self.current_objective.search.spin_pwm
        self.node.publish_yaw_cmd(cmd)
        
    def get_vision_action_for_current_objective(self) -> VisionAction:
        if self.current_objective is None:
            return VisionAction.IDLE

        if self.current_objective.action.type == ActionType.CIRCLE_MARKER:
            return VisionAction.CIRCLE_MARKER

        return VisionAction.IDLE

    def is_current_action_done(self) -> bool:
        if self.current_objective is None:
            return True

        action = self.current_objective.action.type

        if action == ActionType.NONE:
            return True

        if action == ActionType.FORWARD:
            #done = self.state_lifespan >= self.current_objective.action_duration
            return self.forward_position >= (self.current_objective.action.forward_distance_m - 0.4)
            
        if action == ActionType.CIRCLE_MARKER:
            if self.current_objective.action.camera_mean_depth_target_mm is None:
                return False

            return (
                self.mean_depth_forward_cam == self.current_objective.action.camera_mean_depth_target_mm
                and self.state_lifespan >= self.current_objective.action.min_lifespan_s
            )
        if action == ActionType.FIRE_TORPEDO:
            return self.state_lifespan > self.current_objective.action.min_lifespan_s
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

        px = target[DetectionIndex.CENTER_FOV_RATIO]
        return abs(px) < self.current_objective.center.center_tolerance_fov

    def is_target_approached(self) -> bool:
        if self.current_objective is None:
            return False

        target = self.detection_store.get_detection(self.target_ids)
        if target is None:
            return False

        depth = target[DetectionIndex.DEPTH_MM]
        return depth < self.current_objective.approach.approach_distance_mm

    def is_target_perpendicular(self) -> bool:
        if self.current_objective is None:
            return False

        target = self.detection_store.get_detection(self.target_ids)
        if target is None:
            return False

        alignement_error = target[DetectionIndex.ANGLE_DEG]
        return abs(alignement_error) < self.current_objective.center.alignement_tolerance
    
    def ekf_reset_done(self, event):
        if self.ekf_resetted:
            return True

        self.node.get_logger().info("Resetting EKF before leaving APPROACH_TARGET")
        self.ekf_resetted = reset_ekf_pose(self.node)

        return self.ekf_resetted

    def state_changed(self, event):
        self.state_start_time = time.monotonic()
        self.node.get_logger().info('\n')
        self.node.get_logger().info(f'Entered state {self.state}')
        self.node.get_logger().info(f'Transition: {event.transition.source} -> {event.transition.dest}, current state: {self.state}')

    @property
    def state_lifespan(self):
        return time.monotonic() - self.state_start_time
