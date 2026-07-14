#!/usr/bin/env python3

import sys
import time
from typing import Optional

from rclpy.node import Node
from transitions import Machine

from enums.ObjectID import ObjectID
from enums.VisionAction import VisionAction
from enums.DetectionIndex import DetectionIndex
from enums.ServoEnum import ServoEnum
from enums.InferenceMode import InferenceMode

from .detection_store import DetectionStore
from nautilus_services import request_depth_change, reset_pids, reset_ekf_pose
from nautilus_mission.search_patterns import search_bottom_spiral, forward_search

class StateMachine:
    """Mission decision logic only. No ROS subscriptions and no vision error calculation."""

    def __init__(self, node: Node, detection_store: DetectionStore, use_sim: bool, mission_offset_timer: float = 0.0):
        self.node = node
        self.detection_store = detection_store
        self.mission_offset_timer = mission_offset_timer

        if use_sim:
            from .mission_objectives_sim import mission_list, Objective, ActionType, role_choice
        else:
            from .mission_objectives import mission_list, Objective, ActionType, role_choice

        self.ActionType = ActionType
        self.Objective = Objective

        self.objectives : list[Objective] = mission_list
        self.role_choice = role_choice

        if self.role_choice == ObjectID.SOS_SAFETY:
            self.dropper_choice = ObjectID.BLOOD
            self.second_dropper_choice = ObjectID.FIRE
        else:
            self.dropper_choice = ObjectID.FIRE
            self.second_dropper_choice = ObjectID.BLOOD

        self.objective_index = 0
        self.current_objective: Optional[Objective] = None
        self.target_ids: Optional[list[ObjectID]] = None
        self.vision_action = VisionAction.IDLE
        self.mean_depth_forward_cam: Optional[int] = None
        self.forward_position = 0.0
        self.lateral_position = 0.0
        self.ekf_resetted = False
        self.forward_action_ready = False
        self.forward_reset_threshold_m = 0.1
        self.approach_distance_error_m = 0.0

        self.current_success_frame_count = 0
        self.target_missing_count = 0
        self.target_missing_limit = 100
        self.state_start_time = time.monotonic()
        self.execute_action_start_time = None

        self.current_objective_lifetime_s = 0.0
        self.current_objective_start_time_s = time.monotonic()
        self.objective_timeout_s = 2.5 * 60

        self.bottom_search_leg = 0
        self.bottom_search_leg_start = 0.0

        # Last inference mode published to /yolo/inference_mode. Kept so we only
        # republish when the active objective actually requests a different mode.
        self.last_inference_mode: Optional[InferenceMode] = None
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
            {'trigger': 'no_target_to_be_reached', 'source': '*', 'dest': 'EXECUTE_ACTION', 'conditions': 'ekf_reset_done'},
            {'trigger': 'action_done', 'source': 'EXECUTE_ACTION', 'dest': 'LOAD_OBJECTIVE'},
            {'trigger': 'finish_mission', 'source': '*', 'dest': 'MISSION_COMPLETE'},
            {'trigger': 'skip_to_next_objective', 'source': '*', 'dest': 'LOAD_OBJECTIVE', 'after': 'increment_objective_index'},
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

        self.current_objective_lifetime_s = time.monotonic() - self.current_objective_start_time_s
        if self.current_objective_lifetime_s > self.objective_timeout_s and self.state != 'MISSION_COMPLETE':
            self.node.get_logger().warn(f"Objective {self.current_objective.name} timed out after {self.current_objective_lifetime_s:.1f}s. Skipping to next objective.")
            self.skip_to_next_objective()

        elif self.target_ids is None and self.state == 'SEARCH_TARGET':
            self.no_target_to_be_reached()

        elif self.state == 'SEARCH_TARGET':
            self.search()
            self.vision_action = VisionAction.IDLE
            # self.node.get_logger().info(f"{self.is_target_present()}")
            if self.is_target_present():
                self.target_found()

        elif self.state == 'CENTER_TARGET':
            if self.current_objective.center.full_centering:
                self.vision_action = VisionAction.CENTER_TARGET
            elif self.current_objective.center.center_bottom:
                self.vision_action = VisionAction.CENTER_BOTTOM
            else:
                self.vision_action = VisionAction.CENTER_FOV

            if self.is_target_lost_filtered():
                self.target_lost()
                return

            is_centered = self.is_target_centered()
            # self.node.get_logger().info(f"{self.current_objective.center.full_centering}")
            if self.current_objective.center.full_centering:
                success_condition = is_centered and self.is_target_perpendicular()
            else:
                success_condition = is_centered

            if self.update_success_frame_count(success_condition):
                self.target_centered_event()

        elif self.state == 'APPROACH_TARGET':
            if self.current_objective.action.type is self.ActionType.FORWARD:
                if self.current_objective.action.dropper_search and self.is_target_present(self.dropper_choice):
                    self.skip_to_next_objective()

            if self.current_objective.approach.approach_distance_mm is None:
                self.target_reached()
                return 
            else:
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
        self.node.publish_cmd("forward",1500)
        self.approach_distance_error_m = 0.0
        self.current_success_frame_count = 0
        self.ekf_resetted = False

    def on_enter_SEARCH_TARGET(self, event):
        self.bottom_search_leg = 0
        self.bottom_search_leg_start = self.node.get_clock().now()

    def load_current_objective(self, event):
        self.current_objective = self.objectives[self.objective_index]
        self.current_objective_start_time_s = time.monotonic()
        self.current_objective_lifetime_s = 0.0

        self.node.publish_servo_cmd(ServoEnum.TORPEDO_ID, ServoEnum.TORPEDO_INIT_PWM) 
        self.node.publish_servo_cmd(ServoEnum.DROPPER_ID, ServoEnum.DROPPER_INIT_PWM)

        if self.current_objective.action.type == self.ActionType.CHOOSE_GATE_SIDE:
            self.detection_store.save_role = False
            positions = self.detection_store.role_positions

            if positions is None:
                self.node.get_logger().warn('Role choice unavailable for chooseGateSide')
                self.target_ids = None
            else:
                
                # if positions[0] == self.role_choice :
                #     self.target_ids = [ObjectID.GATE_LEFT_MID]
                # else :
                #     self.target_ids = [ObjectID.GATE_MID_RIGHT]

                self.target_ids = [self.role_choice]

        elif 'slalom' in self.current_objective.name:
            positions = self.detection_store.role_positions
            if positions is None:
                self.node.get_logger().warn('Role choice unavailable for slalom')
                self.target_ids = None
            else:
                
                if positions[0] == self.role_choice :
                    self.target_ids = [ObjectID.SLALOM_LEFT_MID]
                else :
                    self.target_ids = [ObjectID.SLALOM_MID_RIGHT]

        elif self.current_objective.action.type is self.ActionType.LAUNCH_DROPPER :
            if self.current_objective.action.launch_second_dropper:
                self.target_ids = [self.second_dropper_choice]
            else:
                self.target_ids = [self.dropper_choice]

        elif self.current_objective.action.type == self.ActionType.FIRE_TORPEDO:

            if self.current_objective.name == "torpedoFiringPositioning1":
                self.target_ids = ([ObjectID.TARGET_BLOOD] if self.role_choice == ObjectID.SOS_SAFETY
                    else [ObjectID.TARGET_FIRE] )

            elif self.current_objective.name == "torpedoFiringPositioning2":
                self.target_ids = ([ObjectID.TARGET_AMBULANCE] if self.role_choice == ObjectID.SOS_SAFETY
                    else [ObjectID.TARGET_TRUCK])

            self.current_objective.action.fired = False

        elif self.current_objective.name == "traverseGate":
            if self.target_ids is None:
                self.node.get_logger().warn('Role choice unavailable for traverseGate')
                self.target_ids = None
        else:
            self.target_ids = self.current_objective.target_ids

        self.node.get_logger().info('\n')
        self.node.get_logger().info(f'Loaded objective {self.objective_index + 1}/{len(self.objectives)}: 'f'{self.current_objective.name}')

        if self.target_ids is not None:
            self.node.get_logger().info('Target IDs: ' + ', '.join(target.name for target in self.target_ids))
        else:
            self.node.get_logger().info('Target IDs: None')

        if self.current_objective.detections_depth_filter_mm is not None:
            self.node.publish_detections_depth_filter_mm(self.current_objective.detections_depth_filter_mm)

        # Publish the requested inference mode only when it changes between
        # objectives. The publisher is latched (transient local) so a late or
        # restarted YOLO node still gets the last value without periodic spam.
        objective_inference_mode = self.current_objective.inference_mode
        if objective_inference_mode != self.last_inference_mode:
            self.node.publish_inference_mode(objective_inference_mode)
            self.last_inference_mode = objective_inference_mode
            self.node.get_logger().info(f'Inference mode set to {InferenceMode(objective_inference_mode).name} for objective {self.current_objective.name}')

        if self.current_objective.target_auv_depth_m is not None:
            request_depth_change(self.node, self.current_objective.target_auv_depth_m)


    def increment_objective_index(self, event):
        self.objective_index+=1

    def on_enter_CENTER_TARGET(self, event):
        reset_pids(self.node)
        self.target_missing_count = 0

    def on_enter_APPROACH_TARGET(self, event):
        reset_pids(self.node)
        self.target_missing_count = 0
        self.ekf_resetted = False

    def on_enter_EXECUTE_ACTION(self, event):
        reset_pids(self.node)

        if self.current_objective is None:
            self.finish_mission()
            return
        
        if self.current_objective.action.type == self.ActionType.FORWARD:
            self.forward_action_ready = False
    
        self.execute_action_start_time = time.monotonic()

        self.node.get_logger().info(
            f'Executing action {self.current_objective.action.type.name} '
            f'for objective {self.current_objective.name}'
        )

    def on_enter_MISSION_COMPLETE(self, event):
        self.target_ids = None
        self.vision_action = VisionAction.IDLE
        self.node.publish_cmd("forward",1500)
        self.node.get_logger().info('Mission complete')

    def has_more_objectives(self, event):
        return self.objective_index < len(self.objectives)

    def run_current_action(self):
        if self.state != 'EXECUTE_ACTION' or self.current_objective is None:
            return

        if self.current_objective.action.type == self.ActionType.FIRE_TORPEDO:
            if not self.current_objective.action.fired:
                if self.current_objective.name == "torpedoFiringPositioning1":
                    self.node.get_logger().info('Launching torpedo no 1!')
                    self.node.publish_servo_cmd(ServoEnum.TORPEDO_ID, ServoEnum.TORPEDO_L_PWM)

                elif self.current_objective.name == "torpedoFiringPositioning2":
                    self.node.get_logger().info('Launching torpedo no 2!')
                    self.node.publish_servo_cmd(ServoEnum.TORPEDO_ID, ServoEnum.TORPEDO_R_PWM)

                self.current_objective.action.fired = True

        if self.current_objective.action.type == self.ActionType.FORWARD:
            error_ekf_fwd_position = self.current_objective.action.forward_distance_m - self.forward_position
            error_ekf_lat_position = self.current_objective.action.lateral_distance_m - self.lateral_position

            self.node.publish_error("forward_ekf",error_ekf_fwd_position)
            self.node.publish_error("lateral_ekf",error_ekf_lat_position)

        if self.current_objective.action.type == self.ActionType.SAVE_ROLE:
            self.node.get_logger().info('Saving role choice for current objective.')
            self.detection_store.save_role = True 

        if self.current_objective.action.type == self.ActionType.LAUNCH_DROPPER:
            self.node.get_logger().info('Launching droppers!')
            self.node.publish_servo_cmd(ServoEnum.DROPPER_ID, ServoEnum.DROPPER_2_PWM)  
     
        
    def search(self):
        if self.current_objective.center.center_bottom:
            search_bottom_spiral(self)
        else:
            forward_search(self)
 
    def get_vision_action_for_current_objective(self) -> VisionAction:
        if self.current_objective is None:
            return VisionAction.IDLE

        if self.current_objective.action.type == self.ActionType.CIRCLE_MARKER:
            return VisionAction.CIRCLE_MARKER

        return VisionAction.IDLE

    def is_current_action_done(self) -> bool:
        if self.current_objective is None:
            return True

        action = self.current_objective.action.type

        if action == self.ActionType.NONE or action == self.ActionType.CHOOSE_GATE_SIDE:
            return True

        if action == self.ActionType.FORWARD:
            #done = self.state_lifespan >= self.current_objective.action_duration            
            if not self.forward_action_ready:
                if abs(self.forward_position) < self.forward_reset_threshold_m:
                    self.forward_action_ready = True
                    self.node.get_logger().info(f'Forward action armed after EKF reset: x={self.forward_position:.3f}')
                else:
                    self.node.get_logger().info(f'Waiting for EKF odom reset before FORWARD: x={self.forward_position:.3f}')
                    return False
                
            if self.current_objective.action.dropper_search and self.is_target_present(self.dropper_choice):
                return True

            if self.current_objective.action.forward_distance_m >= 0:
                arrived_forward = self.forward_position >= self.current_objective.action.forward_distance_m - 0.3 - self.approach_distance_error_m
            else: 
                arrived_forward = self.forward_position <= self.current_objective.action.forward_distance_m + 0.3

            if self.current_objective.action.lateral_distance_m >= 0:
                arrived_lateral = self.lateral_position >= self.current_objective.action.lateral_distance_m - 0.3
            else:
                arrived_lateral = self.lateral_position <= self.current_objective.action.lateral_distance_m + 0.3 

            return arrived_forward and arrived_lateral
            
            
        if action == self.ActionType.CIRCLE_MARKER:
            if self.current_objective.action.camera_mean_depth_target_mm is None:
                return False

            return (
                self.mean_depth_forward_cam == self.current_objective.action.camera_mean_depth_target_mm
                and self.state_lifespan >= self.current_objective.action.min_lifespan_s
            )
        
        if action == self.ActionType.SAVE_ROLE:
            return self.state_lifespan > 2.0
        
        if action == self.ActionType.LAUNCH_DROPPER:
            return self.state_lifespan > self.current_objective.action.duration_s

        if action == self.ActionType.FIRE_TORPEDO:
            if not self.current_objective.action.fired:
                return False

            if self.state_lifespan <= self.current_objective.action.min_lifespan_s:
                return False

            self.node.publish_servo_cmd(
                ServoEnum.TORPEDO_ID,
                ServoEnum.TORPEDO_INIT_PWM
            )

            return True
        
        return False

    def is_target_present(self, ids=None) -> bool:
        ids = self.target_ids if ids is None else ids
        # self.node.get_logger().info(f"{ids}")
        return self.detection_store.get_detection(ids) is not None


    def is_target_lost_filtered(self, ids=None) -> bool:
        ids = self.target_ids if ids is None else ids

        if self.is_target_present(ids):
            self.target_missing_count = 0
            return False

        self.target_missing_count += 1
        return self.target_missing_count >= self.target_missing_limit


    def is_target_centered(self, ids=None) -> bool:
        ids = self.target_ids if ids is None else ids

        if self.current_objective is None:
            return False

        target = self.detection_store.get_detection(ids)
        if target is None:
            return False
        
        px = target[DetectionIndex.CENTER_FOV_RATIO_X]

        if self.current_objective.center.center_bottom:
            py = target[DetectionIndex.CENTER_FOV_RATIO_Y]
            
            error_x = abs(px - self.current_objective.center.target_offset_x) < self.current_objective.center.x_center_tolerance_fov
            error_y = abs(py - self.current_objective.center.target_offset_y) < self.current_objective.center.y_center_tolerance_fov
            
            return self.update_success_frame_count(error_x and error_y)
               
        else :
            return abs(px) < self.current_objective.center.x_center_tolerance_fov


    def is_target_approached(self, ids=None) -> bool:
        ids = self.target_ids if ids is None else ids

        if self.current_objective is None:
            return False

        target = self.detection_store.get_detection(ids)
        if target is None:
            return False

        depth = target[DetectionIndex.DEPTH_MM]
        self.approach_distance_error_m = (self.current_objective.approach.approach_distance_mm - depth) / 1000.0

        return depth < self.current_objective.approach.approach_distance_mm


    def is_target_perpendicular(self, ids=None) -> bool:
        ids = self.target_ids if ids is None else ids

        if self.current_objective is None:
            return False

        target = self.detection_store.get_detection(ids)
        if target is None:
            return False
        if self.current_objective.center.align_width:
            width = target[DetectionIndex.WIDTH]
            error = abs(width - self.current_objective.center.target_width_px)
            return  error < self.current_objective.center.width_tolerance_px

        else:
            alignement_error = target[DetectionIndex.ANGLE_DEG]
            return abs(alignement_error) < self.current_objective.center.alignement_tolerance
    
    
    def ekf_reset_done(self, event):
        if self.ekf_resetted:
            return True

        self.node.get_logger().info("Resetting EKF before leaving APPROACH_TARGET")
        self.ekf_resetted = reset_ekf_pose(self.node)
        return self.ekf_resetted
    
    # def center_lifespan_reached(self, event):
    #     if self.current_objective.center.full_centering is False:
    #         return (self.state_lifespan >= 0.5)
    #     else:
    #         return (self.state_lifespan >= 0.5)

    def state_changed(self, event):
        self.state_start_time = time.monotonic()
        self.node.get_logger().info('\n')
        self.node.get_logger().info(f'Entered state {self.state}')
        self.node.get_logger().info(f'Transition: {event.transition.source} -> {event.transition.dest}, current state: {self.state}')

    def update_success_frame_count(self, condition):
        # self.node.get_logger().info(f"{condition}")
        if condition:
            self.current_success_frame_count += 1
            self.node.get_logger().info(f"current success frame count {self.current_success_frame_count}")
        # else: 
            # self.current_success_frame_count = 0
            # self.node.get_logger().info("condition false")

        return self.current_success_frame_count >= self.current_objective.success_frame_treshold
       
    @property
    def state_lifespan(self):
        return time.monotonic() - self.state_start_time
