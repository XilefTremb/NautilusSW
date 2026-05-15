#!/usr/bin/env python3

from dataclasses import dataclass
from enum import Enum, auto
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Int16
from transitions import Machine

from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID
from nautilus_bringup.VisionAction import VisionAction


class ActionType(Enum):
    NONE = auto()
    FORWARD = auto()
    CIRCLE_MARKER = auto()


@dataclass
class Objective:
    name: str
    target_ids: list[ObjectID]

    center_tolerance_px: float = 50.0
    approach_distance: float = 5000.0
    angle_tolerance_deg: float = 15.0

    # Optional YOLO depth threshold command sent when objective starts
    depth_threshold: Optional[int] = None

    # Action executed after SEARCH -> CENTER -> APPROACH succeeds
    action_type: ActionType = ActionType.NONE
    action_duration: float = 0.0
    action_forward_pwm: int = 1500

    # Optional completion condition for actions like CIRCLE_MARKER
    mean_depth_target: Optional[int] = None
    min_action_lifespan: float = 0.0


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        # ------------------------------------------------------------------------------------------
        # Mission objectives
        # ------------------------------------------------------------------------------------------
        self.objectives: list[Objective] = [
            Objective(
                name='gate',
                target_ids=[ObjectID.GATE_LEFT_MID],
                center_tolerance_px=50.0,
                approach_distance=3000.0,
                depth_threshold=7000,
                action_type=ActionType.FORWARD,
                action_duration=3.0,
                action_forward_pwm=1700,
            ),
            Objective(
                name='marker',
                target_ids=[ObjectID.GATE_LEG_L],
                center_tolerance_px=50.0,
                approach_distance=3000.0,
                depth_threshold=15000,
                action_type=ActionType.CIRCLE_MARKER,
                mean_depth_target=20000,
                min_action_lifespan=10.0,
            ),
           Objective(
                name='return_gate_area',
                target_ids=[ObjectID.GATE_LEG_L, ObjectID.REQUIN, ObjectID.POISSON],
                center_tolerance_px=80.0,
                approach_distance=6000.0,
                action_type=ActionType.NONE,
            ),
            Objective(
                name='return_home',
                target_ids=[ObjectID.GATE_LEFT_MID],
                center_tolerance_px=50.0,
                approach_distance=2000.0,
                action_type=ActionType.FORWARD,
                action_duration=3.0,
                action_forward_pwm=1700,
            ),  
        ]

        self.objective_index = 0
        self.current_objective: Optional[Objective] = None

        # ------------------------------------------------------------------------------------------
        # Internal data
        # ------------------------------------------------------------------------------------------
        self.state_start_time = time.monotonic()
        self.detections: list[list[float]] = []
        self.mean_depth_forward_cam: Optional[int] = None

        self.target_ids: Optional[list[ObjectID]] = None
        self.vision_action = VisionAction.IDLE

        # ------------------------------------------------------------------------------------------
        # Generic behavior FSM
        # ------------------------------------------------------------------------------------------
        self.states = [
            'IDLE',
            'LOAD_OBJECTIVE',
            'SEARCH_TARGET',
            'CENTER_TARGET',
            'APPROACH_TARGET',
            'EXECUTE_ACTION',
            'MISSION_COMPLETE',
        ]

        self.transitions = [
            {'trigger': 'start_mission', 'source': 'IDLE', 'dest': 'LOAD_OBJECTIVE'},
            {'trigger': 'objective_loaded', 'source': 'LOAD_OBJECTIVE', 'dest': 'SEARCH_TARGET'},
            {'trigger': 'no_more_objectives', 'source': 'LOAD_OBJECTIVE', 'dest': 'MISSION_COMPLETE'},

            {'trigger': 'target_found', 'source': 'SEARCH_TARGET', 'dest': 'CENTER_TARGET'},
            {'trigger': 'target_lost', 'source': ['CENTER_TARGET', 'APPROACH_TARGET'], 'dest': 'SEARCH_TARGET'},
            {'trigger': 'target_centered_event', 'source': 'CENTER_TARGET', 'dest': 'APPROACH_TARGET'},
            {'trigger': 'target_reached', 'source': 'APPROACH_TARGET', 'dest': 'EXECUTE_ACTION'},

            {'trigger': 'action_done', 'source': 'EXECUTE_ACTION', 'dest': 'LOAD_OBJECTIVE'},
            {'trigger': 'finish_mission', 'source': '*', 'dest': 'MISSION_COMPLETE'},
        ]

        self.machine = Machine(
            model=self,
            states=self.states,
            initial='IDLE',
            transitions=self.transitions,
            after_state_change='state_changed',
            ignore_invalid_triggers=True,
        )

        # ------------------------------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------------------------------
        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/detections',
            self.detection_callback,
            10,
        )

        self.mean_depth_sub = self.create_subscription(
            Int16,
            '/yolo/mean_depth_forward_cam',
            self.mean_depth_callback,
            10,
        )

        # ------------------------------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------------------------------
        self.state_pub = self.create_publisher(Int8, '/mission/state', 10)
        self.vision_action_pub = self.create_publisher(Int8, '/mission/vision_action', 10)
        self.target_detection_pub = self.create_publisher(Float32MultiArray, '/mission/target_detection', 10)
        self.forward_cmd_pub = self.create_publisher(Int16, '/control/cmd/forward', 10)
        self.depth_threshold_pub = self.create_publisher(Int16, '/yolo/depth_threshold', 10)

        # ------------------------------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------------------------------
        self.timer_behavior = self.create_timer(1 / 20, self.behavior_timer)
        self.timer_sender = self.create_timer(1 / 20, self.action_targets_sender)

        self.start_mission()

    # ==============================================================================================
    # Generic behavior loop
    # ==============================================================================================

    def behavior_timer(self):
        if self.state == 'SEARCH_TARGET':
            self.vision_action = VisionAction.IDLE
            self.run_search_behavior()

            if self.is_target_present():
                self.target_found()

        elif self.state == 'CENTER_TARGET':
            self.vision_action = VisionAction.CENTER_TARGET
            self.run_center_behavior()

            if not self.is_target_present():
                self.target_lost()
            elif self.is_target_centered():
                self.target_centered_event()

        elif self.state == 'APPROACH_TARGET':
            self.vision_action = VisionAction.APPROACH_TARGET
            self.run_approach_behavior()

            if not self.is_target_present():
                self.target_lost()
            elif self.is_target_approached():
                self.target_reached()

        elif self.state == 'EXECUTE_ACTION':
            self.vision_action = self.get_vision_action_for_current_action()
            self.run_current_action()

            if self.is_current_action_done():
                self.objective_index += 1
                self.action_done()

        else:
            self.vision_action = VisionAction.IDLE

    # ==============================================================================================
    # State entry actions
    # ==============================================================================================

    def on_enter_LOAD_OBJECTIVE(self):
        self.vision_action = VisionAction.IDLE
        self.publish_forward_cmd(1500)

        if self.objective_index >= len(self.objectives):
            self.no_more_objectives()
            return

        self.current_objective = self.objectives[self.objective_index]
        self.target_ids = self.current_objective.target_ids

        self.get_logger().info(
            f'Loaded objective {self.objective_index + 1}/{len(self.objectives)}: '
            f'{self.current_objective.name}'
        )

        self.get_logger().info(
            'Target IDs: ' + ', '.join(target.name for target in self.target_ids)
        )

        if self.current_objective.depth_threshold is not None:
            self.publish_depth_threshold(self.current_objective.depth_threshold)

        self.objective_loaded()

    def on_enter_EXECUTE_ACTION(self):
        if self.current_objective is None:
            self.finish_mission()
            return

        self.get_logger().info(
            f'Executing action {self.current_objective.action_type.name} '
            f'for objective {self.current_objective.name}'
        )

    def on_enter_MISSION_COMPLETE(self):
        self.target_ids = None
        self.vision_action = VisionAction.IDLE
        self.publish_forward_cmd(1500)
        self.get_logger().info('Mission complete')

    # ==============================================================================================
    # Generic state behaviors
    # ==============================================================================================

    def run_search_behavior(self):
        """
        Search behavior placeholder.
        Add yaw scan / sweep commands here if needed.
        """
        pass

    def run_center_behavior(self):
        """
        Centering is handled by vision_controller using /mission/target_detection
        and /mission/vision_action.
        """
        pass

    def run_approach_behavior(self):
        """
        Approach is handled by vision_controller using /mission/target_detection
        and /mission/vision_action.
        """
        pass

    def run_current_action(self):
        if self.current_objective is None:
            return

        action = self.current_objective.action_type

        if action == ActionType.FORWARD:
            self.publish_forward_cmd(self.current_objective.action_forward_pwm)

        elif action == ActionType.CIRCLE_MARKER:
            # Actual circling behavior is delegated to the vision_controller
            # through VisionAction.CIRCLE_MARKER.
            pass

    def get_vision_action_for_current_action(self) -> VisionAction:
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
            return self.state_lifespan >= self.current_objective.action_duration

        if action == ActionType.CIRCLE_MARKER:
            if self.current_objective.mean_depth_target is None:
                return False

            return (
                self.mean_depth_forward_cam == self.current_objective.mean_depth_target
                and self.state_lifespan >= self.current_objective.min_action_lifespan
            )

        return False

    # ==============================================================================================
    # Detection methods
    # ==============================================================================================

    def is_target_present(self) -> bool:
        target = self.get_detection(self.target_ids)

        if target is not None:
            self.get_logger().info(f'Target ID {ObjectID(int(target[0])).name} was found!')
            return True

        return False

    def is_id_present(self, ids: int | list[int]) -> bool:
        target = self.get_detection(ids)

        if target is not None:
            self.get_logger().info(f'Target ID {ObjectID(int(target[0])).name} was found!')
            return True

        return False

    def is_target_centered(self) -> bool:
        if self.current_objective is None:
            return False

        target = self.get_detection(self.target_ids)

        if target is None:
            return False

        px = target[1]

        if abs(px) < self.current_objective.center_tolerance_px:
            self.get_logger().info(f'Target ID {ObjectID(int(target[0])).name} is centered!')
            return True

        return False

    def is_target_approached(self) -> bool:
        if self.current_objective is None:
            return False

        target = self.get_detection(self.target_ids)

        if target is None:
            return False

        depth = target[3]

        if depth < self.current_objective.approach_distance:
            self.get_logger().info(
                f'Target ID {ObjectID(int(target[0])).name} is in range! Depth: {depth}'
            )
            return True

        return False

    def is_target_perpendicular(self) -> bool:
        target = self.get_detection(self.target_ids)

        if target is None:
            return False

        angle = target[2]

        if abs(angle) < self.current_objective.angle_tolerence_deg:
            self.get_logger().info(f'Target ID {ObjectID(int(target[0])).name} is perpendicular!')
            return True

        return False

    def get_detection(self, ids):
        if ids is None:
            return None

        if isinstance(ids, int):
            ids = [ids]

        ids_as_int = [int(id_) for id_ in ids]

        return next(
            (
                detection for detection in self.detections
                if int(detection[0]) in ids_as_int
            ),
            None,
        )

    # ==============================================================================================
    # Publishers/subscribers
    # ==============================================================================================

    def action_targets_sender(self):
        self.publish_state()
        self.publish_vision_action()
        self.publish_target_detection()

    def publish_state(self):
        if self.state is None:
            return

        if not hasattr(RobotState, self.state):
            return

        msg = Int8()
        msg.data = getattr(RobotState, self.state).value
        self.state_pub.publish(msg)

    def publish_vision_action(self):
        if self.vision_action is None:
            return

        msg = Int8()
        msg.data = self.vision_action.value
        self.vision_action_pub.publish(msg)

    def publish_target_detection(self):
        target = self.get_detection(self.target_ids)

        if target is None:
            return

        msg = Float32MultiArray()
        msg.data = [
            float(target[0]),  # id
            float(target[1]),  # px
            float(target[2]),  # angle
            float(target[3]),  # depth
        ]
        self.target_detection_pub.publish(msg)

    def publish_forward_cmd(self, pwm: int):
        msg = Int16()
        msg.data = pwm
        self.forward_cmd_pub.publish(msg)

    def publish_depth_threshold(self, threshold: int):
        msg = Int16()
        msg.data = threshold
        self.depth_threshold_pub.publish(msg)

    def detection_callback(self, msg):
        data = msg.data

        if len(data) % 4 != 0:
            self.get_logger().warn(
                f'Received malformed detection array of length {len(data)}. Expected multiple of 4.'
            )
            return

        self.detections = [
            data[i:i + 4]
            for i in range(0, len(data), 4)
        ]

    def mean_depth_callback(self, msg):
        self.mean_depth_forward_cam = msg.data

    # ==============================================================================================
    # State timing
    # ==============================================================================================

    def state_changed(self):
        self.state_start_time = time.monotonic()
        self.get_logger().info(f'Entered state {self.state}')

    @property
    def state_lifespan(self):
        return time.monotonic() - self.state_start_time


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
