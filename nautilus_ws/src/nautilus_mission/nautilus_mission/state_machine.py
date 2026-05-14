#!/usr/bin/env python3

import time
import rclpy

from transitions import Machine
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Int16

from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID

class StateMachine(Node):
    def __init__(self):

        super().__init__('state_machine')

        self.state_start_time = time.monotonic()

        self.states = ['SEARCH','CENTER_GATE','APPROACH_GATE','TRAVERSE_GATE','APPROACH_MARKER','CIRCLE_MARKER','RETURN_GATE','APPROACH_ANY']

        self.transitions = [
            {'trigger': 'search_to_center',            'source': 'SEARCH',          'dest': 'CENTER_GATE',     'conditions': 'condition_search_to_center'},
            {'trigger': 'center_to_approach',          'source': 'CENTER_GATE',     'dest': 'APPROACH_GATE',   'conditions': 'condition_center_to_approach'},
            {'trigger': 'approach_to_traverse',        'source': 'APPROACH_GATE',   'dest': 'TRAVERSE_GATE',   'conditions': 'condition_approach_to_traverse'},
            {'trigger': 'traverse_to_approach',        'source': 'TRAVERSE_GATE',   'dest': 'APPROACH_MARKER', 'conditions': 'condition_traverse_to_approach'},
            {'trigger': 'approach_to_circle',          'source': 'APPROACH_MARKER', 'dest': 'CIRCLE_MARKER',   'conditions': 'condition_approach_to_circle'},
            {'trigger': 'circle_to_return',            'source': 'CIRCLE_MARKER',   'dest': 'RETURN_GATE',     'conditions': 'condition_circle_to_return'},
            {'trigger': 'return_target_to_approach_any', 'source': 'RETURN_GATE',     'dest': 'APPROACH_ANY',    'conditions': 'condition_return_to_approach_any'},
            {'trigger': 'approach_any_to_traverse',    'source': 'APPROACH_ANY',    'dest': 'TRAVERSE_GATE',   'conditions': 'condition_approach_any_to_traverse'}
        ]

        self.machine = Machine(model=self, states = self.states, initial = 'SEARCH', transitions = self.transitions, after_state_change = 'state_changed')

        self.target_ids = None

        self.detections = []

        self.mean_depth_forward_cam = None

        # Subscribers
        self.detection_sub = self.create_subscription(Float32MultiArray, '/yolo/detections', self.detection_callback, 10)
        self.mean_depth_sub = self.create_subscription(Int16, '/yolo/mean_depth_forward_cam', self.mean_depth_callback, 10)

        # Publishers
        self.state_pub = self.create_publisher(Int8, '/mission/state', 10)
        self.target_detection_pub = self.create_publisher(Int8, '/mission/target_detection', 10)
        self.forward_cmd_pub = self.create_publisher(Int16, '/control/cmd/forward', 10)
        self.depth_threshold_pub = self.create_publisher(Int16, '/yolo/depth_threshold', 10)

        # Timers
        self.timer_state_machine = self.create_timer(1/20, self.state_machine_timer)
        self.timer_state_sender = self.create_timer(1/20, self.state_targets_sender)

        self.target_ids = [ObjectID.GATE_TOTAL]
        self.get_logger().info(f'Set target gate to : {self.target_ids[0].name}')

    # State machine loop ---------------------------------------------------------------------------

    def state_machine_timer(self):
        if self.state == 'SEARCH':
            self.search_to_center()

        elif self.state == 'CENTER_GATE':
            self.center_to_approach()

        elif self.state == 'APPROACH_GATE':
            self.approach_to_traverse()

        elif self.state == 'TRAVERSE_GATE':
            forward_msg = Int16()
            forward_msg.data = 1700
            self.forward_cmd_pub.publish(forward_msg)
            self.traverse_to_circle()

        elif self.state == 'CIRCLE_MARKER':
            self.circle_to_return()

        elif self.state == 'RETURN_GATE':
            self.return_target_to_approach_any()

        elif self.state == 'APPROACH_ANY':
            self.approach_any_to_traverse()

    # On enter or exit actions ----------------------------------------------------------------------

    def on_enter_CENTER_GATE(self):
        msg = Int16()
        msg.data = 7000
        self.depth_threshold_pub.publish(msg)

    def on_enter_APPROACH_GATE(self):
        self.target_ids = [ObjectID.GATE_TOTAL]
        self.get_logger().info(f'Set target object to : {self.target_ids[0].name}')

    def on_enter_TRAVERSE_GATE(self):
        msg = Int16()
        msg.data = 15000
        self.depth_threshold_pub.publish(msg)

    def on_enter_APPROACH_MARKER(self):
        self.target_ids = [ObjectID.MARQUEUR]
        self.get_logger().info(f'Set target object to : {self.target_ids[0].name}')

    def on_enter_CIRCLE_MARKER(self):
        self.target_target_id = ObjectID.SLALOM_SIDE_MID

    def on_enter_APPROACH_ANY(self):
        self.target_ids = [ObjectID.GATE_LEG, ObjectID.GATE_TOTAL, ObjectID.LUMIERE]
        self.get_logger().info(f'Set target object to : {self.target_ids[0].name}, {self.target_ids[1].name}, {self.target_ids[2].name}')

    # State changement trigger conditions ------------------------------------------------------------
    
    def condition_search_to_center(self):
        return self.is_target_present()
    
    def condition_center_to_approach(self):
        return self.is_target_centered() and self.state_lifespan > 10.0
    
    def condition_approach_to_traverse(self):
        return not self.is_target_present()
    
    def condition_traverse_to_approach(self):
        return self.state_lifespan > 3.0

    def condition_approach_to_circle(self):
        return self.is_target_approached(5000)
    
    def condition_circle_to_return(self):
        return (self.is_target_present() and self.mean_depth_forward_cam > 8200)
    
    def condition_return_to_approach_any(self):
        return self.state_lifespan > 5.0  
    
    def condition_approach_any_to_traverse(self):
        return self.is_target_approached(2000)

    # Detection methods -------------------------------------------------------------------------------
    
    def is_target_present(self):
        target = self.get_target_detection()

        if target is not None:
            self.get_logger().info(f'Target ID {ObjectID(int(target[0])).name} was found!')
            return True

        return False

    def is_target_centered(self):
        target = self.get_target_detection()

        if target is None:
            return False

        px = target[1]

        if abs(px) < 50:
            self.get_logger().info(f'Target ID {ObjectID(int(target[0])).name} is centered!')
            return True

        return False
    
    def is_target_perpendicular(self):
        target = self.get_target_detection()

        if target is None:
            return False

        angle = target[2]

        if abs(angle) < 15:
            self.get_logger().info(f'Target ID {ObjectID(int(target[0])).name} is perpendicular!')
            return True

        return False

    def is_target_approached(self, distance):
        target = self.get_target_detection()

        if target is None:
            return False

        depth = target[3]

        if depth < distance:
            self.get_logger().info(
                f'Target ID {ObjectID(int(target[0])).name} is in range! Depth: {depth}'
            )
            return True

        return False
    
    # Publisher loops or subscribers callbacks --------------------------------------------------------

    def state_targets_sender(self):
        if self.state is not None:
            state = eval(f"RobotState.{self.state}")
            msg = Int8()
            msg.data = state.value
            self.state_pub.publish(msg)

        target = self.get_target_detection()

        if target is not None:
            msg = Float32MultiArray()
            msg.data = [
                float(target[0]),  # id
                float(target[1]),  # px
                float(target[2]),  # angle
                float(target[3]),  # depth
            ]
            self.target_detection_pub.publish(msg)

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


    def get_target_detection(self):
        if self.target_ids is None:
            return None

        return next(
            (
                detection for detection in self.detections
                if int(detection[0]) in self.target_ids
            ),
            None
        )
    
    # Necessary for state lifespan --------------------------------------------------------------------

    def state_changed(self):
        self.state_start_time = time.monotonic()
        self.get_logger().info(f"Entered state {self.state}")

    @property
    def state_lifespan(self):
        return time.monotonic() - self.state_start_time


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()