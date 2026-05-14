#!/usr/bin/env python3

import time
import rclpy

from transitions import Machine
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Int16

from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID, GateLikeObjectID

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
            {'trigger': 'return_gate_to_approach_any', 'source': 'RETURN_GATE',     'dest': 'APPROACH_ANY',    'conditions': 'condition_return_to_approach_any'},
            {'trigger': 'approach_any_to_traverse',    'source': 'APPROACH_ANY',    'dest': 'TRAVERSE_GATE',   'conditions': 'condition_approach_any_to_traverse'}
        ]

        self.machine = Machine(model=self, states = self.states, initial = 'SEARCH', transitions = self.transitions, after_state_change = 'state_changed')

        self.target_gate_id = None
        self.target_object_id = None

        self.gate_objects = None
        self.objects = None

        self.mean_depth_forward_cam = None

        # Subscribers
        self.detection_sub = self.create_subscription(Float32MultiArray, '/yolo/obj_depth_dist', self.obj_detection_callback, 10)
        self.gate_detection_sub = self.create_subscription(Float32MultiArray, '/yolo/obj_angle', self.gate_detection_callback, 10)
        self.mean_depth_sub = self.create_subscription(Int16, '/yolo/mean_depth_forward_cam', self.mean_depth_callback, 10)

        # Publishers
        self.state_pub = self.create_publisher(Int8, '/mission/state', 10)
        self.target_gate_pub = self.create_publisher(Int8, '/mission/target_gate', 10)
        self.target_object_pub = self.create_publisher(Int8, '/mission/target_object', 10)
        self.forward_cmd_pub = self.create_publisher(Int16, '/control/cmd/forward', 10)
        self.depth_threshold_pub = self.create_publisher(Int16, '/yolo/depth_threshold', 10)

        # Timers
        self.timer_state_machine = self.create_timer(1/20, self.state_machine_timer)
        self.timer_state_sender = self.create_timer(1/20, self.state_targets_sender)

        self.target_object_id = [ObjectID.GATE_TOTAL]
        self.get_logger().info(f'Set target gate to : {self.target_object_id[0].name}')

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
            self.return_gate_to_approach_any()

        elif self.state == 'APPROACH_ANY':
            self.approach_any_to_traverse()

    # On enter or exit actions ----------------------------------------------------------------------

    def on_enter_CENTER_GATE(self):
        msg = Int16()
        msg.data = 7000
        self.depth_threshold_pub.publish(msg)

    def on_enter_APPROACH_GATE(self):
        self.target_object_id = [ObjectID.GATE_TOTAL]
        self.get_logger().info(f'Set target object to : {self.target_object_id[0].name}')

    def on_enter_TRAVERSE_GATE(self):
        msg = Int16()
        msg.data = 15000
        self.depth_threshold_pub.publish(msg)

    def on_enter_APPROACH_MARKER(self):
        self.target_object_id = [ObjectID.MARQUEUR]
        self.get_logger().info(f'Set target object to : {self.target_object_id[0].name}')

    def on_enter_CIRCLE_MARKER(self):
        self.target_gate_id = GateLikeObjectID.SLALOM_SIDE_MID

    def on_enter_APPROACH_ANY(self):
        self.target_object_id = [ObjectID.GATE_LEG, ObjectID.GATE_TOTAL, ObjectID.LUMIERE]
        self.get_logger().info(f'Set target object to : {self.target_object_id[0].name}, {self.target_object_id[1].name}, {self.target_object_id[2].name}')

    # State changement trigger conditions ------------------------------------------------------------
    
    def condition_search_to_center(self):
        return self.is_object_present()
    
    def condition_center_to_approach(self):
        return self.is_object_centered() and self.state_lifespan > 10.0
    
    def condition_approach_to_traverse(self):
        return not self.is_object_present()
    
    def condition_traverse_to_approach(self):
        return self.state_lifespan > 3.0

    def condition_approach_to_circle(self):
        return self.is_target_approached(5000)
    
    def condition_circle_to_return(self):
        return (self.is_gate_present() and self.mean_depth_forward_cam > 8200)
    
    def condition_return_to_approach_any(self):
        return self.state_lifespan > 5.0  
    
    def condition_approach_any_to_traverse(self):
        return self.is_target_approached(2000)

    # Detection methods -------------------------------------------------------------------------------

    def is_gate_present(self):
        target_gate = self.get_target_gate()
        if target_gate is not None:
            self.get_logger().info(f"Gate like object {self.target_gate_id.name} was found!")
            return True
        return False
    
    def is_object_present(self):
        target_object = self.get_target_object()
        if target_object is not None:
            self.get_logger().info(f"Object {self.target_object_id.name} was found!")
            return True
        return False

    def is_gate_centered(self):
        target_gate = self.get_target_gate()
        if target_gate is not None:
            if abs(target_gate[1]) < 3 and abs(target_gate[2]) < 15:
                self.get_logger().info(f"Gate like object {self.target_gate_id.name} is centered!")
                return True
        return False
    
    def is_object_centered(self):
        target_object = self.get_target_object()
        if target_object is not None:
            if abs(target_object[2]) < 15:
                self.get_logger().info(f"Object {self.target_object_id[0].name} is centered!")
                return True
        return False

    def is_target_approached(self, distance):
        if self.target_object_id is not None:
            target_object = self.get_target_object()
            if target_object is not None and target_object[1] < distance:
                self.get_logger().info(f"Target object : {self.target_object_id[0].name} is in range!")
                return True
        return False
    
    # Publisher loops or subscribers callbacks --------------------------------------------------------

    def state_targets_sender(self):
        msg = Int8()

        if self.state is not None:
            msg.data = RobotState[self.state].value
            self.state_pub.publish(msg)

        if self.target_gate_id is not None:
            msg.data = self.target_gate_id
            self.target_gate_pub.publish(msg)

        target_object = self.get_target_object()

        if target_object is not None:
            msg.data = int(target_object[0])
            self.target_object_pub.publish(msg)

    def obj_detection_callback(self, msg):
        data = msg.data
        self.objects = [data[i:i+3] for i in range(0, len(data), 3)]
        
    def gate_detection_callback(self, msg):
        data = msg.data
        self.gate_objects = [data[i:i+3] for i in range(0, len(data), 3)]

    def mean_depth_callback(self, msg):
            self.mean_depth_forward_cam = msg.data

    # YOLO_PIPELINE parser for target_id ------------------------------------------------------------

    def get_target_object(self):
        if self.target_object_id is not None and self.objects is not None:
            target_object = next((o for o in self.objects if ObjectID(o[0]) in self.target_object_id),None)
            return target_object
        else:
            return None

    def get_target_gate(self):
        if self.target_gate_id is not None and self.gate_objects is not None:
            target_gate = next((o for o in self.gate_objects if GateLikeObjectID(o[0]) == self.target_gate_id),None)
            return target_gate
        else:
            return None
    
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