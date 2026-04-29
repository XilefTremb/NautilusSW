#!/usr/bin/env python3

import argparse
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Int16
from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID, GateLikeObjectID


class StateMachine(Node):
    def __init__(self):

        super().__init__('state_machine')

        self.state = None
        self.target_gate_id = None
        self.target_object_id = None

        self.gate_objects = None
        self.objects = None

        self.last_target_gate_detection = None
        self.last_target_object_detection = None

        # Subscribers
        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/obj_depth_dist',
            self.obj_detection_callback,
            10
        )

        self.gate_detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/obj_angle',
            self.gate_detection_callback,
            10
        )

        # Publishers
        self.state_pub = self.create_publisher(Int8, '/mission/state', 10)
        self.target_gate_pub = self.create_publisher(Int8, '/mission/target_gate', 10)
        self.target_object_pub = self.create_publisher(Int8, '/mission/target_object', 10)
        self.forward_cmd_pub = self.create_publisher(Int16, '/control/cmd/forward', 10)
        self.depth_threshold_pub = self.create_publisher(Int16, '/yolo/depth_threshold', 10)

        # Timers
        self.timer_state_machine = self.create_timer(1/10, self.state_machine)
        self.timer_forward_cmd = self.create_timer(1/10, self.forward_cmd_pub_callback)
        self.timer_state_sender = self.create_timer(1, self.state_targets_sender)

        # Initial state
        self.state = RobotState.SEARCH
        self.get_logger().info(f'Set state to : {self.state}')

        self.target_gate_id = GateLikeObjectID.GATE_LEFT_MID
        self.get_logger().info(f'Set target gate to : {self.target_gate_id.name}')

    def state_machine(self):
        if self.state == RobotState.SEARCH:
            if self.is_gate_present():
                self.state = RobotState.CENTER_GATE
                self.get_logger().info(f'Set state to : {self.state}')
            else:
                self.get_logger().info('Target gate is not in view')

        elif self.state == RobotState.CENTER_GATE:
            if self.is_gate_centered():
                self.state = RobotState.APPROACH_GATE
                self.get_logger().info(f'Set state to : {self.state}')
                self.target_object_id = ObjectID.REQUIN
                self.get_logger().info(f'Set target object to : {self.target_object_id.name}')

        elif self.state == RobotState.APPROACH_GATE:
            if self.is_target_approached(1500):
                self.state = RobotState.TRAVERSE_GATE
                self.get_logger().info(f'Set state to : {self.state}')
                self.target_object_id = ObjectID.GATE_LEG_L
                self.get_logger().info(f'Set target object to : {self.target_object_id.name}')

                msg = Int16()
                msg.data = 15000
                self.depth_threshold_pub.publish(msg)

        elif self.state == RobotState.TRAVERSE_GATE:
            if self.is_target_approached(2000):
                msg = Int16()
                msg.data = 5000
                self.depth_threshold_pub.publish(msg)
                self.state = RobotState.CIRCLE_MARKER

        elif self.state == RobotState.CIRCLE_MARKER:
            pass

    def state_targets_sender(self):
        msg = Int8()

        if self.state is not None:
            msg.data = self.state
            self.state_pub.publish(msg)

        if self.target_gate_id is not None:
            msg.data = self.target_gate_id
            self.target_gate_pub.publish(msg)

        if self.target_object_id is not None:
            msg.data = self.target_object_id
            self.target_object_pub.publish(msg)

    def forward_cmd_pub_callback(self):
        if self.state == RobotState.TRAVERSE_GATE:
            msg = Int16()
            msg.data = 1600
            self.forward_cmd_pub.publish(msg)

    def obj_detection_callback(self, msg):
        data = msg.data
        self.objects = [data[i:i+3] for i in range(0, len(data), 3)]

        if self.target_object_id is not None:
            self.last_target_object_detection = next(
                (o for o in self.objects if ObjectID(o[0]) == self.target_object_id),
                None
            )

    def gate_detection_callback(self, msg):
        data = msg.data
        self.gate_objects = [data[i:i+3] for i in range(0, len(data), 3)]

        if self.target_gate_id is not None:
            self.last_target_gate_detection = next(
                (o for o in self.gate_objects if GateLikeObjectID(o[0]) == self.target_gate_id),
                None
            )

    def is_gate_present(self):
        if self.last_target_gate_detection is not None:
            self.get_logger().info(f"Gate like object {self.target_gate_id.name} was found!")
            return True
        return False

    def is_gate_centered(self):
        if self.last_target_gate_detection is not None:
            if abs(self.last_target_gate_detection[1]) < 5 and abs(self.last_target_gate_detection[2]) < 30:
                self.get_logger().info(f"Gate like object {self.target_gate_id.name} is centered!")
                return True
        return False

    def is_target_approached(self, distance):
        if self.last_target_object_detection is not None:
            if self.last_target_object_detection[1] < distance:
                self.get_logger().info(f"Target object : {self.target_object_id.name} is in range!")
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()