#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Int16

from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID, GateLikeObjectID
from nautilus_mission.state import State


class StateMachine(Node):
    def __init__(self):

        super().__init__('state_machine')

        self._state = State()

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
        self.timer_state_machine = self.create_timer(1/10, self.state_machine)
        self.timer_state_sender = self.create_timer(1/10, self.state_targets_sender)

        # Initial state
        self.state = RobotState.SEARCH

        self.target_gate_id = GateLikeObjectID.GATE_LEFT_MID
        self.get_logger().info(f'Set target gate to : {self.target_gate_id.name}')

    def state_machine(self):
        if self.state == RobotState.SEARCH:
            if self.is_gate_present():
                self.state = RobotState.CENTER_GATE

        elif self.state == RobotState.CENTER_GATE:
            if self.is_gate_centered() and self.state.lifespan > 10.0:
                self.state = RobotState.APPROACH_GATE
                self.target_object_id = [ObjectID.REQUIN]
                self.get_logger().info(f'Set target object to : {self.target_object_id[0].name}')

        elif self.state == RobotState.APPROACH_GATE:
            if self.is_target_approached(1500):
                self.state = RobotState.TRAVERSE_GATE
                msg = Int16()
                msg.data = 15000
                self.depth_threshold_pub.publish(msg)

        elif self.state == RobotState.TRAVERSE_GATE:
            forward_msg = Int16()
            forward_msg.data = 1900
            self.forward_cmd_pub.publish(forward_msg)

            if self.state.lifespan > 10.0:
                self.target_object_id = [ObjectID.GATE_LEG_L]
                self.get_logger().info(f'Set target object to : {self.target_object_id[0].name}')

                if self.is_target_approached(5000):
                    msg = Int16()
                    msg.data = 5000
                    self.depth_threshold_pub.publish(msg)
                    self.state = RobotState.CIRCLE_MARKER

        elif self.state == RobotState.CIRCLE_MARKER:
            if self.state.lifespan > 10.0 and self.mean_depth_forward_cam >= 20000:
                msg = Int16()
                msg.data = 15000
                self.depth_threshold_pub.publish(msg)
                self.state = RobotState.RETURN_GATE

        elif self.state == RobotState.RETURN_GATE:
            forward_msg = Int16()
            forward_msg.data = 1900
            self.forward_cmd_pub.publish(forward_msg)

            if self.state.lifespan > 5.0:
                self.target_object_id = [ObjectID.REQUIN, ObjectID.POISSON]
                self.get_logger().info(f'Set target object to : {self.target_object_id[0].name}')
                self.state = RobotState.APPROACH_ANY

        elif self.state == RobotState.APPROACH_ANY:
            if self.is_target_approached(2000):
                msg = Int16()
                msg.data = 5000
                self.depth_threshold_pub.publish(msg)
                self.state = RobotState.CENTER_GATE

    def state_targets_sender(self):
        msg = Int8()

        if self.state._state is not None:
            msg.data = self.state.value
            self.state_pub.publish(msg)

        if self.target_gate_id is not None:
            msg.data = self.target_gate_id
            self.target_gate_pub.publish(msg)

        target_object = self.get_target_object()

        if target_object is not None:
            msg.data = int(target_object[0])
            self.target_object_pub.publish(msg)
    
    def mean_depth_callback(self, msg):
            self.mean_depth_forward_cam = msg.data

    def obj_detection_callback(self, msg):
        data = msg.data
        self.objects = [data[i:i+3] for i in range(0, len(data), 3)]
        
    def gate_detection_callback(self, msg):
        data = msg.data
        self.gate_objects = [data[i:i+3] for i in range(0, len(data), 3)]

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

    def is_gate_present(self):
        target_gate = self.get_target_gate()
        if target_gate is not None:
            self.get_logger().info(f"Gate like object {self.target_gate_id.name} was found!")
            return True
        return False

    def is_gate_centered(self):
        target_gate = self.get_target_gate()
        if target_gate is not None:
            if abs(target_gate[1]) < 3 and abs(target_gate[2]) < 15:
                self.get_logger().info(f"Gate like object {self.target_gate_id.name} is centered!")
                return True
        return False

    def is_target_approached(self, distance):
        if self.target_object_id is not None:
            target_object = self.get_target_object()
            if target_object is not None and target_object[1] < distance:
                self.get_logger().info(f"Target object : {self.target_object_id[0].name} is in range!")
                return True
        return False

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        self._state.set(new_state)
        self.get_logger().info(f'Set state to : {new_state.name}')


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()