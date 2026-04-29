#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Float32, Int16

from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID, GateLikeObjectID


class VisionControllerNode(Node):
    def __init__(self):
        super().__init__('vision_controller_node')

        self.current_object_detection_callback = self.empty_callback
        self.current_gate_detection_callback = self.empty_callback

        self.state = None
        self.previous_state = None
        self.objects = None
        self.gate_objects = None

        self.target_gate_id = None
        self.previous_target_gate_id = None
        self.target_object_id = None
        self.previous_target_object_id = None

        self.last_target_gate_detection = None
        self.last_target_object_detection = None

        # Subscribers
        self.state_sub = self.create_subscription(Int8, '/mission/state', self.state_callback, 10)
        self.target_gate_sub = self.create_subscription(Int8, '/mission/target_gate', self.target_gate_callback, 10)
        self.target_object_sub = self.create_subscription(Int8, '/mission/target_object', self.target_object_callback, 10)

        self.obj_detection_sub = self.create_subscription(Float32MultiArray, '/yolo/obj_depth_dist', self.obj_detection_wrapper, 10)
        self.gate_detection_sub = self.create_subscription(Float32MultiArray, '/yolo/obj_angle', self.gate_detection_wrapper, 10)

        # Publishers
        self.yaw_error_pub = self.create_publisher(Float32, '/control/vision_errors/yaw', 10)
        self.forward_error_pub = self.create_publisher(Float32, '/control/vision_errors/forward', 10)
        self.lateral_error_pub = self.create_publisher(Float32, '/control/vision_errors/lateral', 10)
        self.forward_cmd_pub = self.create_publisher(Int16, '/control/cmd/forward', 10)
        self.lateral_cmd_pub = self.create_publisher(Int16, '/control/cmd/lateral', 10)

        self.get_logger().info('Vision controller node started.')

    def state_callback(self, msg):
        self.state = RobotState(msg.data)

        if self.previous_state != self.state:
            self.current_gate_detection_callback = self.empty_callback
            self.current_object_detection_callback = self.empty_callback

            if self.previous_state is not None:
                self.get_logger().info(f'Set state from {self.previous_state.name} to : {self.state.name}')
            else:
                self.get_logger().info(f'Set state from {self.previous_state} to : {self.state.name}')

        self.previous_state = self.state

        # Reacts to state ----------------------------------------------------------------------------------

        if self.state == RobotState.CENTER_GATE:
            self.current_gate_detection_callback = self.center_gate_callback

        elif self.state == RobotState.APPROACH_GATE:
            self.current_gate_detection_callback = self.approach_gate_callback

        elif self.state == RobotState.TRAVERSE_GATE:
            self.current_gate_detection_callback = self.empty_callback

        elif self.state == RobotState.CIRCLE_MARKER:
            self.current_object_detection_callback = self.circle_marker_callback

        elif self.state == RobotState.RETURN_GATE:
            self.current_object_detection_callback = self.empty_callback

        elif self.state == RobotState.APPROACH_ANY:
            self.current_object_detection_callback = self.approach_object_callback
        
    def target_gate_callback(self, msg):
        self.target_gate_id = GateLikeObjectID(msg.data)
        if self.previous_target_gate_id != self.target_gate_id:
            if self.previous_target_gate_id is not None:
                self.get_logger().info(f'Set target gate from : {self.previous_target_gate_id.name} to : {self.target_gate_id.name}')
            else:
                self.get_logger().info(f'Set target gate from : {self.previous_target_gate_id} to : {self.target_gate_id.name}')

        self.previous_target_gate_id = self.target_gate_id

    def target_object_callback(self, msg):
        self.target_object_id = ObjectID(msg.data)
        if self.previous_target_object_id != self.target_object_id:
            if self.previous_target_object_id is not None:
                self.get_logger().info(f'Set target object from : {self.previous_target_object_id.name} to : {self.target_object_id.name}')
            else:
                self.get_logger().info(f'Set target object from : {self.previous_target_object_id} to : {self.target_object_id.name}')

        self.previous_target_object_id = self.target_object_id

    def center_gate_callback(self, msg):
        if self.last_target_gate_detection is None:
            return

        forward_error, lateral_error = self.split_angle(
            self.last_target_gate_detection[1]
        )

        forward_msg = Float32()
        forward_msg.data = forward_error
        self.forward_error_pub.publish(forward_msg)

        lateral_msg = Float32()
        lateral_msg.data = lateral_error
        self.lateral_error_pub.publish(lateral_msg)

        yaw_msg = Float32()
        yaw_msg.data = self.last_target_gate_detection[2]
        self.yaw_error_pub.publish(yaw_msg)

    def approach_gate_callback(self, msg):
        if self.last_target_gate_detection is not None:
            yaw_msg = Float32()
            yaw_msg.data = self.last_target_gate_detection[2]
            self.yaw_error_pub.publish(yaw_msg)

        forward_msg = Int16()
        forward_msg.data = 1600
        self.forward_cmd_pub.publish(forward_msg)

    def approach_object_callback(self, msg):
        if self.last_target_object_detection is not None:
            yaw_msg = Float32()
            yaw_msg.data = self.last_target_object_detection[2]
            self.yaw_error_pub.publish(yaw_msg)

        forward_msg = Int16()
        forward_msg.data = 1600
        self.forward_cmd_pub.publish(forward_msg)

    def circle_marker_callback(self, msg):
        if self.last_target_object_detection is None:
            return

        yaw_msg = Float32()
        yaw_msg.data = self.last_target_object_detection[2] - 80
        self.yaw_error_pub.publish(yaw_msg)

        forward_msg = Int16()
        forward_msg.data = 1545
        self.forward_cmd_pub.publish(forward_msg)

        lateral_msg = Int16()
        lateral_msg.data = 1455
        self.lateral_cmd_pub.publish(lateral_msg)

    def empty_callback(self, msg):
        pass

    def obj_detection_wrapper(self, msg):
        self.objects = [msg.data[i:i + 3] for i in range(0, len(msg.data), 3)]

        if self.target_object_id is not None:
            self.last_target_object_detection = next((obj for obj in self.objects if ObjectID(obj[0]) == self.target_object_id),None)

        self.current_object_detection_callback(msg)

    def gate_detection_wrapper(self, msg):
        self.gate_objects = [
            msg.data[i:i + 3] for i in range(0, len(msg.data), 3)
        ]

        if self.target_gate_id is not None:
            self.last_target_gate_detection = next((obj for obj in self.gate_objects if GateLikeObjectID(obj[0]) == self.target_gate_id),None)

        self.current_gate_detection_callback(msg)

    def split_angle(self, angle_deg):
        angle = math.radians(angle_deg)

        forward_error = -angle_deg * math.sin(angle)
        lateral_error = angle_deg * math.cos(angle)

        return forward_error, lateral_error


def main(args=None):
    rclpy.init(args=args)
    node = VisionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()