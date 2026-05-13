#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Float32, Int16

from nautilus_bringup.RobotState import RobotState


class VisionControllerNode(Node):
    def __init__(self):
        super().__init__('vision_controller_node')

        self.state = None
        self.previous_state = None

        self.current_detection_callback = self.empty_callback

        # Selected detection coming from state_machine:
        # [id, px, angle, depth]
        self.last_target_detection = None

        # Subscribers
        self.state_sub = self.create_subscription(
            Int8,
            '/mission/state',
            self.state_callback,
            10
        )

        self.target_detection_sub = self.create_subscription(
            Float32MultiArray,
            '/mission/target_detection',
            self.target_detection_callback,
            10
        )

        # Publishers
        self.yaw_error_pub = self.create_publisher(
            Float32,
            '/control/vision_errors/yaw',
            10
        )

        self.forward_error_pub = self.create_publisher(
            Float32,
            '/control/vision_errors/forward',
            10
        )

        self.lateral_error_pub = self.create_publisher(
            Float32,
            '/control/vision_errors/lateral',
            10
        )

        self.forward_cmd_pub = self.create_publisher(
            Int16,
            '/control/cmd/forward',
            10
        )

        self.lateral_cmd_pub = self.create_publisher(
            Int16,
            '/control/cmd/lateral',
            10
        )

        self.get_logger().info('Vision controller node started.')

    def state_callback(self, msg):
        self.state = RobotState(msg.data)

        if self.previous_state != self.state:
            self.current_detection_callback = self.empty_callback

            if self.previous_state is not None:
                self.get_logger().info(
                    f'Set state from {self.previous_state.name} to: {self.state.name}'
                )
            else:
                self.get_logger().info(
                    f'Set state from {self.previous_state} to: {self.state.name}'
                )

            self.select_state_callback()

        self.previous_state = self.state

    def select_state_callback(self):
        if self.state == RobotState.CENTER_GATE:
            self.current_detection_callback = self.center_gate_callback

        elif self.state == RobotState.APPROACH_GATE:
            self.current_detection_callback = self.approach_gate_callback

        elif self.state == RobotState.TRAVERSE_GATE:
            self.current_detection_callback = self.empty_callback

        elif self.state == RobotState.CIRCLE_MARKER:
            self.current_detection_callback = self.circle_marker_callback

        elif self.state == RobotState.RETURN_GATE:
            self.current_detection_callback = self.empty_callback

        elif self.state == RobotState.APPROACH_ANY:
            self.current_detection_callback = self.approach_object_callback

        else:
            self.current_detection_callback = self.empty_callback

    def target_detection_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(
                f'Received invalid target detection length {len(msg.data)}. Expected 4.'
            )
            return

        self.last_target_detection = msg.data

        self.current_detection_callback()

    def center_gate_callback(self):
        if self.last_target_detection is None:
            return

        _, px, angle, depth = self.last_target_detection

        yaw_msg = Float32()
        yaw_msg.data = float(angle)
        self.yaw_error_pub.publish(yaw_msg)

    def approach_gate_callback(self):
        if self.last_target_detection is not None:
            _, px, angle, depth = self.last_target_detection

            yaw_msg = Float32()
            yaw_msg.data = float(angle)
            self.yaw_error_pub.publish(yaw_msg)

        forward_msg = Int16()
        forward_msg.data = 1600
        self.forward_cmd_pub.publish(forward_msg)

    def approach_object_callback(self):
        if self.last_target_detection is not None:
            _, px, angle, depth = self.last_target_detection

            yaw_msg = Float32()
            yaw_msg.data = float(angle)
            self.yaw_error_pub.publish(yaw_msg)

        forward_msg = Int16()
        forward_msg.data = 1600
        self.forward_cmd_pub.publish(forward_msg)

    def circle_marker_callback(self):
        if self.last_target_detection is None:
            return

        _, px, angle, depth = self.last_target_detection

        yaw_msg = Float32()
        yaw_msg.data = float(px) - 320.0
        self.yaw_error_pub.publish(yaw_msg)

        forward_msg = Int16()
        forward_msg.data = 1540
        self.forward_cmd_pub.publish(forward_msg)

        lateral_msg = Int16()
        lateral_msg.data = 1375
        self.lateral_cmd_pub.publish(lateral_msg)

    def empty_callback(self):
        pass

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