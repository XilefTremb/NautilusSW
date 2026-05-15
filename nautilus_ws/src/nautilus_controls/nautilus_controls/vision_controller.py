#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Float32, Int16

from nautilus_bringup.VisionAction import VisionAction


class VisionControllerNode(Node):
    def __init__(self):
        super().__init__('vision_controller_node')

        self.vision_action = None
        self.previous_vision_action = None

        self.current_detection_callback = self.empty_callback

        # Selected detection coming from vision_action_machine:
        # [id, px, angle, depth]
        self.last_target_detection = None

        # Subscribers
        self.vision_action_sub = self.create_subscription(Int8, '/mission/vision_action', self.vision_action_callback, 10)
        self.target_detection_sub = self.create_subscription(Float32MultiArray, '/mission/target_detection', self.target_detection_callback, 10)

        # Publishers
        self.yaw_error_pub = self.create_publisher(Float32, '/control/vision_errors/yaw', 10)
        self.forward_error_pub = self.create_publisher(Float32, '/control/vision_errors/forward', 10)
        self.lateral_error_pub = self.create_publisher(Float32, '/control/vision_errors/lateral', 10)

        self.forward_cmd_pub = self.create_publisher(Int16, '/control/cmd/forward', 10)
        self.lateral_cmd_pub = self.create_publisher(Int16, '/control/cmd/lateral', 10)

        self.get_logger().info('Vision controller node started.')

    def vision_action_callback(self, msg):
        self.vision_action = VisionAction(msg.data)

        if self.previous_vision_action != self.vision_action:
            self.current_detection_callback = self.empty_callback

            if self.previous_vision_action is not None:
                self.get_logger().info(
                    f'Set vision_action from {self.previous_vision_action.name} to: {self.vision_action.name}'
                )
            else:
                self.get_logger().info(
                    f'Set vision_action from {self.previous_vision_action} to: {self.vision_action.name}'
                )

            self.select_vision_action_callback()

        self.previous_vision_action = self.vision_action

    def select_action_callback(self):
        if self.vision_action == VisionAction.CENTER_TARGET:
            self.current_detection_callback = self.center_target_callback

        elif self.vision_action == VisionAction.APPROACH_TARGET:
            self.current_detection_callback = self.approach_target_callback

        elif self.vision_action == VisionAction.CIRCLE_MARKER:
            self.current_detection_callback = self.circle_marker_callback

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

    def center_target_callback(self):
        if self.last_target_detection is None:
            return

        _, px, depth, angle = self.last_target_detection

        forward_error, lateral_error = self.split_angle(angle)
        fwd_msg = Float32()
        fwd_msg.data = forward_error

        lat_msg = Float32()
        lat_msg.data = lateral_error

        yaw_msg = Float32()
        yaw_msg.data = float(px)

        self.forward_error_pub(fwd_msg)
        self.lateral_error_pub(lat_msg)
        self.yaw_error_pub.publish(yaw_msg)

    def approach_target_callback(self):
        if self.last_target_detection is not None:
            _, px, depth, angle = self.last_target_detection

            yaw_msg = Float32()
            yaw_msg.data = float(px)
            self.yaw_error_pub.publish(yaw_msg)

        forward_msg = Int16()
        forward_msg.data = 1600
        self.forward_cmd_pub.publish(forward_msg)

    def circle_marker_callback(self):
        if self.last_target_detection is None:
            return

        _, px, depth, angle = self.last_target_detection

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