#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray


class JoyServoNode(Node):
    def __init__(self):
        super().__init__('joy_servo_node')

        self.servo_number = 11
        self.servo_pwm_left = 1100
        self.servo_pwm_right = 1900

        self.dpad_lr_axis = 7
        self.last_dpad_lr = 0.0

        self.servo_pub = self.create_publisher(
            Int16MultiArray,
            '/control/cmd/servo',
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):
        if len(msg.axes) <= self.dpad_lr_axis:
            return

        dpad_lr = msg.axes[self.dpad_lr_axis]

        if dpad_lr > 0.5 and self.last_dpad_lr <= 0.5:
            self.send_servo_cmd(self.servo_pwm_right)

        elif dpad_lr < -0.5 and self.last_dpad_lr >= -0.5:
            self.send_servo_cmd(self.servo_pwm_left)

        self.last_dpad_lr = dpad_lr

    def send_servo_cmd(self, pwm: int):
        msg = Int16MultiArray()
        msg.data = [self.servo_number, pwm]
        self.servo_pub.publish(msg)

        self.get_logger().info(
            f'Published servo command: servo={self.servo_number}, pwm={pwm}'
        )


def main():
    rclpy.init()
    node = JoyServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()