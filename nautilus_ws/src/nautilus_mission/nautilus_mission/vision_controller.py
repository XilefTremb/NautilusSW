#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16


class AnglePidNode(Node):
    def __init__(self):
        super().__init__('angle_pid_node')

        # PID gains
        self.kp = 5.0
        self.ki = 0.1
        self.kd = 0.5

        # Desired angle
        self.setpoint = 0.0

        # Command limits
        self.cmd_center = 1500.0
        self.cmd_min = 1300.0
        self.cmd_max = 1700.0

        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        # Subscriber
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/id_depth_angle',
            self.angle_callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(
            Int16,
            '/control/cmd_img_yaw',
            10
        )

        self.get_logger().info('Angle PID node started.')

    def angle_callback(self, msg):
        data = msg.data
        objects = [data[i:i+3] for i in range(0, len(data), 3)]
        current_angle = None
        for obj in objects:
            if int(obj[0]) == 1:
                current_angle = obj[2]
                now = self.get_clock().now()

        if current_angle is not None:
            if self.prev_time is None:
                self.prev_time = now
                return

            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt <= 0.0:
                return

            # Error
            error = self.setpoint - current_angle

            # Integral
            self.integral += error * dt

            # Derivative
            derivative = (error - self.prev_error) / dt

            # PID output
            output = (
                self.kp * error +
                self.ki * self.integral +
                self.kd * derivative
            )

            # Convert PID output to command around 1500
            cmd = self.cmd_center - output

            # Clamp command
            cmd = int(max(self.cmd_min, min(self.cmd_max, cmd)))

            # Publish
            cmd_msg = Int16()
            cmd_msg.data = cmd
            self.pub.publish(cmd_msg)

            self.get_logger().info(
                f'angle={current_angle:.2f}, error={error:.2f}, cmd={cmd}'
            )

            # Save state
            self.prev_error = error
            self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = AnglePidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()