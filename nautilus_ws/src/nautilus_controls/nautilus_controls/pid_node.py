#!/usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from std_msgs.msg import Float32, Int16


class VisionPidNode(Node):
    def __init__(self):
        super().__init__('vision_pid_node')

     
        self.declare_parameter('input_topic', Parameter.Type.STRING)
        self.declare_parameter('output_topic', Parameter.Type.STRING)
        self.declare_parameter('kp', Parameter.Type.DOUBLE)
        self.declare_parameter('ki', Parameter.Type.DOUBLE)
        self.declare_parameter('kd', Parameter.Type.DOUBLE)
        self.declare_parameter('flip_output', False)

        # --- Get parameters ---
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        if self.get_parameter('flip_output').value:
            self.output_sign = -1
        else:
            self.output_sign = 1

        # Command limits
        self.cmd_center = 1500.0
        self.cmd_min = 1300.0
        self.cmd_max = 1700.0

        # PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

        self.derivative_alpha = 0.2
        self.filtered_derivative = 0.0

        # Subscriber
        self.sub = self.create_subscription(
            Float32,
            self.input_topic,
            self.error_callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(
            Int16,
            self.output_topic,
            10
        )

        self.get_logger().info('Vision PID node started.')

    def error_callback(self, msg):
        error = msg.data
        # objects = [data[i:i+3] for i in range(0, len(data), 3)]
        # error = None
        # for obj in objects:
        #     if int(obj[0]) == 1:
        #         error = obj[2]
        now = self.get_clock().now()

        if error is not None:
            if self.prev_time is None:
                self.prev_time = now
                return

            dt = (now - self.prev_time).nanoseconds / 1e9
            if dt <= 0.0:
                return

            # Integral
            self.integral += error * dt

            # Raw derivative
            raw_derivative = (error - self.prev_error) / dt

            #Low-pass filtered derivative
            self.filtered_derivative = (
               self.derivative_alpha * raw_derivative
               + (1.0 - self.derivative_alpha) * self.filtered_derivative
            )


            # PID output
            output = (
                self.kp * error +
                self.ki * self.integral +
                self.kd * self.filtered_derivative
            )

            output *= self.output_sign

            # Convert PID output to command around 1500
            cmd = self.cmd_center + output

            # Clamp command
            cmd = int(max(self.cmd_min, min(self.cmd_max, cmd)))

            # Publish
            cmd_msg = Int16()
            cmd_msg.data = cmd
            self.pub.publish(cmd_msg)

            # Save state
            self.prev_error = error
            self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = VisionPidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()