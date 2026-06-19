#!/usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
from std_srvs.srv import Trigger

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
        self.cmd_min = 1400.0
        self.cmd_max = 1600.0

        # PID state
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.reset_requested = False

        # Subscriber
        self.sub = self.create_subscription(Float32, self.input_topic, self.error_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Int16, self.output_topic, 10)

        # Service
        self.reset_pid = self.create_service(Trigger, '~/reset_pid', self.reset_pid_callback)

        self.get_logger().info('Vision PID node started.')

    def error_callback(self, msg):
        error = msg.data
        now = self.get_clock().now()

        if error is None:
            return

        # Reset requested by service
        if self.reset_requested:
            self.integral = 0.0
            self.prev_error = None
            self.prev_time = None
            self.reset_requested = False
            return

        # First valid sample after startup/reset
        if self.prev_time is None or self.prev_error is None:
            self.prev_time = now
            self.prev_error = error
            return

        dt = (now - self.prev_time).nanoseconds / 1e9

        use_i_d = True

        if dt <= 0.0 or dt > 1.0:
            use_i_d = False
            dt = 0.0

        if use_i_d:
            self.integral += error * dt
            raw_derivative = (error - self.prev_error) / dt
        else:
            raw_derivative = 0.0
            self.get_logger().info("PID node stalled for too long ; P preserved, I and D ignored")

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * raw_derivative
        )

        output *= self.output_sign

        cmd = self.cmd_center + output
        cmd = int(max(self.cmd_min, min(self.cmd_max, cmd)))

        cmd_msg = Int16()
        cmd_msg.data = cmd
        self.pub.publish(cmd_msg)

        self.prev_error = error
        self.prev_time = now

    # Reset integrator between completed steps
    def reset_pid_callback(self, request, response):
        self.reset_requested = True
        response.success = True
        response.message = "PID integrator reset!"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionPidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()