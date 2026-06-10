#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Int8
from nautilus_controls.auv_pymavlink import AuvPymavlink
from nautilus_interfaces.srv import SetTargetDepth
import time


def parse_args():
    parser = argparse.ArgumentParser()

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sitl", action="store_true", help="Run against SITL")
    mode.add_argument("--auv", action="store_true", help="Run against real vehicle")

    parser.add_argument(
        "--endpoint",
        default="udpin:localhost:14551",
        help="MAVLink endpoint",
    )

    parser.add_argument("--ros-args", action="store_true")

    return parser.parse_args()


class CubeInterface(Node):
    def __init__(self, args):
        super().__init__('cube_interface')

        self.cmd_timeout_s = 0.5

        self.last_yaw_cmd = None
        self.last_yaw_cmd_time = None

        self.last_lateral_cmd = None
        self.last_lateral_cmd_time = None

        self.last_forward_cmd = None
        self.last_forward_cmd_time = None

        self.startup(args)

        self.subs = {
            'yaw_cmd': self.create_subscription(
                Int16, '/control/cmd/yaw', self.yaw_cmd_callback, 10
            ),
            'lateral_cmd': self.create_subscription(
                Int16, '/control/cmd/lateral', self.lateral_cmd_callback, 10
            ),
            'forward_cmd': self.create_subscription(
                Int16, '/control/cmd/forward', self.forward_cmd_callback, 10
            ),
        }

        self.depth_service = self.create_service(
            SetTargetDepth,
            '/mission/set_target_depth',
            self.set_target_depth_callback
        )

        self.timer = self.create_timer(1.0 / 200.0, self.timer_callback)

    def startup(self, args):
        self.auv = AuvPymavlink(self)
        self.auv.connect(args.endpoint, start_receiver=False)

        profile = self.auv.sitl_profile if args.sitl else self.auv.auv_profile

        profile_name = 'SITL' if args.sitl else 'AUV'
        self.get_logger().info(f'Applying {profile_name} parameter profile...')

        self.auv.apply_param_profile(profile)
        self.auv.start_receiver()

        # time.sleep(2)
        self.auv.change_mode("ALT_HOLD")

        self.get_logger().info("Testing servo 11...")
        
        self.auv.set_servo(11, 1100)
        time.sleep(2.0)
        self.auv.set_servo(11, 1900)
        # self.auv.go_to_depth(-0.67)

    def yaw_cmd_callback(self, msg):
        self.last_yaw_cmd = msg.data
        self.last_yaw_cmd_time = self.get_clock().now()

    def lateral_cmd_callback(self, msg):
        self.last_lateral_cmd = msg.data
        self.last_lateral_cmd_time = self.get_clock().now()

    def forward_cmd_callback(self, msg):
        self.last_forward_cmd = msg.data
        self.last_forward_cmd_time = self.get_clock().now()

    def is_fresh(self, last_time):
        if last_time is None:
            return False

        age_s = (self.get_clock().now() - last_time).nanoseconds * 1e-9
        return age_s < self.cmd_timeout_s

    def timer_callback(self):
        yaw_cmd = int(self.last_yaw_cmd) if self.is_fresh(self.last_yaw_cmd_time) else None
        lateral_cmd = int(self.last_lateral_cmd) if self.is_fresh(self.last_lateral_cmd_time) else None
        forward_fresh = self.is_fresh(self.last_forward_cmd_time) 
        self.get_logger().info(f"{forward_fresh}")
        if forward_fresh:
            forward_cmd = int(self.last_forward_cmd)
        else:
            forward_cmd = None

        if yaw_cmd is None and lateral_cmd is None and forward_cmd is None:
            self.get_logger().info('No fresh command')
            return

        self.auv.send_rc_override(
            forward=forward_cmd,
            lateral=lateral_cmd,
            yaw=yaw_cmd,
        )

        self.get_logger().info(
            f'Sent cmd yaw: {yaw_cmd}, forward: {forward_cmd}, lateral: {lateral_cmd}'
        )

    def set_target_depth_callback(self, request, response):

        depth = request.depth_m

        try:
            # Your MAVLink code goes here
            self.auv.go_to_depth(depth)

            self.get_logger().info(
                f"Received new target depth: {depth:.2f} m"
            )

            response.success = True
            response.message = f"Target depth set to {depth:.2f} m"

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response


def main():
    args = parse_args()

    rclpy.init()
    node = CubeInterface(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()