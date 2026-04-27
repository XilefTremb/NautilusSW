#!/usr/bin/env python3

import argparse
import time
import math
from nautilus_controls.auv_pymavlink import AuvPymavlink
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Int16, Int8, Float32
from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID
import numpy as np


def parse_args():
    p = argparse.ArgumentParser()
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sitl", action="store_true", help="Run against SITL (keep sim GPS, don't start DVL aiding)")
    mode.add_argument("--auv", action="store_true", help="Run against real vehicle (enable DVL aiding + ExternalNav fusion)")
    p.add_argument("--endpoint", default="udpin:localhost:14551", help="MAVLink endpoint (udpin:... or udpout:...)")
    p.add_argument("--ros-args",action="store_true")
    return p.parse_args()

class CubeInterface(Node):
    def __init__(self, args):

        super().__init__('CubeInterface')

        self.cmd_timeout_s = 0.5
        
        self.last_yaw_cmd = None
        self.last_yaw_cmd_time = None
        self.last_lateral_cmd = None
        self.last_lateral_cmd_time = None
        self.last_forward_cmd = None
        self.last_forward_cmd_time = None

        self.Startup(args)

        self.yaw_cmd_sub = self.create_subscription(
            Int16,
            '/control/cmd/yaw',
            self.yaw_cmd_callback,
            10)
        
        self.lateral_cmd_sub = self.create_subscription(
            Int16,
            '/control/cmd/lateral',
            self.lateral_cmd_callback,
            10)
        
        self.forward_cmd_sub = self.create_subscription(
            Int16,
            '/control/cmd/forward',
            self.forward_cmd_callback,
            10)
        
        self.vision_lateral_error_sub = self.create_subscription(
            Int8,
            '/mission/state',
            self.state_callback,
            10)
        
        self.create_timer(1.0/40.0, self.timer_callback)

        # self.get_logger().info("Stopping...")
        
        # self.auv.StopReceiver()

    def Startup(self, args):
        # --------------------------------------------
        # Runs at very beginning of node instanciation

        self.auv = AuvPymavlink(self)
        self.auv.Connect(args.endpoint, start_receiver=False)

        profile = self.auv.SITL_PROFILE if args.sitl else self.auv.AUV_PROFILE
        self.get_logger().info(f"Applying {'SITL' if args.sitl else 'AUV'} parameter profile...")
        self.auv.ApplyParamProfile(profile)

        self.auv.StartReceiver()

    def state_callback(self, msg):
        self.state = RobotState(msg.data)

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
        yaw_active = self.is_fresh(self.last_yaw_cmd_time)
        lateral_active = self.is_fresh(self.last_lateral_cmd_time)
        forward_active = self.is_fresh(self.last_forward_cmd_time)

        yaw_cmd = None
        lateral_cmd = None
        forward_cmd = None
        
        # Do nothing if neither topic has published recently
        if not yaw_active and not lateral_active and not forward_active:
            self.get_logger().info('no fresh cmd')
            return

        if yaw_active:
            yaw_cmd = int(self.last_yaw_cmd)

        if forward_active:
            forward_cmd = int(self.last_forward_cmd)

        if lateral_active:
            lateral_cmd = int(self.last_lateral_cmd)

        self.auv.SendRCOverride(
            forward=forward_cmd,
            lateral=lateral_cmd,
            yaw=yaw_cmd,
        )
        self.get_logger().info(f"sent cmd yaw : {yaw_cmd}, forward: {forward_cmd}, lateral : {lateral_cmd}")

def main():
    args = parse_args()
    rclpy.init()
    master = CubeInterface(args)
    rclpy.spin(master)
    master.destroy_node()
    rclpy.shutdown()
    
   
if __name__ == "__main__":
    main()
