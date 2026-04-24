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

        self.cmd_timeout_s = 2.0
        
        self.last_yaw_cmd = None
        self.last_yaw_cmd_time = None
        self.last_lateral_cmd = None
        self.last_lateral_cmd_time = None
        self.last_yaw_error = None

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
        
        self.vision_yaw_error_sub = self.create_subscription(
            Float32,
            '/control/vision_errors/yaw',
            self.yaw_error_callback,
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

    def yaw_cmd_callback(self, msg):
        self.last_yaw_cmd = msg.data
        self.last_yaw_cmd_time = self.get_clock().now()

    def lateral_cmd_callback(self, msg):
        self.last_lateral_cmd = msg.data
        self.last_lateral_cmd_time = self.get_clock().now()

    def yaw_error_callback(self, msg):
        self.last_yaw_error = msg.data
    
    def is_fresh(self, last_time):
        if last_time is None:
            return False

        age_s = (self.get_clock().now() - last_time).nanoseconds * 1e-9
        return age_s < self.cmd_timeout_s
    
    def timer_callback(self):
        yaw_active = self.is_fresh(self.last_yaw_cmd_time)
        lateral_active = self.is_fresh(self.last_lateral_cmd_time)

        yaw_cmd = None
        lateral_cmd = None
        forward_cmd = None
        right_cmd = None
        
        # Do nothing if neither topic has published recently
        if not yaw_active and not lateral_active:
            self.get_logger().info('no fresh cmd')
            return

        if yaw_active:
            yaw_cmd = int(self.last_yaw_cmd)

        if lateral_active:
            lateral_cmd = int(self.last_lateral_cmd)
            forward_cmd, right_cmd = self.split_pwm_by_angle(lateral_cmd, self.last_yaw_error) 
            
        self.auv.SendRCOverride(
            forward=forward_cmd,
            lateral=right_cmd,
            yaw=yaw_cmd,
        )
        self.get_logger().info(f"sent cmd yaw : {yaw_cmd}, forward: {forward_cmd}, lateral : {right_cmd}")

    
    def clamp_pwm(self, x):
        return max(1100, min(1900, x))

    def split_pwm_by_angle(self, pwm, angle_deg):
        # Convert PWM to signed command
        magnitude = pwm - 1500   # range: -400 to +400

        angle = math.radians(angle_deg)

        forward_offset = magnitude * math.sin(angle)
        lateral_offset = magnitude * math.cos(angle)

        forward_pwm = int(1500 + forward_offset)
        lateral_pwm = int(1500 + lateral_offset)

        forward_pwm = self.clamp_pwm(forward_pwm)
        lateral_pwm = self.clamp_pwm(lateral_pwm)

        return forward_pwm, lateral_pwm

def main():
    args = parse_args()
    rclpy.init()
    master = CubeInterface(args)
    rclpy.spin(master)
    master.destroy_node()
    rclpy.shutdown()
    
   
if __name__ == "__main__":
    main()
