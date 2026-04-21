#!/usr/bin/env python3

import argparse
import time
from math import pi
from nautilus_controls.auv_pymavlink import AuvPymavlink
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import Int16

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

        self.Startup(args)

        self.StateReader()

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

    def StateReader(self):
        # ----------------------------------------------------------------------
        # Should be a ros2 callback that is based on the state topics reception?

        state = 1 # Read state that is coming from the state machine

        match state:
            case 1:
                self.Dive()

            case 2:
                self.Rotate()

            case 3:
                self.CenterGate()
        
            case 4:
                # Not implemented yet, TO BE DONE
                self.get_logger().info("Entered Go Forward state")

    def Dive(self):
        # -------------------------------
        # Dive the vehicle to wanted depth

        self.get_logger().info("Entered Dive state")
        time.sleep(0.1) # To be completed         

    def Rotate(self):
        # -------------------------------
        # Rotate the vehicle undefinitely

        self.get_logger().info("Entered Rotating state")
        time.sleep(0.1) # To be completed

    def CenterGate(self):
        # ------------------------------------------------
        # Logic for CenterGate state called by StateReader

        self.get_logger().info("Entered Center gate state")

        self.latest_cmd = 1500.0
        self.cmd_received = False
        self.should_send_cmd = True

        self.sub = self.create_subscription(
            Int16,
            '/control/cmd_img_yaw',
            self.cmd_callback,
            10
        )

        self.timer = self.create_timer((1/40), self.control_loop)

    def cmd_callback(self, msg):
        self.latest_cmd = msg.data
        self.cmd_received = True

    def control_loop(self):
        if not self.cmd_received:
            self.get_logger().info('No cmd received yet')
            return

        if self.should_send_cmd:
            cmd = self.latest_cmd
            self.get_logger().info(f'Sending cmd: {cmd}')
            self.auv.SendRCOverride(yaw = cmd)        

def main():
    args = parse_args()
    rclpy.init()
    master = CubeInterface(args)
    rclpy.spin(master)
    master.destroy_node()
    rclpy.shutdown()
    
   
if __name__ == "__main__":
    main()
