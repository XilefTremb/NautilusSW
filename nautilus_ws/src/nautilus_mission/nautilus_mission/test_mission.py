#!/usr/bin/env python3

import argparse
import time
from math import pi
from nautilus_bringup.auv_pymavlink import AuvPymavlink
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node

def parse_args():
    p = argparse.ArgumentParser()
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sitl", action="store_true", help="Run against SITL (keep sim GPS, don't start DVL aiding)")
    mode.add_argument("--auv", action="store_true", help="Run against real vehicle (enable DVL aiding + ExternalNav fusion)")
    p.add_argument("--endpoint", default="udpin:localhost:14550", help="MAVLink endpoint (udpin:... or udpout:...)")
    p.add_argument("--dvl-rate", type=float, default=10.0, help="VISION_POSITION_DELTA rate (Hz) in --auv mode")
    p.add_argument("--ros-args",action="store_true")
    return p.parse_args()

class Master(Node):
    def __init__(self, args):

        self.auv = AuvPymavlink()

        self.auv.Connect(args.endpoint, start_receiver=False)

        profile = self.auv.SITL_PROFILE if args.sitl else self.auv.AUV_PROFILE
        print(f"Applying {'SITL' if args.sitl else 'AUV'} parameter profile...")
        self.auv.ApplyParamProfile(profile)

        self.auv.StartReceiver()

        self.mission()

    def mission(self):
        self.auv.Arm()
        self.auv.ChangeMode('GUIDED')
        self.auv.GoToWaypointLocal(2, 0, 0, 0)
        # try:
        #     # while True:
        #     #     time.sleep(0.1)
        #     #     self.auv.GoToWaypointLocal(1, 1, 1, 0)
        #     #     self.auv.GoToWaypointLocal(0, 0, 0.5, pi/2)
        # except KeyboardInterrupt:
        #     print("Stopping...")
        #     self.auv.StopReceiver()

def main():
    args = parse_args()
    rclpy.init()

    master = Master(args)
    rclpy.spin(master)
    master.destroy_node()
    rclpy.shutdown()
    
   
if __name__ == "__main__":
    main()
