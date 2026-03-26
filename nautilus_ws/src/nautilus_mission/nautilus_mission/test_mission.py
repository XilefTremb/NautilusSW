#!/usr/bin/env python3

import argparse
import time
from math import pi
from nautilus_mission.auv_pymavlink import AuvPymavlink
import rclpy
from rclpy.node import Node
from pymavlink import mavutil

def parse_args():
    p = argparse.ArgumentParser()
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sitl", action="store_true", help="Run against SITL (keep sim GPS, don't start DVL aiding)")
    mode.add_argument("--auv", action="store_true", help="Run against real vehicle (enable DVL aiding + ExternalNav fusion)")
    p.add_argument("--endpoint", default="udpin:localhost:14551", help="MAVLink endpoint (udpin:... or udpout:...)")
    p.add_argument("--dvl-rate", type=float, default=10.0, help="VISION_POSITION_DELTA rate (Hz) in --auv mode")
    p.add_argument("--ros-args",action="store_true")
    return p.parse_args()

class Master(Node):
    def __init__(self, args):

        super().__init__('master')

        self.auv = AuvPymavlink(self)
        self.auv.Connect(args.endpoint, start_receiver=False)

        profile = self.auv.SITL_PROFILE if args.sitl else self.auv.AUV_PROFILE
        self.get_logger().info(f"Applying {'SITL' if args.sitl else 'AUV'} parameter profile...")
        self.auv.ApplyParamProfile(profile)

        self.auv.StartReceiver()

        # self.validate_ekf()

        self.mission()

        self.get_logger().info("Stopping...")
        
        self.auv.StopReceiver()

    def mission(self):
        self.auv.Arm()
        self.auv.ChangeMode('GUIDED')
        while True:
            self.auv.GoToWaypointLocal(2, 0, 0, 0)
            self.auv.ResetPosEstimate()
        # self.auv.ChangeMode('POSHOLD')
        # try:
        #     # while True:
        #     #     time.sleep(0.1)
        #     #     self.auv.GoToWaypointLocal(1, 1, 1, 0)
        #     #     self.auv.GoToWaypointLocal(0, 0, 0.5, pi/2)
        # except KeyboardInterrupt:

    def ekf_good(self, ekf):
        flags = ekf.flags
        required_bits = []
        for name in [
            "EKF_ATTITUDE",
            "EKF_VELOCITY_HORIZ",
            "EKF_VELOCITY_VERT",
            "EKF_POS_HORIZ_REL",   # common for DVL-based nav
            "EKF_POS_VERT_ABS",    # depth/baro, depends on setup
        ]:
            bit = getattr(mavutil.mavlink, name, None)
            if bit is not None:
                required_bits.append(bit)

        # If enums exist, require all
        if required_bits:
            if not all((flags & b) for b in required_bits):
                return False

        # Variance thresholds (tune for your vehicle)
        if ekf.pos_horiz_variance > 2.0:   # (m^2-ish) tune
            return False
        if ekf.velocity_variance > 1.0:
            return False
        if ekf.pos_vert_variance > 3.0:
            return False

        return True

    def validate_ekf(self):
        print('validating ekf before starting mission')
        t0 = time.time()
        last_local = None

        while True:
            msg = self.auv.the_connection.recv_match(type=["EKF_STATUS_REPORT", "LOCAL_POSITION_NED", "STATUSTEXT"], blocking=True, timeout=1)
            if not msg:
                continue

            if msg.get_type() == "STATUSTEXT":
                text = msg.text.lower()
                if "ekf" in text or "prearm" in text:
                    self.get_logger().info(f"STATUSTEXT:{msg.text}")

            if msg.get_type() == "LOCAL_POSITION_NED":
                last_local = time.time()

            if msg.get_type() == "EKF_STATUS_REPORT":
                ok = self.ekf_good(msg)
                have_local = (last_local is not None and (time.time() - last_local) < 1.0)

                self.get_logger().info(f"EKF flags={msg.flags} "
                    f"pos_h_var={msg.pos_horiz_variance:.3f} "
                    f"vel_var={msg.velocity_variance:.3f} "
                    f"pos_v_var={msg.pos_vert_variance:.3f} "
                    f"local={have_local}")

                if ok and have_local:
                    self.get_logger().info("EKF/odometry looks good; safe to attempt GUIDED.")
                    break

            # if time.time() - t0 > 60:
            #     raise RuntimeError("EKF never became 'good' within 60s")
        # We avoid hardcoding bit numbers; instead, interpret by behavior:
        # If your pymavlink has these enums, use them. If not, see note below.
def main():
    args = parse_args()
    rclpy.init()

    master = Master(args)
    rclpy.spin(master)
    master.destroy_node()
    rclpy.shutdown()
    
   
if __name__ == "__main__":
    main()
