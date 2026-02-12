import argparse
import time
from math import pi
from nautilus_bringup.nautilus_bringup.auv_pymavlink import AuvPymavlink
import rclpy
from geometry_msgs.msg import Pose

def parse_args():
    p = argparse.ArgumentParser()
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sitl", action="store_true", help="Run against SITL (keep sim GPS, don't start DVL aiding)")
    mode.add_argument("--auv", action="store_true", help="Run against real vehicle (enable DVL aiding + ExternalNav fusion)")
    p.add_argument("--endpoint", default="udpin:localhost:14551", help="MAVLink endpoint (udpin:... or udpout:...)")
    p.add_argument("--dvl-rate", type=float, default=10.0, help="VISION_POSITION_DELTA rate (Hz) in --auv mode")
    return p.parse_args()

class Master(rclpy.Node):
    def __init__(self, args):

        self.auv = AuvPymavlink()

        # 1) Connect WITHOUT receiver thread (startup uses recv_match() to confirm PARAM_VALUE)
        self.auv.Connect(args.endpoint, start_receiver=False)

        # 2) Apply SITL vs AUV params
        profile = self.auv.SITL_PROFILE if args.sitl else self.auv.AUV_PROFILE
        print(f"Applying {'SITL' if args.sitl else 'AUV'} parameter profile...")
        self.auv.ApplyParamProfile(profile)

        # 3) Start receiver thread AFTER params are set
        self.auv.StartReceiver()

def main():
    args = parse_args()
    rclpy.init(args=args)

    master = Master(args)
    rclpy.spin(master)
    master.destroy_node()
    rclpy.shutdown()
    
    # ---- Your original style control code (kept) ----
    master.auv.Arm()
    master.auv.ChangeMode('GUIDED')
    master.auv.GoToWaypointLocal(2, 0, 0, 0)
    try:
        while True:
            time.sleep(0.1)
            master.auv.GoToWaypointLocal(1, 1, 1, 0)
            master.auv.GoToWaypointLocal(0, 0, 0.5, pi/2)
    except KeyboardInterrupt:
        print("Stopping...")
        master.auv.StopReceiver()


if __name__ == "__main__":
    main()
