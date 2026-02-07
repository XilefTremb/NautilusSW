import argparse
import time
from math import pi
from auv_pymavlink import AuvPymavlink


def parse_args():
    p = argparse.ArgumentParser()
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sitl", action="store_true", help="Run against SITL (keep sim GPS, don't start DVL aiding)")
    mode.add_argument("--auv", action="store_true", help="Run against real vehicle (enable DVL aiding + ExternalNav fusion)")
    p.add_argument("--endpoint", default="udpin:localhost:14551", help="MAVLink endpoint (udpin:... or udpout:...)")
    p.add_argument("--dvl-rate", type=float, default=10.0, help="VISION_POSITION_DELTA rate (Hz) in --auv mode")
    return p.parse_args()


# Parameter profiles (minimal, extend as needed)
SITL_PROFILE = {
    "VISO_TYPE": 0,
    "EK3_SRC1_POSXY": 3,   # GPS
    "EK3_SRC1_VELXY": 3,   # GPS
}

AUV_PROFILE = {
    "VISO_TYPE": 1,        # MAVLink vision/odometry (DVL integration)
    "EK3_SRC1_POSXY": 6,   # ExternalNav
    "EK3_SRC1_VELXY": 6,   # ExternalNav
}


def main():
    args = parse_args()

    auv = AuvPymavlink()

    # 1) Connect WITHOUT receiver thread (startup uses recv_match() to confirm PARAM_VALUE)
    auv.Connect(args.endpoint, start_receiver=False)

    # 2) Apply SITL vs AUV params
    profile = SITL_PROFILE if args.sitl else AUV_PROFILE
    print(f"Applying {'SITL' if args.sitl else 'AUV'} parameter profile...")
    auv.ApplyParamProfile(profile)

    # 3) Start receiver thread AFTER params are set
    auv.StartReceiver()

    # 4) Optional DVL thread (only in --auv)
    dvl_stop = None

    def dvl_delta_fn():
        # TODO: replace with real integrated DVL deltas (meters)
        return 0.0, 0.0, 0.0

    if args.auv:
        print("Starting DVL -> VISION_POSITION_DELTA thread...")
        dvl_stop, _ = auv.StartDvlThread(dvl_delta_fn, rate_hz=args.dvl_rate, confidence=100.0)
    else:
        print("SITL mode: DVL thread disabled (SITL sim GPS will be used).")

    # ---- Your original style control code (kept) ----
    auv.Arm()
    auv.ChangeMode('GUIDED')
    auv.GoToWaypointLocal(2, 0, 0, 0)
    try:
        while True:
            time.sleep(0.1)
            auv.GoToWaypointLocal(1, 1, 1, 0)
            auv.GoToWaypointLocal(0, 0, 0.5, pi/2)
    except KeyboardInterrupt:
        print("Stopping...")
        if dvl_stop is not None:
            dvl_stop.set()
        auv.StopReceiver()


if __name__ == "__main__":
    main()
