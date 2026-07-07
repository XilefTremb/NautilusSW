import argparse
import time

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--timer", type=float, default=0.0)

    return parser.parse_known_args()

def main():

    parsed_args, ros_args = parse_args()
    print(f"sim : {parsed_args.sim}, {parsed_args.timer}")

if __name__=="__main__":
    main()
