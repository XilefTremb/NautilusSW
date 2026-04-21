#!/usr/bin/env python3

import argparse
import time
import rclpy
from rclpy.node import Node


class StateMachine(Node):
    def __init__(self):

        super().__init__('StateMachine')

        self.mission()

        self.get_logger().info("Stopping...")
        self.auv.StopReceiver()

    def mission(self):

        time.sleep(1.0)

        # State machine logic must come here?
 
def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
   
if __name__ == "__main__":
    main()
