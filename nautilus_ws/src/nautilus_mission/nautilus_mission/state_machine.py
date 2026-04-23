#!/usr/bin/env python3

import argparse
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8
from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID


class StateMachine(Node):
    def __init__(self):

        super().__init__('StateMachine')

        self.state = None
        self.freq = 2.0

        # Subscriber - to the output of the YOLO pipeline
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/obj_depth_dist',
            self.ObjDetectionCallback,
            10
        )

        # Publisher - for the current state
        self.pub = self.create_publisher(
            Int8,
            '/mission/state',
            10
        )
        

        self.StateMachine()

    def StateMachine(self):

        # First state: Dive to the right depth
        # TO BE DONE
        # while not self.IsDived():
        #     self.state = 1

        # Second state: Find the gate or search for it
        while not self.IsGatePresent():
            continue

        self.state = 3 
        self.StateSender()

        

        # Third state: Center the vehicle on the gate
        # TO BE DONE
        # while not self.IsCentered():
        #     self.state = 3

        self.state = RobotState.CENTER_GATE
        self.StateSender()

    def StateSender(self):
        # Publish the state at a certain frequency
        msg = Int8()
        msg.data = self.state
        self.pub.publish(msg)
        self.get_logger().info(f"Published state: {self.state}")

    def ObjDetectionCallback(self, msg):
        data = msg.data
        self.objects = [data[i:i+3] for i in range(0, len(data), 3)]

    def IsGatePresent(self):
        present = any(int(obj[0]) == 1 for obj in self.objects) and any(int(obj[0]) == 3 for obj in self.objects)
        self.get_logger().info("A gate was found!")
        return present
        
def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
   
if __name__ == "__main__":
    main()
