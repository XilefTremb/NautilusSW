#!/usr/bin/env python3

import argparse
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Int16
from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID, GateLikeObjectID


class StateMachine(Node):
    def __init__(self):

        super().__init__('StateMachine')

        self.state = None
        self.target_gate_ID = None
        self.gate_objects = None
        self.objects = None
        self.last_target_gate_detection = None

        # Subscriber - to the output of the YOLO pipeline
        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/obj_depth_dist',
            self.ObjDetectionCallback,
            10
        )

        # Subscriber - to the output of the YOLO pipeline
        self.gate_detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/obj_angle',
            self.GateDetectionCallback,
            10
        )

        # Publisher - for the current state
        self.state_pub = self.create_publisher(
            Int8,
            '/mission/state',
            10
        )
        
        # Publisher - for the target gate
        self.target_gate_pub = self.create_publisher(
            Int8,
            '/mission/target_gate',
            10
        )

        self.forward_cmd_pub = self.create_publisher(
            Int16,
            '/control/cmd/forward',
            10
        )

        self.depth_threshold_pub = self.create_publisher(
            Int16,
            '/yolo/depth_threshold',
            10
        )

        self.timer1 = self.create_timer(1/10, self.StateMachine)
        self.timer2 = self.create_timer(1/10, self.ForwardCmdPub)

        self.state = RobotState.SEARCH
        self.StateSender()

        self.target_gate_ID = GateLikeObjectID.GATE_LEFT_MID
        self.TargetGateSender()

    def StateMachine(self):
        if self.state == RobotState.SEARCH:
            if self.IsGatePresent():
                self.state = RobotState.CENTER_GATE
                self.StateSender()
            else:
                self.get_logger().info('Target gate is not in view')

        elif self.state == RobotState.CENTER_GATE:
            if self.IsGateCentered():
                self.state = RobotState.APPROACH_GATE
                self.StateSender()

        elif self.state == RobotState.APPROACH_GATE:
            if self.IsGateApproached():
                self.state = RobotState.TRAVERSE_GATE
                self.StateSender()
        
        elif self.state == RobotState.TRAVERSE_GATE:
            msg = Int16()
            msg.data = 15000
            self.depth_threshold_pub.publish(msg)

    def StateSender(self):
        msg = Int8()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Published state: {self.state.name}")

    def TargetGateSender(self):
        msg = Int8()
        msg.data = self.target_gate_ID
        self.state_pub.publish(msg)
        self.get_logger().info(f"Published target gate: {self.target_gate_ID.name}")

    def ForwardCmdPub(self):
        if self.state == RobotState.TRAVERSE_GATE:
            msg = Int16()
            msg.data = 1600
            self.forward_cmd_pub.publish(msg)
    
    def ObjDetectionCallback(self, msg):
        data = msg.data
        self.objects = [data[i:i+3] for i in range(0, len(data), 3)]

    def GateDetectionCallback(self, msg):
        data = msg.data
        self.gate_objects = [data[i:i+3] for i in range(0, len(data), 3)]
        if self.target_gate_ID is not None:
            self.last_target_gate_detection = next((o for o in self.gate_objects if GateLikeObjectID(o[0]) == self.target_gate_ID), None)

    def IsGatePresent(self):
        if self.last_target_gate_detection is not None:
            self.get_logger().info(f"Gate like object {self.target_gate_ID.name} was found!")
            return True
        else:
            return False
    
    def IsGateCentered(self):
        if self.last_target_gate_detection is not None:
            if abs(self.last_target_gate_detection[1]) < 5 and abs(self.last_target_gate_detection[2]) < 30:   
                self.get_logger().info(f"Gate like object {self.target_gate_ID.name} is centered!")
                return True
            else:
                return False
            
    def IsGateApproached(self):
        if self.objects is not None:
            obj = next((o for o in self.objects if int(o[0]) == ObjectID.REQUIN), None)

            if obj is not None and obj[1] < 1500:
                self.get_logger().info("Gate was approached!")
                return True
            else:
                return False
            
def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
   
if __name__ == "__main__":
    main()
