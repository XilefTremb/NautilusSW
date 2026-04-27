#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Float32, Int16
from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID, GateLikeObjectID
import math


class VisionControllerNode(Node):
    def __init__(self):
        super().__init__('vision_controller_node')

        self.current_detection_callback = self.EMPTY_CALLBACK
        self.current_gate_detection_callback = self.CENTER_GATE_CALLBACK
        self.state = None
        self.objects = None
        self.gate_objects = None
        self.target_gate = GateLikeObjectID.GATE_LEFT_MID
        self.last_target_gate_detection = None

        self.state_sub = self.create_subscription(
            Int8,
            '/mission/state',
            self.state_callback,
            10
        )

        self.target_gate_sub = self.create_subscription(
            Int8,
            '/mission/target_gate',
            self.target_gate_callback,
            10
        )

        self.obj_detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/obj_depth_dist',
            self._obj_detection_wrapper,
            10
        )

        self.gate_detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/obj_angle',
            self._gate_detection_wrapper,
            10
        )

        self.yaw_error_pub = self.create_publisher(
            Float32,
            '/control/vision_errors/yaw',
            10)
        
        self.forward_error_pub = self.create_publisher(
            Float32,
            '/control/vision_errors/forward',
            10
        )
        
        self.lateral_error_pub = self.create_publisher(
            Float32,
            '/control/vision_errors/lateral',
            10
        )

        self.forward_cmd_pub = self.create_publisher(
            Int16,
            '/control/cmd/forward',
            10
        )

        self.get_logger().info('Vision controller node started.')

    def state_callback(self, msg):

        self.state = RobotState(msg.data)

        if self.state == RobotState.CENTER_GATE:
            self.current_gate_detection_callback = self.CENTER_GATE_CALLBACK

        if self.state == RobotState.APPROACH_GATE:
            self.current_gate_detection_callback = self.APPROACH_GATE_CALLBACK
            
    def target_gate_callback(self, msg):
        self.target_gate = RobotState(msg.data)

    def CENTER_GATE_CALLBACK(self, msg):
        if self.last_target_gate_detection is not None:

            forward_error, lateral_error = self.split_angle(self.last_target_gate_detection[1])
            msg = Float32()
            msg.data = forward_error
            self.forward_error_pub.publish(msg)

            msg = Float32()
            msg.data = lateral_error
            self.lateral_error_pub.publish(msg)
            
            msg = Float32()
            msg.data = self.last_target_gate_detection[2]
            self.yaw_error_pub.publish(msg)

    def APPROACH_GATE_CALLBACK(self, msg):
        if self.last_target_gate_detection is not None:
            msg = Float32()
            msg.data = self.last_target_gate_detection[2]
            self.yaw_error_pub.publish(msg)
        
        msg = Int16()
        msg.data = 1600
        self.forward_cmd_pub.publish(msg)
    
    def EMPTY_CALLBACK(self,msg):
        pass

    def _obj_detection_wrapper(self, msg):
        self.objects = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
        self.current_detection_callback(msg)
    
    def _gate_detection_wrapper(self, msg):
        self.gate_objects = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
        if self.target_gate is not None:
            self.last_target_gate_detection = next((o for o in self.gate_objects if o[0] == self.target_gate), None)
        self.current_gate_detection_callback(msg)

    def split_angle(self, angle_deg):
        """
        split angle of gate object so 2 identical pid convert them into 
        forward and lateral movement to move parallel to the gate
        """
        angle = math.radians(angle_deg)

        forward_error = -angle_deg* math.sin(angle)
        lateral_error = angle_deg * math.cos(angle)

        return forward_error, lateral_error



def main(args=None):
    rclpy.init(args=args)
    node = VisionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()