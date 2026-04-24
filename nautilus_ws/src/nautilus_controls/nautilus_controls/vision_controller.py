#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8
from nautilus_bringup.RobotState import RobotState
from nautilus_bringup.ObjectID import ObjectID


class VisionControllerNode(Node):
    def __init__(self):
        super().__init__('vision_controller_node')

        self.current_detection_callback = self.EMPTY_CALLBACK
        self.current_gate_detection_callback = self.EMPTY_CALLBACK
        self.state = None
        self.objects = None
        self.gate_objects = None

        self.state_sub = self.create_subscription(
            Int8,
            '/mission/state',
            self.state_callback,
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


        self.get_logger().info('Vision controller node started.')

    def state_callback(self, msg):

        self.state = RobotState(msg.data)

        if self.state == RobotState.CENTER_GATE:
            self.current_gate_detection_callback = self.CENTER_GATE_CALLBACK

    def CENTER_GATE_CALLBACK(self, msg):
        

    
    def EMPTY_CALLBACK(self,msg):
        pass

    def _obj_detection_wrapper(self, msg):
        self.objects = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
        self.current_detections_callback(msg)
    
    def _gate_detection_wrapper(self, msg):
        self.gate_objects = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]
        self.current_gate_detection_callback(msg)



def main(args=None):
    rclpy.init(args=args)
    node = VisionControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()