import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Header


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()
        self.model = YOLO('src/nautilus_sensors/vision/yolo_models/model_sim.pt')

        # ---------------- SUBSCRIBERS ----------------
        self.rgb_sub = Subscriber(self, Image, '/camera/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')

        # ApproximateTimeSynchronizer with allow_headerless=True
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
            allow_headerless=True
        )
        self.ts.registerCallback(self.synced_callback)

        # ---------------- PUBLISHERS ----------------
        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            '/yolo/id_depth_angle',
            10
        )

        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            '/yolo/region_angle',
            10
        )

        self.get_logger().info('YOLOv8 node with depth started')

    def synced_callback(self, rgb_msg, depth_msg):

        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # YOLO inference
        results = self.model(frame, conf=0.4, verbose=False)


        # Publish annotated image
        # out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        # out_msg.header = rgb_msg.header
        # self.image_pub.publish(out_msg)

        # Publish detections
        # det_msg = Float32MultiArray()
        # det_msg.data = detections_data
        # self.detection_pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()