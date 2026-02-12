import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()
        self.model = YOLO('src/nautilus_sensors/vision/yolo_models/best.pt')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            '/yolo/image_annotated',
            10
        )

        self.get_logger().info('YOLOv8 node started')

    def image_callback(self, msg):
        # ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO inference
        results = self.model(frame, conf=0.4, verbose=False)

        # Draw bounding boxes
        annotated_frame = results[0].plot()

        # OpenCV → ROS
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = msg.header  # keep timestamps

        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
