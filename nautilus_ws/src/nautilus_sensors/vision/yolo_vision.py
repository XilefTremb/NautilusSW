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
        self.image_pub = self.create_publisher(
            Image,
            '/yolo/image_annotated',
            10
        )

        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            '/yolo/detections_depth',
            10
        )

        self.get_logger().info('YOLOv8 node with depth started')

    def synced_callback(self, rgb_msg, depth_msg):

        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # YOLO inference
        results = self.model(frame, conf=0.4, verbose=False)

        annotated_frame = results[0].plot()

        detections_data = []

        if results[0].boxes is not None:
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Clamp to image bounds
                h, w = depth.shape
                x1, x2 = np.clip([x1, x2], 0, w - 1)
                y1, y2 = np.clip([y1, y2], 0, h - 1)

                # Extract depth region
                depth_crop = depth[y1:y2, x1:x2]
                depth_crop = depth_crop[np.isfinite(depth_crop)]

                if depth_crop.size > 0:
                    depth_value = float(np.median(depth_crop))
                else:
                    depth_value = float('nan')

                # YOLO info
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                # Append structured data
                detections_data.extend([
                    float(x1), float(y1),
                    float(x2), float(y2),
                    depth_value,
                    float(class_id),
                    confidence
                ])

                # Draw depth on image
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.putText(
                    annotated_frame,
                    f"{depth_value:.2f}m",
                    (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = rgb_msg.header
        self.image_pub.publish(out_msg)

        # Publish detections
        det_msg = Float32MultiArray()
        det_msg.data = detections_data
        self.detection_pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()