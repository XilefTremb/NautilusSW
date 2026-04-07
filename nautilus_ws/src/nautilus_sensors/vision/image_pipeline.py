import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2

class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__('vision_pipeline_node')

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_detections = None

        # Publisher
        self.edge_pub = self.create_publisher(Image, '/vision/edges_depth', 10)

        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(Float32MultiArray, '/yolo/detections_depth', self.detection_callback, 10)

        self.get_logger().info('Vision pipeline node started')

    def rgb_callback(self, msg):
        self.latest_rgb = msg
        self.run_pipeline_if_ready()

    def depth_callback(self, msg):
        self.latest_depth = msg
        self.run_pipeline_if_ready()

    def detection_callback(self, msg):
        self.latest_detections = msg
        self.run_pipeline_if_ready()

    def run_pipeline_if_ready(self):
        if self.latest_rgb is None or self.latest_depth is None or self.latest_detections is None:
            return

        # Convert images
        rgb = self.bridge.imgmsg_to_cv2(self.latest_rgb, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='32FC1')

        # Black canvas
        output = np.zeros_like(rgb)

        # Decode detections
        detections = self.decode_detections(self.latest_detections.data)

        for det in detections:
            x1, y1, x2, y2, depth_val, class_id, conf = det
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            h, w = rgb.shape[:2]
            x1, x2 = max(0, x1), min(w, x2)
            y1, y2 = max(0, y1), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue

            roi_rgb = rgb[y1:y2, x1:x2]
            roi_depth = depth[y1:y2, x1:x2]

            gray = cv2.cvtColor(roi_rgb, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Clean depth
            valid_mask = np.isfinite(roi_depth)
            depth_clean = np.copy(roi_depth)
            depth_clean[~valid_mask] = 0

            # Normalize FIRST (important for OpenCV median)
            depth_norm = cv2.normalize(depth_clean, None, 0, 255, cv2.NORM_MINMAX)
            depth_norm = depth_norm.astype(np.uint8)

            # Apply median blur (kernel must be odd → use 5 ≈ your 4x4 idea)
            depth_smooth = cv2.medianBlur(depth_norm, 7)

            # Apply colormap
            depth_color = cv2.applyColorMap(depth_smooth, cv2.COLORMAP_JET)

            # Mask edges
            edge_mask = edges > 0
            colored_edges = np.zeros_like(roi_rgb)
            colored_edges[edge_mask] = depth_color[edge_mask]

            # Paste edges into output
            output[y1:y2, x1:x2] = colored_edges

        # Publish
        msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
        msg.header = self.latest_rgb.header
        self.edge_pub.publish(msg)

        # Reset latests to avoid processing same frames repeatedly
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_detections = None

    def decode_detections(self, data):
        detections = []
        if len(data) % 7 != 0:
            self.get_logger().warn("Detection array size is not multiple of 7")
            return detections

        for i in range(0, len(data), 7):
            x1 = data[i]
            y1 = data[i+1]
            x2 = data[i+2]
            y2 = data[i+3]
            depth = data[i+4]
            class_id = data[i+5]
            conf = data[i+6]
            detections.append((x1, y1, x2, y2, depth, class_id, conf))
        return detections

def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()