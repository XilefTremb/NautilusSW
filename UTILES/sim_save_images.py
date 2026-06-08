#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

SAVE_DIR = os.path.expanduser("~/Documents/captured_images/oakd_640x480")
TARGET_HZ = 2.0
MAX_IMAGES = 300

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.bridge = CvBridge()
        self.last_save_time = 0.0
        self.image_count = 0

        os.makedirs(SAVE_DIR, exist_ok=True)

        self.sub = self.create_subscription(
            Image,
            '/oakd/camera/image_raw',
            self.callback,
            10
        )

    def callback(self, msg):
        now = time.time()

        # Limit to 2 Hz
        if now - self.last_save_time < 1.0 / TARGET_HZ:
            return

        if self.image_count >= MAX_IMAGES:
            self.get_logger().info("Reached 300 images. Shutting down.")
            rclpy.shutdown()
            return

        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        filename = os.path.join(SAVE_DIR, f"sim_image_{self.image_count:04d}.png")
        cv2.imwrite(filename, frame)

        self.get_logger().info(f"Saved {filename}")

        self.image_count += 1
        self.last_save_time = now


def main():
    rclpy.init()
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
