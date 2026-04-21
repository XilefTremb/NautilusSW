#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Header
from Pixel_and_depth import *
from Angle_between_object import *


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()
        self.model = YOLO(
            '/home/devs/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/model_sim.pt')

        # ---------------- SUBSCRIBERS ----------------
        self.rgb_sub = Subscriber(self, Image, 'oakd/camera/image_raw')
        self.depth_sub = Subscriber(self, Image, 'oakd/camera/depth/image_raw')

        # ApproximateTimeSynchronizer with allow_headerless=True
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
            allow_headerless=True
        )
        self.ts.registerCallback(self.synced_callback)

        # ---------------- PUBLISHERS ----------------
        self.depth_angle_topic = self.create_publisher(
            Float32MultiArray,
            '/yolo/id_depth_angle',
            10
        )

        self.region_angle_topic = self.create_publisher(
            Float32MultiArray,
            '/yolo/region_angle',
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/yolo/image_annotated',
            10
        )

        self.get_logger().info('YOLOv8 node with depth started for simulation')

        # ---------------- mode ----------------
        self.mode = "sim"

    def synced_callback(self, rgb_msg, depth_msg):

        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # YOLO inference
        results = self.model(frame, conf=0.4, verbose=False)

        payload = []
        payload_angle_bet = []

        # Dictionnaire: clé = id objet, valeur = infos pour angle_between_object
        objets = {}

        annotated_frame = frame.copy()

        if results[0].boxes is not None:
            for box in results[0].boxes:

                # ----------- POSITION -----------
                xywh = box.xywh[0].cpu().numpy()
                cx = int(xywh[0])
                cy = int(xywh[1])

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Clamp image
                h, w = depth.shape
                x1, x2 = np.clip([x1, x2], 0, w - 1)
                y1, y2 = np.clip([y1, y2], 0, h - 1)

                # ----------- ID / CONF -----------
                object_id = int(box.cls[0])
                confidence = float(box.conf[0])

                # ----------- DEPTH -----------
                try:
                    depth_value = find_depth(
                        depth_frame=depth,
                        half=1,
                        cy_boundingbox=cy,
                        cx_boundingbox=cx,
                        mode=self.mode
                    )
                except Exception as e:
                    self.get_logger().warn(f'Erreur find_depth pour objet {object_id}: {e}')
                    depth_value = None

                # ----------- ANGLE / DIST -----------
                dist_center = find_dist_from_center(cx, self.mode)

                # ----------- DICT POUR ANGLE BETWEEN -----------
                if object_id not in objets or depth_value < objets[object_id]["depth"]:
                    objets[object_id] = {
                        "depth": depth_value,
                        "angle": dist_center
                    }
                
                if depth_value<5000:
                    # ----------- PAYLOAD -----------
                    payload.extend([float(object_id), depth_value, dist_center])

                    # ----------- AFFICHAGE -----------
                    # ----------- DRAW BOX -----------
                    cv2.rectangle(
                        annotated_frame,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2
                    )

                    label = f"{object_id} | {confidence:.2f}"
                    cv2.putText(
                        annotated_frame,
                        label,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1
                    )

                    # ----------- TES INFOS -----------
                    cv2.putText(
                        annotated_frame,
                        f"{depth_value:.2f}mm",
                        (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1
                    )

                    cv2.putText(
                        annotated_frame,
                        f"{dist_center:.2f}px",
                        (cx, cy + 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1
                    )

        # -----Publication topic profondeur + angle------
        msg = Float32MultiArray()
        msg.data = payload

        # Layout: N x 3
        nb_objets = len(payload) // 3
        msg.layout.dim = [
            MultiArrayDimension(label='objects', size=nb_objets, stride=max(len(payload), 1)),
            MultiArrayDimension(label='fields', size=3, stride=3)
        ]
        msg.layout.data_offset = 0

        self.depth_angle_topic.publish(msg)

        # -----Publication topic angle et zone------
        # Call function
        payload_angle_bet = switch_case_sub_angle(objets, self.mode)

        msg_angle_between_angle = Float32MultiArray()
        msg_angle_between_angle.data = payload_angle_bet

        # Layout: N x 2
        nb_objets_angle_bet = len(payload_angle_bet) // 2
        msg_angle_between_angle.layout.dim = [
            MultiArrayDimension(label='objects', size=nb_objets_angle_bet, stride=max(len(payload_angle_bet), 1)),
            MultiArrayDimension(label='fields', size=2, stride=2)
        ]
        msg_angle_between_angle.layout.data_offset = 0

        self.region_angle_topic.publish(msg_angle_between_angle)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = rgb_msg.header
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()