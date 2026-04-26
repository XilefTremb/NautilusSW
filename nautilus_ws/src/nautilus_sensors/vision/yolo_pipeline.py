#!/usr/bin/env python3

import argparse
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
from vision.Pixel_and_depth import *
from vision.Angle_between_object import *
from pathlib import Path

def parse_args():
    p = argparse.ArgumentParser()
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true", help="Run against SITL (keep sim GPS, don't start DVL aiding)")
    mode.add_argument("--real", action="store_true", help="Run against real vehicle (enable DVL aiding + ExternalNav fusion)")
    p.add_argument("--ros-args",action="store_true")
    return p.parse_args()

class YoloNode(Node):
    def __init__(self, args):
        super().__init__('yolo_node')
        if args.sim:
            self.mode = 'sim'
        else:
            self.mode = 'real'

        self.bridge = CvBridge()
        if self.mode == 'sim':
            self.model = YOLO(
                '/home/devs/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/model_sim_low_res_openvino_model', task='detect')
        else:
            self.model = YOLO(
                '/home/nautilus/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/Model_Realtime_18_mars.pt')

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
        self.obj_depth_dist_pub = self.create_publisher(
            Float32MultiArray,
            '/yolo/obj_depth_dist',
            10
        )

        self.region_angle_topic = self.create_publisher(
            Float32MultiArray,
            '/yolo/obj_angle',
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/yolo/image_annotated',
            10
        )

        self.get_logger().info(f'YOLOv8 node with depth started, mode : {self.mode}')


    def synced_callback(self, rgb_msg, depth_msg):

        # Convert ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # YOLO inference
        results = self.model(frame, conf=0.4, verbose=False, imgsz=320)

        payload = []
        payload_angle_bet = []

        # Dictionnaire: clé = id objet, valeur = infos pour angle_between_object
        objects = {}

        annotated_frame = frame.copy()

        if results[0].boxes is not None:
            for box in results[0].boxes:

                # ----------- POSITION -----------
                xywh = box.xywh[0].cpu().numpy()
                bbox_cx = int(xywh[0])
                bbox_cy = int(xywh[1])

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
                        bbox_cy=bbox_cy,
                        bbox_cx=bbox_cx,
                        mode=self.mode
                    )
                except Exception as e:
                    self.get_logger().warn(f'Erreur find_depth pour objet {object_id}: {e}')
                    depth_value = None

                # ----------- ANGLE / DIST -----------
                dist_center = find_dist_from_center(bbox_cx, self.mode)

                # ----------- DICT POUR ANGLE BETWEEN -----------
                if object_id not in objects or depth_value < objects[object_id]["depth"]:
                    objects[object_id] = {
                        "depth": depth_value,
                        "bbox_cx": bbox_cx
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
                        (bbox_cx, bbox_cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1
                    )

                    cv2.putText(
                        annotated_frame,
                        f"{dist_center:.2f}px",
                        (bbox_cx, bbox_cy + 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1
                    )

        msg = Float32MultiArray()
        msg.data = payload

        # Layout: N x 3
        nb_objects = len(payload) // 3
        msg.layout.dim = [
            MultiArrayDimension(label='objects', size=nb_objects, stride=max(len(payload), 1)),
            MultiArrayDimension(label='fields', size=3, stride=3)
        ]
        msg.layout.data_offset = 0

        self.obj_depth_dist_pub.publish(msg)

        # -----Publication topic angle et zone------
        # Call function
        payload_angle_bet = switch_case_sub_angle(objects, self.mode)

        msg_angle_between_object = Float32MultiArray()
        msg_angle_between_object.data = payload_angle_bet

        # Layout: N x 3
        nb_objects_angle_bet = len(payload_angle_bet) // 3
        msg_angle_between_object.layout.dim = [
            MultiArrayDimension(label='objects', size=nb_objects_angle_bet, stride=max(len(payload_angle_bet), 1)),
            MultiArrayDimension(label='fields', size=3, stride=3)
        ]
        msg_angle_between_object.layout.data_offset = 0

        self.region_angle_topic.publish(msg_angle_between_object)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = rgb_msg.header
        self.image_pub.publish(out_msg)


def main():
    args = parse_args()
    rclpy.init()
    node = YoloNode(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()