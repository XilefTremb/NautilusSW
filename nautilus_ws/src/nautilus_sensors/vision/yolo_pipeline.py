#!/usr/bin/env python3

import argparse
import rclpy
import torch
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int16
from ultralytics import YOLO
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from collections import deque

from vision.Pixel_and_depth import *
from vision.Angle_between_object import *
from vision.Filters import TemporalFilter

MOVING_MEAN_ACTIVATED =  True
PREQUALIFICATION = False

def parse_args():
    p = argparse.ArgumentParser()

    # -------- MODE --------
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true")
    mode.add_argument("--real", action="store_true")

    # -------- MODEL TYPE --------
    model_type = p.add_mutually_exclusive_group(required=True)
    model_type.add_argument("--obb", action="store_true")
    model_type.add_argument("--bbox", action="store_true")

    return p.parse_args()


class YoloNode(Node):
    def __init__(self, args):
        super().__init__('yolo_node')

        # ----------- MODE -----------
        if args.sim:
            self.mode = 'sim'
            self.model = YOLO(
                '/home/nautilus/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/sim_640_bbox_18mars.pt')
        else:
            self.mode = 'real'
            self.model = YOLO('/home/nautilus/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/model_prequal.pt')

        # -------- MODEL TYPE --------
        if args.obb:
            self.type_yolo = 'obb'

        else:
            self.type_yolo = 'bbox'

        # ----------- INIT PARAMS FILTER-----------
        self.depth_threshold = 9999999

        self.temporal_filter = TemporalFilter(self)

        if PREQUALIFICATION:
            self.depth_history = {
                "gate_left": deque(maxlen=10),
                "gate_right": deque(maxlen=10),}
            self.last_seen = {
                "gate_left": None,
                "gate_right": None, }

        else:
            self.depth_history = {}
            self.last_seen = {}

        self.angle_history = deque(maxlen=10)
        self.spike_threshold_mm = 1000
        self.reset_after_sec = 2.0

        # ----------- CPU -----------
        if torch.cuda.is_available():
            self.model.to('cuda')

        self.bridge = CvBridge()

        # ----------- SUBSCRIBER -----------
        self.rgb_sub = Subscriber(self, Image, 'oakd/camera/image_raw')
        self.depth_sub = Subscriber(self, Image, 'oakd/camera/depth/image_raw')

        self.depth_threshold_sub = self.create_subscription(
            Int16,
            '/yolo/depth_threshold',
            self.depth_threshold_callback,
            10
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
            allow_headerless=True
        )

        self.ts.registerCallback(self.synced_callback)

        # ----------- PUBLISHER -----------
        #self.obj_depth_dist_pub = self.create_publisher(Float32MultiArray, '/yolo/obj_depth_dist', 10)
        #self.region_angle_topic = self.create_publisher(Float32MultiArray, '/yolo/obj_angle', 10)
        self.detection_topic = self.create_publisher(Float32MultiArray, '/yolo/detection', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.mean_depth_forward_cam = self.create_publisher(Int16, '/yolo/mean_depth_forward_cam', 10)

        self.get_logger().info(f'YOLOv8 node started, mode: {self.mode}, type: {self.type_yolo}')

    def synced_callback(self, rgb_msg, depth_msg):

        # ----------- FRAME -----------
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        # ----------- MODEL -----------
        results = self.model(frame, conf=0.4, verbose=False)
        payload = []

        if PREQUALIFICATION:
            gate_legs_detected = []
            dict_leg = {}
        else:
            objects = {}

        annotated_frame = frame.copy()

        if self.type_yolo == 'obb':
            detection = results[0].obb
        else:
            detection = results[0].boxes

        if detection is not None:
            for box in detection:

                # ----------- BOX INFORMATION-----------
                if self.type_yolo == 'obb':
                    box_cx, box_cy, x1, y1, x2, y2, half, points = self.obb_model(box)
                else:
                    box_cx, box_cy, x1, y1, x2, y2, half = self.bbox_model(box)

                # Clamp to image
                h, w = depth.shape
                x1, x2 = np.clip([x1, x2], 0, w - 1)
                y1, y2 = np.clip([y1, y2], 0, h - 1)

                # ----------- CLASS / CONF -----------
                object_id = int(box.cls[0])
                confidence = float(box.conf[0])

                # ----------- DEPTH -----------
                try:
                    depth_value = find_depth(
                        depth_frame=depth,
                        half=half,
                        bbox_cy=box_cy,
                        bbox_cx=box_cx,
                        mode=self.mode
                    )
                
                except Exception as e:
                    self.get_logger().warn(f'Depth error for obj {object_id}: {e}')
                    depth_value = None

                if depth_value is None:
                    continue

                # ----------- DIST / ANGLE -----------
                dist_center = find_dist_from_center(box_cx, self.mode)

                # ----------- DICT FOR ANGLE BETWEEN -----------

                if PREQUALIFICATION:
                    if object_id == ObjectID.GATE_LEG:
                        gate_legs_detected.append({
                            "depth": depth_value,
                            "box_cx": box_cx
                        })

                else:
                    if object_id not in objects or depth_value < objects[object_id]["depth"]:
                        if MOVING_MEAN_ACTIVATED:
                            depth_value = self.temporal_filter.moving_median_filter(
                                key=f"depth_{object_id}",
                                new_value=depth_value
                            )

                        objects[object_id] = {
                            "depth": depth_value,
                            "box_cx": box_cx}

                if depth_value < self.depth_threshold:
                    payload.extend([float(object_id), float(dist_center), float(depth_value), 0.0])
                    if self.type_yolo == 'obb':
                        # ----------- DRAW OBB -----------
                        cv2.polylines(
                            annotated_frame,
                            [points],
                            isClosed=True,
                            color=(0, 255, 0),
                            thickness=2
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

                        cv2.putText(
                            annotated_frame,
                            f"{depth_value:.2f}mm",
                            (box_cx, box_cy),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1
                        )

                        cv2.putText(
                            annotated_frame,
                            f"{dist_center:.2f}px",
                            (box_cx, box_cy + 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1
                        )

                    elif self.type_yolo== 'bbox':
                        # ----------- DRAW BBOX -----------
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

                        cv2.putText(
                            annotated_frame,
                            f"{depth_value:.2f}mm",
                            (box_cx, box_cy),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1
                        )

                        cv2.putText(
                            annotated_frame,
                            f"{dist_center:.2f}px",
                            (box_cx, box_cy + 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1
                        )

        # ----------- ANGLE BETWEEN OBJECTS -----------
        if PREQUALIFICATION:
            dict_leg = self.build_gate_leg_dict(gate_legs_detected, MOVING_MEAN_ACTIVATED)
            payload_angle_bet = switch_case_sub_angle(dict_leg, self.mode, PREQUALIFICATION)

            if len(payload_angle_bet) >= 2 and MOVING_MEAN_ACTIVATED:
                angle = payload_angle_bet[3]
                self.angle_history.append(angle)
                angle_filtered = float(np.median(self.angle_history))
                payload_angle_bet[3] = angle_filtered

        else:
            payload_angle_bet = switch_case_sub_angle(objects, self.mode, PREQUALIFICATION)

            if MOVING_MEAN_ACTIVATED:
                for i in range(0, len(payload_angle_bet), 4):
                    group_id = int(payload_angle_bet[i])
                    angle_index = i + 3

                    payload_angle_bet[angle_index] = self.temporal_filter.moving_median_filter(
                        key=f"angle_{group_id}",
                        new_value=payload_angle_bet[angle_index])


        payload.extend(payload_angle_bet)

        # ----------- PUBLISH DETECTION -----------
        msg = Float32MultiArray()
        msg.data = payload

        nb_objects = len(payload) // 4
        msg.layout.dim = [
            MultiArrayDimension(label='objects', size=nb_objects, stride=max(len(payload), 1)),
            MultiArrayDimension(label='fields', size=4, stride=4)
        ]
        msg.layout.data_offset = 0

        self.detection_topic.publish(msg)

        # ----------- IMAGE OUTPUT -----------
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = rgb_msg.header
        self.image_pub.publish(out_msg)

        # ----------- GLOBAL DEPTH -----------
        depth_global_mean = global_median_forward_cam(depth, self.mode)
        if not depth_global_mean < -32767 and not depth_global_mean > 32767:
            msg_depth = Int16()
            msg_depth.data = int(depth_global_mean)
            self.mean_depth_forward_cam.publish(msg_depth)

    def depth_threshold_callback(self, msg):
        self.depth_threshold = msg.data
        self.get_logger().info(f'Updated depth threshold: {self.depth_threshold}')

    def obb_model(self, box):
        xywhr = box.xywhr[0].cpu().numpy()
        box_cx = int(xywhr[0])
        box_cy = int(xywhr[1])

        # ----------- CORNERS -----------
        points = box.xyxyxyxy[0].cpu().numpy().astype(int)

        x_coords = points[:, 0]
        y_coords = points[:, 1]

        x1, x2 = x_coords.min(), x_coords.max()
        y1, y2 = y_coords.min(), y_coords.max()

        # ----------- DEPTH ZONE -----------
        depth_zone_h = abs(y2 - y1)
        depth_zone_w = abs(x2 - x1)

        if depth_zone_w > depth_zone_h:
            half = 0.40 * depth_zone_h
        else:
            half = 0.40 * depth_zone_w

        half = max(1, min(8, int(np.ceil(half))))

        return box_cx, box_cy, x1, y1, x2, y2, half, points

    def bbox_model(self, box):
        xywh = box.xywh[0].cpu().numpy()
        box_cx = int(xywh[0])
        box_cy = int(xywh[1])

        x1, y1, x2, y2 = map(int, box.xyxy[0])

        half = 5

        return box_cx, box_cy, x1, y1, x2, y2, half

    def build_gate_leg_dict(self, gate_legs_detected, activated):
        """
        Sort gate legs left/right using their x-position in the camera,
        then smooth left and right depths with a moving average of 10 frames.
        ONLY FOR PREQUALIFICATION
        """
        if len(gate_legs_detected) < 2:
            return {}

        # If more than two legs are detected, keep the leftmost and rightmost ones.
        gate_legs_detected = sorted(gate_legs_detected, key=lambda obj: obj["box_cx"])
        gate_left = gate_legs_detected[0]
        gate_right = gate_legs_detected[-1]

        if activated:
            left_filtered = self.spike_filter_with_timeout(
                gate_left["depth"],
                self.depth_history["gate_left"],
                "gate_left"
            )

            right_filtered = self.spike_filter_with_timeout(
                gate_right["depth"],
                self.depth_history["gate_right"],
                "gate_right"
            )

            self.depth_history["gate_left"].append(left_filtered)
            self.depth_history["gate_right"].append(right_filtered)

            gate_left["depth"] = float(np.median(self.depth_history["gate_left"]))
            gate_right["depth"] = float(np.median(self.depth_history["gate_right"]))

        # 0 = left, 1 = right. Angle_between_object.py now assumes this is already ordered.
        return {
            0: gate_left,
            1: gate_right,
        }


def main():
    args = parse_args()
    rclpy.init()
    node = YoloNode(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()