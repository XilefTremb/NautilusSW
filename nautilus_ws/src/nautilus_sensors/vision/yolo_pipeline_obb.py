#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int16
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge
import torch
from message_filters import Subscriber, ApproximateTimeSynchronizer
from vision.Pixel_and_depth import *
from vision.Angle_between_object import *
from collections import deque

MOVING_MEAN_ACTIVATED =  False

def parse_args():
    p = argparse.ArgumentParser()
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true")
    mode.add_argument("--real", action="store_true")
    p.add_argument("--ros-args", action="store_true")
    return p.parse_args()


class YoloNode(Node):
    def __init__(self, args):
        super().__init__('yolo_node')

        # ----------- MODE -----------
        if args.sim:
            self.mode = 'sim'
            self.model = YOLO(
                '/home/devs/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/obb_sim_320.pt')
        else:
            self.mode = 'real'
            self.model = YOLO('/home/nautilus/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/model_prequal.pt')

        # ----------- INIT PARAMS -----------
        self.depth_threshold = 5000
        self.depth_history = {
            "gate_left": deque(maxlen=10),
            "gate_right": deque(maxlen=10),}

        self.angle_history = deque(maxlen=10)

        self.last_seen = {
            "gate_left": None,
            "gate_right": None, }

        self.spike_threshold_mm = 1000
        self.reset_after_sec = 2.0
        #self.depth_history = {}
        #self.last_seen = {}

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
        self.obj_depth_dist_pub = self.create_publisher(Float32MultiArray, '/yolo/obj_depth_dist', 10)
        self.region_angle_topic = self.create_publisher(Float32MultiArray, '/yolo/obj_angle', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.mean_depth_forward_cam = self.create_publisher(Int16, '/yolo/mean_depth_forward_cam', 10)

        self.get_logger().info(f'YOLOv8 OBB node started, mode: {self.mode}')

    def synced_callback(self, rgb_msg, depth_msg):

        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

        results = self.model(frame, conf=0.4, verbose=False)

        payload = []
        payload_angle_bet = []
        objects = {}
        gate_legs_detected = []
        dict_leg = {}
        annotated_frame = frame.copy()


        # 🔥 OBB processing
        if results[0].obb is not None:
            for obb in results[0].obb:

                # ----------- CENTER -----------
                xywhr = obb.xywhr[0].cpu().numpy()
                bbox_cx = int(xywhr[0])
                bbox_cy = int(xywhr[1])

                # ----------- CORNERS -----------
                points = obb.xyxyxyxy[0].cpu().numpy().astype(int)

                x_coords = points[:, 0]
                y_coords = points[:, 1]

                x1, x2 = x_coords.min(), x_coords.max()
                y1, y2 = y_coords.min(), y_coords.max()

                # Clamp to image
                h, w = depth.shape
                x1, x2 = np.clip([x1, x2], 0, w - 1)
                y1, y2 = np.clip([y1, y2], 0, h - 1)

                # ----------- CLASS / CONF -----------
                object_id = int(obb.cls[0])
                confidence = float(obb.conf[0])

                # ----------- DEPTH ZONE -----------
                depth_zone_h = abs(y2 - y1)
                depth_zone_w = abs(x2 - x1)

                if depth_zone_w > depth_zone_h:
                    half = 0.40 * depth_zone_h
                else:
                    half = 0.40 * depth_zone_w

                half = max(1, min(8, int(np.ceil(half))))

                # ----------- DEPTH -----------
                try:
                    depth_value = find_depth(
                        depth_frame=depth,
                        half=half,
                        bbox_cy=bbox_cy,
                        bbox_cx=bbox_cx,
                        mode=self.mode
                    )
                
                except Exception as e:
                    self.get_logger().warn(f'Depth error for obj {object_id}: {e}')
                    depth_value = None

                if depth_value is None:
                    continue

                # ----------- DIST / ANGLE -----------
                dist_center = find_dist_from_center(bbox_cx, self.mode)

                # ----------- DICT FOR ANGLE BETWEEN -----------

                if object_id == ObjectID.GATE_LEG:
                    gate_legs_detected.append({
                        "depth": depth_value,
                        "bbox_cx": bbox_cx
                    })

                """
                if object_id not in objects or depth_value < objects[object_id]["depth"]:
                    objects[object_id] = {
                        "depth": depth_value,
                        "bbox_cx": bbox_cx
                    }
                """
                # if depth_value < self.depth_threshold:
                payload.extend([float(object_id), depth_value, dist_center])

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

        # ----------- PUBLISH DEPTH DATA -----------
        msg = Float32MultiArray()
        msg.data = payload

        nb_objects = len(payload) // 3
        msg.layout.dim = [
            MultiArrayDimension(label='objects', size=nb_objects, stride=max(len(payload), 1)),
            MultiArrayDimension(label='fields', size=3, stride=3)
        ]
        msg.layout.data_offset = 0

        self.obj_depth_dist_pub.publish(msg)

        # ----------- ANGLE BETWEEN OBJECTS -----------
        dict_leg = self.build_gate_leg_dict(gate_legs_detected, MOVING_MEAN_ACTIVATED) #moving mean calcul
        payload_angle_bet = switch_case_sub_angle(dict_leg, self.mode)
        #objects = self.filter_objects_depth(objects, MOVING_MEAN_ACTIVATED)
        #payload_angle_bet = switch_case_sub_angle(objects, self.mode)

        if len(payload_angle_bet) >= 2 and MOVING_MEAN_ACTIVATED:
            angle = payload_angle_bet[1]

            self.angle_history.append(angle)

            angle_filtered = float(np.median(self.angle_history))

            payload_angle_bet[1] = angle_filtered


        msg_angle = Float32MultiArray()
        msg_angle.data = payload_angle_bet

        nb_objects_angle = len(payload_angle_bet) // 3
        msg_angle.layout.dim = [
            MultiArrayDimension(label='objects', size=nb_objects_angle, stride=max(len(payload_angle_bet), 1)),
            MultiArrayDimension(label='fields', size=3, stride=3)
        ]
        msg_angle.layout.data_offset = 0

        self.region_angle_topic.publish(msg_angle)

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

    def filter_objects_depth(self, objects, activated):
        if not activated:
            return objects

        filtered_objects = {}

        for object_id, obj in objects.items():
            key = str(object_id)

            if key not in self.depth_history:
                self.depth_history[key] = deque(maxlen=10)
                self.last_seen[key] = None

            filtered_depth = self.spike_filter_with_timeout(
                obj["depth"],
                self.depth_history[key],
                key
            )

            self.depth_history[key].append(filtered_depth)

            filtered_objects[object_id] = {
                "depth": float(np.median(self.depth_history[key])),
                "bbox_cx": obj["bbox_cx"]
            }

        return filtered_objects

    def build_gate_leg_dict(self, gate_legs_detected, activated):
        """
        Sort gate legs left/right using their x-position in the camera,
        then smooth left and right depths with a moving average of 10 frames.
        """
        if len(gate_legs_detected) < 2:
            return {}

        # If more than two legs are detected, keep the leftmost and rightmost ones.
        gate_legs_detected = sorted(gate_legs_detected, key=lambda obj: obj["bbox_cx"])
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

    def spike_filter_with_timeout(self, new_value, history, key):
        now = self.get_clock().now().nanoseconds / 1e9

        last_seen = self.last_seen[key]

        if last_seen is None or len(history) == 0:
            self.last_seen[key] = now
            return new_value

        time_since_seen = now - last_seen
        self.last_seen[key] = now

        if time_since_seen > self.reset_after_sec:
            return new_value

        last_value = history[-1]

        if abs(new_value - last_value) > self.spike_threshold_mm:
            return last_value

        return new_value


def main():
    args = parse_args()
    rclpy.init()
    node = YoloNode(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()