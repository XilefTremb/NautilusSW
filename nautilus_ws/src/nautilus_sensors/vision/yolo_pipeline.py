#!/usr/bin/env python3

import argparse
import rclpy
import torch
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int16
from ultralytics import YOLO
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from collections import deque

from vision.object_depth import find_depth, find_dist_from_center, global_median_forward_cam
from vision.gate_angle import find_gate_angle
from vision.filters import TemporalFilter
from vision.display_model_boxes import draw_detection, obb_model_coordinates, bbox_model_coordinates
from vision.edge_detector import find_depth_from_mask, detect_dark_object_in_roi

from enums.ObjectID import ObjectID

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
                '/home/nautilus/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/bbox_sim_640.pt')
        else:
            self.mode = 'real'
            self.model = YOLO('/home/nautilus/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/bbox_competition_14_avril.pt')

        # -------- MODEL TYPE --------
        if args.obb:
            self.type_yolo = 'obb'

        else:
            self.type_yolo = 'bbox'

        # ----------- INIT PARAMS FILTER-----------
        self.depth_threshold = 6000

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

        # ----------- GPU -----------
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
        self.detection_pub = self.create_publisher(Float32MultiArray, '/yolo/detections', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.mean_depth_forward_cam = self.create_publisher(Int16, '/yolo/mean_depth_forward_cam', 10)
        self.edge_mask_pub = self.create_publisher(Image, '/yolo/edge_mask', 10)

        self.get_logger().info(f'YOLOv8 node started, mode: {self.mode}, type: {self.type_yolo}')

    def synced_callback(self, rgb_msg, depth_msg):

        # ----------- FRAME -----------
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')


        # ----------- MODEL -----------
        results = self.model(frame, conf=0.4, verbose=False)
        payload = []

        objects = {}
        slalom_tab = []

        annotated_frame = frame.copy()
        edge_debug = np.zeros(frame.shape[:2], dtype=np.uint8)

        if self.type_yolo == 'obb':
            detection = results[0].obb
        else:
            detection = results[0].boxes

        if detection is not None:
            for box in detection:

                # ----------- BOX INFORMATION-----------
                if self.type_yolo == 'obb':
                    box_cx, box_cy, x1, y1, x2, y2, half, points = obb_model_coordinates(box)
                else:
                    box_cx, box_cy, x1, y1, x2, y2, half = bbox_model_coordinates(box)
                    points = None

                # Clamp to image
                h, w = depth.shape
                x1, x2 = np.clip([x1, x2], 0, w - 1)
                y1, y2 = np.clip([y1, y2], 0, h - 1)

                # ----------- CLASS / CONF -----------
                object_id = int(box.cls[0])
                confidence = float(box.conf[0])

                # ----------- DEPTH -----------
                if self.mode == "real" and (object_id == ObjectID.GATE_LEG_L or object_id == ObjectID.GATE_LEG_CENTER
                or object_id == ObjectID.GATE_LEG_R or object_id == ObjectID.SLALOM_SIDE or object_id == ObjectID.SLALOM_CENTER):

                    roi = frame[y1:y2, x1:x2]

                    if roi.size == 0:
                        continue

                    filled_mask = detect_dark_object_in_roi(roi)

                    edge_debug[y1:y2, x1:x2][filled_mask > 0] = 255

                    depth_value = find_depth_from_mask(
                        depth_frame=depth,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                        mask=filled_mask
                    )

                    annotated_frame[y1:y2, x1:x2][filled_mask > 0] = [0, 0, 255] #for debug

                    if depth_value is None:
                        depth_value = find_depth(
                            depth_frame=depth,
                            half=half,
                            bbox_cy=box_cy,
                            bbox_cx=box_cx,
                            mode=self.mode
                        )

                else:

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
                if int(object_id) == int(ObjectID.SLALOM_SIDE):
                    slalom_tab.append({
                        "depth": depth_value,
                        "dist_center": dist_center,
                        "box_cx": box_cx,
                        "box_cy": box_cy,
                        "confidence": confidence,
                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2,
                        "points": points
                    })
                    continue

                elif object_id not in objects or depth_value < objects[object_id]["depth"]:
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
                    draw_detection(
                        annotated_frame, self.type_yolo, object_id, confidence,
                        depth_value, dist_center,
                        box_cx, box_cy, x1, y1, x2, y2,
                        points)

        # ----------- ANGLE BETWEEN OBJECTS -----------
        objects, payload = self.slalom_organizer(slalom_tab, objects, annotated_frame, payload)
        payload_angle_bet = find_gate_angle(objects, self.mode)

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

        self.detection_pub.publish(msg)

        # ----------- PUBLISH IMAGE OUTPUT -----------
        out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out_msg.header = rgb_msg.header
        self.image_pub.publish(out_msg)

        # ----------- PUBLISH IMAGE MASK DEPTH -----------
        edge_msg = self.bridge.cv2_to_imgmsg(edge_debug, encoding='mono8')
        edge_msg.header = rgb_msg.header
        self.edge_mask_pub.publish(edge_msg)

        # ----------- GLOBAL DEPTH -----------
        depth_global_mean = global_median_forward_cam(depth, self.mode)
        if not depth_global_mean < -32767 and not depth_global_mean > 32767:
            msg_depth = Int16()
            msg_depth.data = int(depth_global_mean)
            self.mean_depth_forward_cam.publish(msg_depth)

    def depth_threshold_callback(self, msg):
        self.depth_threshold = msg.data
        self.get_logger().info(f'Updated depth threshold: {self.depth_threshold}')

    def slalom_organizer(self, slalom_tab, objects, annotated_frame, payload):
        
        if len(slalom_tab) == 0:
            return objects, payload
        
        if ObjectID.SLALOM_CENTER not in objects:
            valid_slaloms = [s for s in slalom_tab if s["depth"] != -1000]

            if valid_slaloms:
                closest_side = min(valid_slaloms, key=lambda s: s["depth"])
            else:
                closest_side = min(slalom_tab, key=lambda s: s["depth"])

            payload.extend([float(ObjectID.SLALOM_SIDE), float(closest_side["dist_center"]), float(closest_side["depth"]),0.0])

            draw_detection(
                annotated_frame,
                self.type_yolo,
                ObjectID.SLALOM_SIDE,
                closest_side["confidence"],
                closest_side["depth"],
                closest_side["dist_center"],
                closest_side["box_cx"],
                closest_side["box_cy"],
                closest_side["x1"],
                closest_side["y1"],
                closest_side["x2"],
                closest_side["y2"],
                closest_side["points"]
            )

            return objects, payload

        slalom_middle_cx = objects[ObjectID.SLALOM_CENTER]["box_cx"]

        slalom_left = None
        slalom_right = None

        for slalom in slalom_tab:
            depth_value = slalom["depth"]
            box_cx = slalom["box_cx"]

            if box_cx < slalom_middle_cx:
                if slalom_left is None:
                    slalom_left = slalom
                elif slalom_left["depth"] == -1000 and depth_value != -1000:
                    slalom_left = slalom
                elif depth_value != -1000 and depth_value < slalom_left["depth"]:
                    slalom_left = slalom

            elif box_cx > slalom_middle_cx:
                if slalom_right is None:
                    slalom_right = slalom
                elif slalom_right["depth"] == -1000 and depth_value != -1000:
                    slalom_right = slalom
                elif depth_value != -1000 and depth_value < slalom_right["depth"]:
                    slalom_right = slalom
                
        if slalom_left is not None:
            objects[ObjectID.SLALOM_LEFT] = slalom_left
            payload.extend([float(ObjectID.SLALOM_LEFT),float(slalom_left["dist_center"]), float(slalom_left["depth"]),0.0])
            draw_detection(
                annotated_frame, self.type_yolo, ObjectID.SLALOM_LEFT,
                slalom_left["confidence"], slalom_left["depth"],
                slalom_left["dist_center"],
                slalom_left["box_cx"], slalom_left["box_cy"],
                slalom_left["x1"], slalom_left["y1"],
                slalom_left["x2"], slalom_left["y2"],
                slalom_left["points"]
            )

        if slalom_right is not None:
            objects[ObjectID.SLALOM_RIGHT] = slalom_right
            payload.extend([float(ObjectID.SLALOM_RIGHT),float(slalom_right["dist_center"]), float(slalom_right["depth"]),0.0])
            draw_detection(
                annotated_frame, self.type_yolo, ObjectID.SLALOM_RIGHT,
                slalom_right["confidence"], slalom_right["depth"],
                slalom_right["dist_center"],
                slalom_right["box_cx"], slalom_right["box_cy"],
                slalom_right["x1"], slalom_right["y1"],
                slalom_right["x2"], slalom_right["y2"],
                slalom_right["points"]
            )

        return objects, payload

    def destroy_node(self):
        self.get_logger().info("Destroying YOLO node")

        if hasattr(self, "model"):
            del self.model

        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        super().destroy_node()


def main():
    args = parse_args()

    rclpy.init()
    node = YoloNode(args)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()