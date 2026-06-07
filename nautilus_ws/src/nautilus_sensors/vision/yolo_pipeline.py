#!/usr/bin/env python3

import argparse
import rclpy
import torch
import time 
import numpy as np
import os

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


# =========================================================
# CONFIG
# =========================================================
MOVING_MEAN_ACTIVATED = True
PREQUALIFICATION = False

FORWARD_CAM_RATE_HZ = 20       
DOWNWARD_CAM_RATE_HZ = 20


def parse_args():
    p = argparse.ArgumentParser()

    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true")
    mode.add_argument("--real", action="store_true")

    model_type = p.add_mutually_exclusive_group(required=True)
    model_type.add_argument("--obb", action="store_true")
    model_type.add_argument("--bbox", action="store_true")

    return p.parse_args()


# =========================================================
# YOLO NODE
# =========================================================
class YoloNode(Node):

    def __init__(self, args):
        super().__init__('yolo_node')

        # -------- MODE --------
        if args.sim:
            self.mode = 'sim'
            model_path = os.path.expanduser('~/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/bbox_sim_640_24_mai.pt')
        else:
            self.mode = 'real'
            model_path = os.path.expanduser('~/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/Model_Realtime_18_mars.pt')

        # -------- MODEL TYPE --------
        self.type_yolo = 'obb' if args.obb else 'bbox'

        # -------- MODEL --------
        self.model = YOLO(model_path)

        if torch.cuda.is_available():
            self.model.to('cuda')
            
        # ----------- INIT PARAMS FILTER-----------
        self.depth_threshold = 99999

        self.bridge = CvBridge()

        # -------- FILTERS --------
        self.temporal_filter = TemporalFilter(self)

        self.angle_history = deque(maxlen=10)
        self.spike_threshold_mm = 1000
        self.reset_after_sec = 2.0

        self.last_filter_time = {}

        # -------- FRAME QUEUE (LOW LATENCY CORE) --------
        self.forward_queue = deque(maxlen=1)
        self.downward_queue = deque(maxlen=1)

        # -------- SUBSCRIBERS --------

        # Forward cam (RGB + depth sync)
        self.fwd_rgb_sub = Subscriber(self, Image, 'oakd/camera/image_raw')
        self.depth_sub = Subscriber(self, Image, 'oakd/camera/depth/image_raw')
        self.down_rgb_sub = self.create_subscription(Image,'oak1/camera/image_raw',self.downward_callback,10)
        self.depth_threshold_sub = self.create_subscription(Int16,'/yolo/depth_threshold',self.depth_threshold_callback,10)

        # ----------- PUBLISHER -----------
        #self.obj_depth_dist_pub = self.create_publisher(Float32MultiArray, '/yolo/obj_depth_dist', 10)
        #self.region_angle_topic = self.create_publisher(Float32MultiArray, '/yolo/obj_angle', 10)
        self.detection_pub = self.create_publisher(Float32MultiArray, '/yolo/detections', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.mean_depth_forward_cam = self.create_publisher(Int16, '/yolo/mean_depth_forward_cam', 10)
        self.edge_mask_pub = self.create_publisher(Image, '/yolo/edge_mask', 10)

        self.ts = ApproximateTimeSynchronizer(
            [self.fwd_rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
            allow_headerless=True
        )
        
        self.ts.registerCallback(self.forward_callback)
       
        # -------- TIMER (MAIN INFERENCE LOOP) --------
        self.timer = self.create_timer(0.01, self.inference_loop)

        self.last_forward_time = 0
        self.last_downward_time = 0

        self.forward_interval = 1.0 / FORWARD_CAM_RATE_HZ
        self.downward_interval = 1.0 / DOWNWARD_CAM_RATE_HZ

        self.get_logger().info("YOLO dual-camera node started")

    # =====================================================
    # CALLBACKS (JUST PUSH DATA)
    # =====================================================
    def forward_callback(self, rgb_msg, depth_msg):
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')

        self.forward_queue.append((frame, depth, rgb_msg.header))
    
    def depth_threshold_callback(self, msg):
        self.depth_threshold = msg.data

    def downward_callback(self, rgb_msg):
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        self.downward_queue.append((frame, rgb_msg.header))

    def depth_threshold_callback(self, msg):
        self.depth_threshold = msg.data
        self.get_logger().info(f'Updated depth threshold: {self.depth_threshold}')

    def inference_loop(self):
        now = time.time()

        # =====================================================
        # 1. PRIORITY: FORWARD CAMERA
        # =====================================================
        if self.forward_queue and (now - self.last_forward_time > self.forward_interval):

            frame, depth, header = self.forward_queue.pop()
            self.forward_queue.clear()
            self.last_forward_time = now

            results = self.model(frame, conf=0.4, verbose=False)
            annotated = frame.copy()

            self.process_forward(results, annotated, depth)

            msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            msg.header = header
            self.fwd_image_pub.publish(msg)

            return  # IMPORTANT: prevent double compute

        # =====================================================
        # 2. DOWNWARD CAMERA (LOW PRIORITY)
        # =====================================================
        if self.downward_queue and (now - self.last_downward_time > self.downward_interval):

            frame, header = self.downward_queue.pop()  # newest frame
            self.downward_queue.clear()               # drop stale frames
            self.last_downward_time = now

            results = self.model(frame, conf=0.4, verbose=False)
            annotated = frame.copy()

            self.process_downward(results, annotated)

            msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            msg.header = header
            self.down_image_pub.publish(msg)

    
    # =====================================================
    # DOWNWARD CAMERA PIPELINE (FULL LOGIC)
    # =====================================================

    def process_downward(self, results, annotated_frame):

        # ----------- MODEL -----------
        results = self.model(frame, conf=0.4, verbose=False)
        payload = []

        detection = results[0].boxes

        if detection is None:
            return

        for box in detection:
            box_cx, box_cy, x1, y1, x2, y2, half = bbox_model_coordinates(box)

            object_id = int(box.cls[0])
            confidence = float(box.conf[0])

            dist_center = find_dist_from_center(box_cx, self.mode)

            payload.extend([
                float(object_id),
                float(dist_center),
                -1.0,   # no depth
                0.0     # no angle
            ])

            draw_detection(
                annotated_frame,
                self.type_yolo,
                object_id,
                confidence,
                0.0,
                dist_center,
                box_cx,
                box_cy,
                x1,
                y1,
                x2,
                y2,
                None
            )


        if payload:
            msg = Float32MultiArray()
            msg.data = payload

            nb_objects = len(payload) // 4

            msg.layout.dim = [
                MultiArrayDimension(label='objects', size=nb_objects, stride=max(len(payload), 1)),
                MultiArrayDimension(label='fields', size=4, stride=4)
            ]
            msg.layout.data_offset = 0

            self.detection_pub.publish(msg)
            
    # =====================================================
    # FORWARD CAMERA PIPELINE (FULL LOGIC)
    # =====================================================
    def process_forward(self, results, annotated_frame, depth):

        now = time.time()

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

                objects[object_id] = {
                    "depth": depth_value,
                    "box_cx": box_cx
                }

            # -------- PAYLOAD + DRAW --------
            if depth_value < self.depth_threshold:

                payload.extend([
                    float(object_id),
                    float(dist_center),
                    float(depth_value),
                    0.0
                ])

                draw_detection(
                    annotated_frame,
                    self.type_yolo,
                    object_id,
                    confidence,
                    depth_value,
                    dist_center,
                    box_cx, box_cy,
                    x1, y1, x2, y2,
                    points
                )

        # -------- SLALOM LOGIC --------
        objects, payload = self.slalom_organizer(
            slalom_tab, objects, annotated_frame, payload
        )

        # -------- ANGLE COMPUTATION --------
        payload_angle = find_gate_angle(objects, self.mode)

        if MOVING_MEAN_ACTIVATED:
            for i in range(0, len(payload_angle), 4):
                group_id = int(payload_angle[i])
                angle_index = i + 3

                payload_angle[angle_index] = self.safe_temporal_filter(
                    key=f"angle_{group_id}",
                    value=payload_angle[angle_index],
                    now=now
                )

        payload.extend(payload_angle)

        # -------- PUBLISH --------
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
            msg_depth.data = int(depth_mean)
            self.mean_depth_forward_cam.publish(msg_depth)

    def safe_temporal_filter(self, key, value, now):
        last_time = self.last_filter_time.get(key, None)

        self.last_filter_time[key] = now

        return self.temporal_filter.moving_median_filter(key, value)

    # =====================================================
    # DOWNWARD CAMERA PIPELINE (MODULAR)
    # =====================================================
    def process_downward(self, results, annotated_frame):

        # RIGHT NOW: same YOLO
        # LATER: replace with line detection / color / etc.

        detection = results[0].boxes

        if detection is None:
            return

        for box in detection:
            box_cx, box_cy, x1, y1, x2, y2, half = bbox_model_coordinates(box)

            object_id = int(box.cls[0])
            confidence = float(box.conf[0])

            draw_detection(
                annotated_frame,
                self.type_yolo,
                object_id,
                confidence,
                0.0,  # no depth
                0.0,
                box_cx,
                box_cy,
                x1,
                y1,
                x2,
                y2,
                None
            )

        # No detection_pub here by default
        # You can create a separate topic later if needed

    def slalom_organizer(self, slalom_tab, objects, annotated_frame, payload):
            
            if len(slalom_tab) == 0:
                return objects, payload
            
            if ObjectID.SLALOM_CENTER not in objects:
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
                    if slalom_left is None or depth_value < slalom_left["depth"]:
                        slalom_left = slalom

                elif box_cx > slalom_middle_cx:
                    if slalom_right is None or depth_value < slalom_right["depth"]:
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


# =========================================================
# MAIN
# =========================================================
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
