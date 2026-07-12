#!/usr/bin/env python3

import argparse
import rclpy
import torch
import time
import threading
import numpy as np
import os

from contextlib import contextmanager
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int32, Int32MultiArray
from ultralytics import YOLO
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from collections import deque

from vision.object_depth import find_depth, find_dist_from_center_in_x, find_dist_from_center_in_y, global_median_forward_cam, find_depth_from_edge_detector, find_torpedo_depth
from vision.gate_angle import find_gate_angle, find_angle_torpedo
from vision.filters import TemporalFilter
from vision.display_model_boxes import draw_detection, obb_model_coordinates, bbox_model_coordinates
from vision.slider_edge_detector import load_params_edge_detector_json
from vision.item_organizer import slalom_organizer, target_organizer

from enums.ObjectID import ObjectID


# =========================================================
# CONFIG
# =========================================================
MOVING_MEAN_ACTIVATED = False
FORWARD_CAM_RATE_HZ = 15
DOWNWARD_CAM_RATE_HZ = 15

COLOR_IN_DETECTION = (0, 255, 0)
COLOR_NOT_IN_DETECTION = (255, 0, 0)

PUBLISH_ANNOTATED_IMAGES = True
ENABLE_DOWNWARD_INFERENCE = True

# -------- INFERENCE TUNING --------
# Detection confidence threshold passed to the model.
INFERENCE_CONF = 0.4
# Run the model in FP16 when a CUDA device is available. Big speedup on GPU with
# negligible accuracy impact for detection. Ignored on CPU. Easy to A/B with the
# profiler by toggling this flag.
USE_HALF_PRECISION = True
# Run one dummy inference at startup so the first real frame isn't slow.
WARMUP_ON_START = True
# Rate at which the main inference loop polls the frame queues. A higher rate
# reduces how long a ready frame waits before being picked up (less latency).
# Only used when USE_INFERENCE_THREAD is False. Kept high (fast poll) so the
# single-threaded timer picks up ready frames almost immediately -- this gives
# the low-latency benefit of a dedicated thread WITHOUT the GIL contention that
# a background inference thread + MultiThreadedExecutor introduces.
INFERENCE_LOOP_HZ = 100
# Run inference in a dedicated background thread instead of a polling timer.
# WARNING: for this CPU-bound Python pipeline (cv_bridge, YOLO postprocess,
# numpy) the GIL serialises Python work anyway, so a background thread just
# fights the executor for the GIL and STARVES the camera callbacks -- observed
# as growing publish intervals over time. Leave this False and rely on a fast
# INFERENCE_LOOP_HZ instead. Only flip to True to A/B test.
USE_INFERENCE_THREAD = False
# Sleep applied by the inference thread when no frame is ready (seconds).
INFERENCE_IDLE_SLEEP_S = 0.001

# -------- PROFILING --------
# When True: measure per-stage run times, publish them on /yolo/profiling
# (consumed by yolo_profiling_viewer.py) and print them in the console.
# When False: all instrumentation is skipped (near-zero overhead) and nothing
# is published on /yolo/profiling.
PROFILING_ENABLED = True
PROFILING_CONSOLE_LOG = True
PROFILING_TOPIC = '/yolo/profiling'

# Fixed stage order shared with yolo_profiling_viewer.py. The published
# Float32MultiArray holds one value (in milliseconds) per stage, in this order.
# Stages not measured for a given frame are published as NaN.
PROFILING_STAGES = [
    "camera",            # 0.0 = forward cam, 1.0 = downward cam
    "image_cam_interval",    # time between two received images for this camera
    "yolo_inference",    # YOLO model forward pass
    "detection_processing", # per-box depth extraction loop
    "angle_computation", # gate/torpedo angle computation
    "process_total",     # full process_forward / process_downward
    "publish",           # detection message publishing
    "detections_forward_interval",           # /yolo/detections_forward publish interval
    "detections_downward_interval",          # /yolo/detections_downward publish interval
    "image_annotated_fwd_cam_interval",      # /yolo/image_annotated_fwd_cam publish interval
    "image_annotated_dwd_cam_interval",      # /yolo/image_annotated_dwd_cam publish interval
    "end_2_end",       # end-to-end for the whole frame
]


class StageProfiler:
    """Lightweight per-stage timing collector.

    Records named time deltas (in milliseconds) for the current frame, keeps a
    rolling history per stage for console stats, and can serialize the current
    frame into the fixed PROFILING_STAGES order for publishing.
    """

    def __init__(self, node, stages, window=100):
        self.node = node
        self.stages = stages
        self.history = {s: deque(maxlen=window) for s in stages}
        self.frame = {}
        self._starts = {}
        self._last_event = {}

    def reset_frame(self):
        self.frame = {}

    def record(self, name, dt_ms):
        if dt_ms is None:
            return
        self.frame[name] = dt_ms
        if name in self.history:
            self.history[name].append(dt_ms)

    @contextmanager
    def span(self, name):
        t = time.perf_counter()
        try:
            yield
        finally:
            self.record(name, (time.perf_counter() - t) * 1000.0)

    def event_interval(self, name):
        """Return ms elapsed since the previous call with the same name."""
        now = time.perf_counter()
        last = self._last_event.get(name)
        self._last_event[name] = now
        if last is None:
            return None
        return (now - last) * 1000.0

    def build_msg_data(self):
        return [float(self.frame.get(s, float('nan'))) for s in self.stages]

    def log_console(self):
        parts = []
        for s in self.stages:
            if s == "camera":
                continue
            v = self.frame.get(s)
            if v is not None:
                parts.append(f"{s}={v:.1f}ms")
        if parts:
            self.node.get_logger().info("[PROFILE] " + "  ".join(parts))


def parse_args():
    p = argparse.ArgumentParser()

    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true")
    mode.add_argument("--real", action="store_true")

    model_type = p.add_mutually_exclusive_group(required=True)
    model_type.add_argument("--obb", action="store_true")
    model_type.add_argument("--bbox", action="store_true")

    return p.parse_args()


class YoloNode(Node):

    def __init__(self, args):
        super().__init__('yolo_node')

        # -------- MODE --------
        if args.sim:
            self.mode = 'sim'
            model_path = os.path.expanduser('~/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/bbox_sim_640_16_juin.pt')
        else:
            self.mode = 'real'
            model_path = os.path.expanduser('~/NautilusSW/nautilus_ws/src/nautilus_sensors/vision/yolo_models/bbox_4juillet_competition.pt')

        # -------- MODEL TYPE --------
        self.type_yolo = 'obb' if args.obb else 'bbox'

        # -------- MODEL LOADING--------
        self.model = YOLO(model_path)

        if torch.cuda.is_available():
            self.model.to('cuda')

        # -------- INFERENCE CONFIG --------
        self.use_half = USE_HALF_PRECISION and torch.cuda.is_available()

        # Warm up the model so the first real frame doesn't pay the lazy-init cost.
        if WARMUP_ON_START:
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self.run_inference(dummy)
            self.get_logger().info(f'Model warmed up (half={self.use_half})')

        # ----------- INIT PARAMS -----------
        self.depth_threshold = 99999
        self.bridge = CvBridge()

        # -------- FILTERS SET UP--------
        self.temporal_filter = TemporalFilter(self)

        self.angle_history = deque(maxlen=10)
        self.spike_threshold_mm = 1000
        self.reset_after_sec = 2.0
        self.last_filter_time = {}

        self.edge_params = load_params_edge_detector_json()
        self.get_logger().info(f'edege_params: {self.edge_params}')


        self.last_forward_sync_ts = None
        self.last_forward_sync_count = 0
        self.last_forward_publish_ts = None

        # -------- FRAME QUEUE (LOW LATENCY CORE) --------
        self.forward_queue = deque(maxlen=1)
        self.downward_queue = deque(maxlen=1)

        # -------- SUBSCRIBERS --------
        self.fwd_rgb_sub = Subscriber(self, Image, 'oakd/camera/image_raw')
        self.depth_sub = Subscriber(self, Image, 'oakd/camera/depth/image_raw')
        self.edge_params_sub = self.create_subscription(Int32MultiArray, "/yolo/edge_params", self.edge_params_callback,10)
        self.down_rgb_sub = self.create_subscription(Image,'oak1/camera/image_raw',self.downward_callback, 1)
        self.depth_threshold_sub = self.create_subscription(Int32,'/yolo/depth_threshold',self.depth_threshold_callback, 1)

        # ----------- PUBLISHER -----------
        self.detection_forward_pub = self.create_publisher(Float32MultiArray, '/yolo/detections_forward', 1)
        self.detection_downward_pub = self.create_publisher(Float32MultiArray, '/yolo/detections_downward', 1)
        self.down_image_pub = self.create_publisher(Image, '/yolo/image_annotated_dwd_cam', 1)
        self.fwd_image_pub = self.create_publisher(Image, '/yolo/image_annotated_fwd_cam', 1)
        self.mean_depth_forward_cam = self.create_publisher(Int32, '/yolo/mean_depth_forward_cam', 1)
        self.edge_mask_pub = self.create_publisher(Image, '/yolo/edge_mask', 1)

        # ----------- PROFILING -----------
        self.profiler = StageProfiler(self, PROFILING_STAGES) if PROFILING_ENABLED else None
        self.profiling_pub = (
            self.create_publisher(Float32MultiArray, PROFILING_TOPIC, 1)
            if PROFILING_ENABLED else None
        )
        self.last_forward_image_cam_interval = None
        self.last_downward_image_cam_interval = None

        # ----------- SYNCHRONIZER DEPTH AND RGB -----------
        self.ts = ApproximateTimeSynchronizer(
            [self.fwd_rgb_sub, self.depth_sub],
            queue_size= 5,
            slop= 0.2,
            allow_headerless=True
        )

        self.ts.registerCallback(self.forward_callback)

        self.last_forward_time = 0
        self.last_downward_time = 0

        self.forward_interval = 1.0 / FORWARD_CAM_RATE_HZ
        self.downward_interval = 1.0 / DOWNWARD_CAM_RATE_HZ

        # -------- INFERENCE DRIVER (DEDICATED THREAD OR POLLING TIMER) --------
        self.timer = None
        self._inference_running = False
        self._inference_thread = None
        if USE_INFERENCE_THREAD:
            self._inference_running = True
            self._inference_thread = threading.Thread(
                target=self.inference_worker, daemon=True
            )
            self._inference_thread.start()
        else:
            self.timer = self.create_timer(1.0 / INFERENCE_LOOP_HZ, self.inference_loop)

        self.get_logger().info(
            f'YOLOv8 node started, mode: {self.mode}, type: {self.type_yolo}, '
            f'driver: {"thread" if USE_INFERENCE_THREAD else "timer"}'
        )

    # -------- CALLBACKS --------
    def forward_callback(self, rgb_msg, depth_msg):
        # CALLBACK FOR FORWARD CAM (OAKD)
        now = time.time()

        if self.profiler is not None:
            self.last_forward_image_cam_interval = self.profiler.event_interval("forward_image_cam_interval")

        # if self.last_forward_sync_ts is not None:
        #     interval = now - self.last_forward_sync_ts
        #     self.get_logger().info(
        #         f"[SYNC] interval={interval:.3f}s queue_len={len(self.forward_queue)}"
        #     )
        # else:
        #     self.get_logger().info(
        #         f"[SYNC] first sync queue_len={len(self.forward_queue)}"
        #     )

        # self.last_forward_sync_ts = now
        # self.last_forward_sync_count += 1

        # if self.forward_queue:
        #     self.get_logger().warning(
        #         f"[SYNC_DROP] Forward queue overwritten at sync #{self.last_forward_sync_count}"
        #     )

        self.forward_queue.append((rgb_msg, depth_msg))

    def downward_callback(self, rgb_msg):
        # CALLBACK FOR FORWARD CAM (OAK1)
        if self.profiler is not None:
            self.last_downward_image_cam_interval = self.profiler.event_interval("downward_image_cam_interval")
        self.downward_queue.append(rgb_msg)

    def depth_threshold_callback(self, msg):
        # CALLBACK FOR DEPTH THRESHOLD
        self.depth_threshold = msg.data
        self.get_logger().info(f'Updated depth threshold: {self.depth_threshold}')

    def edge_params_callback(self, msg):
        if len(msg.data) < 6:
            return

        self.edge_params["dark_threshold"] = msg.data[0]
        self.edge_params["light_min_brightness"] = msg.data[1]
        self.edge_params["light_bright_percentile"] = msg.data[2]
        self.edge_params["light_min_brightness_torpedo"] = msg.data[3]
        self.edge_params["light_bright_percentile_torpedo"] = msg.data[4]
        self.edge_params["min_pixel_count"] = msg.data[5]

        self.get_logger().info(f'EDGE PARAMS UPDATED: {self.edge_params}')
        #print("EDGE PARAMS UPDATED", self.edge_params)

    def run_inference(self, frame):
        # CENTRALIZED MODEL CALL (applies inference tuning config)
        kwargs = {"conf": INFERENCE_CONF, "verbose": False}
        if self.use_half:
            kwargs["half"] = True
        return self.model(frame, **kwargs)

    def inference_worker(self):
        # DEDICATED INFERENCE THREAD: process frames back-to-back with no timer
        # quantization. Sleeps briefly only when no frame is ready.
        while self._inference_running and rclpy.ok():
            try:
                did_work = self.inference_loop()
            except Exception as exc:  # keep the thread alive on transient errors
                self.get_logger().error(f"Inference worker error: {exc}")
                did_work = False

            if not did_work:
                time.sleep(INFERENCE_IDLE_SLEEP_S)

    def inference_loop(self):
        now = time.time()

        #PRIORITY1: FORWARD CAMERA
        if self.forward_queue and (now - self.last_forward_time > self.forward_interval):

            frame_t0 = time.perf_counter()
            if self.profiler is not None:
                self.profiler.reset_frame()
                self.profiler.record("camera", 0.0)
                self.profiler.record("image_cam_interval", self.last_forward_image_cam_interval)

            rgb_msg, depth_msg = self.forward_queue.pop()
            self.forward_queue.clear()

            frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            header = rgb_msg.header

            self.last_forward_time = now

            t0 = time.time()
            if self.profiler is not None:
                with self.profiler.span("yolo_inference"):
                    results = self.run_inference(frame)
            else:
                results = self.run_inference(frame)
            t1 = time.time()

            if self.profiler is not None:
                with self.profiler.span("process_total"):
                    annotated_frame, edge_debug = self.process_forward(results, frame, depth)
            else:
                annotated_frame, edge_debug = self.process_forward(results, frame, depth)
            t2 = time.time()

            #self.get_logger().info(f"YOLO={t1-t0:.3f}s PROCESS={t2-t1:.3f}s TOTAL={t2-t0:.3f}s")

            # ----------- PUBLISH IMAGE ANNOTATED OAKD -----------
            if PUBLISH_ANNOTATED_IMAGES:
                out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                out_msg.header = header
                self.fwd_image_pub.publish(out_msg)

                if self.profiler is not None:
                    self.profiler.record("image_annotated_fwd_cam_interval", self.profiler.event_interval("image_annotated_fwd_cam_interval"))

                # ----------- PUBLISH IMAGE MASK DEPTH -----------
                edge_msg = self.bridge.cv2_to_imgmsg(edge_debug, encoding='mono8')
                edge_msg.header = header
                self.edge_mask_pub.publish(edge_msg)

            # ----------- GLOBAL DEPTH -----------
            depth_global_mean = global_median_forward_cam(depth, self.mode, "forward")
            if depth_global_mean is not None:
                msg_depth = Int32()
                msg_depth.data = int(depth_global_mean)
                self.mean_depth_forward_cam.publish(msg_depth)

            if self.profiler is not None:
                self.profiler.record("end_2_end", (time.perf_counter() - frame_t0) * 1000.0)
                self.publish_profiling()

            return True

        #PRIORITY2: DOWNWARD CAMERA
        if ENABLE_DOWNWARD_INFERENCE and self.downward_queue and (now - self.last_downward_time > self.downward_interval):

            frame_t0 = time.perf_counter()
            if self.profiler is not None:
                self.profiler.reset_frame()
                self.profiler.record("camera", 1.0)
                self.profiler.record("image_cam_interval", self.last_downward_image_cam_interval)

            rgb_msg = self.downward_queue.pop()
            self.downward_queue.clear()

            frame = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            header = rgb_msg.header

            self.last_downward_time = now

            if self.profiler is not None:
                with self.profiler.span("yolo_inference"):
                    results = self.run_inference(frame)
                with self.profiler.span("process_total"):
                    self.process_downward(results, frame)
            else:
                results = self.run_inference(frame)
                self.process_downward(results, frame)

            # ----------- PUBLISH IMAGE ANNOTATED DOWNWARD CAM -----------
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header = header
            self.down_image_pub.publish(msg)

            if self.profiler is not None:
                self.profiler.record("image_annotated_dwd_cam_interval", self.profiler.event_interval("image_annotated_dwd_cam_interval"))
                self.profiler.record("end_2_end", (time.perf_counter() - frame_t0) * 1000.0)
                self.publish_profiling()

            return True

        return False

    def publish_profiling(self):
        # PUBLISH + OPTIONALLY LOG PER-STAGE RUN TIMES
        if self.profiler is None:
            return

        if self.profiling_pub is not None:
            msg = Float32MultiArray()
            msg.data = self.profiler.build_msg_data()
            msg.layout.dim = [
                MultiArrayDimension(
                    label='stages',
                    size=len(PROFILING_STAGES),
                    stride=len(PROFILING_STAGES))
            ]
            msg.layout.data_offset = 0
            self.profiling_pub.publish(msg)

        if PROFILING_CONSOLE_LOG:
            self.profiler.log_console()


    def process_downward(self, results, annotated_frame):
    # DOWNWARD CAMERA PIPELINE

        if self.type_yolo == 'obb':
            detection = results[0].obb
        else:
            detection = results[0].boxes

        payload = []

        if detection is not None:
            for box in detection:

                # ----------- BOX INFORMATION-----------
                if self.type_yolo == 'obb':
                    box_cx, box_cy, x1, y1, x2, y2, half, points = obb_model_coordinates(box)
                else:
                    box_cx, box_cy, x1, y1, x2, y2, half = bbox_model_coordinates(box)
                    points = None

                # ----------- CLASS / CONF -----------
                object_id = int(box.cls[0])
                confidence = float(box.conf[0])

                dist_center_x = find_dist_from_center_in_x(box_cx, self.mode, "downward")
                dist_center_y = find_dist_from_center_in_y(box_cy, self.mode, "downward")

                payload.extend([float(object_id), float(dist_center_x), -1.0, 0.0, float(dist_center_y)])

                # -------- PAYLOAD + DRAW --------
                draw_detection(
                    annotated_frame,
                    self.type_yolo,
                    object_id,
                    confidence,
                    0, dist_center_x,
                    box_cx, box_cy,
                    x1, y1, x2, y2,
                    None
                )

            if payload:
                msg = Float32MultiArray()
                msg.data = payload

                nb_objects = len(payload) // 5

                msg.layout.dim = [
                    MultiArrayDimension(label='objects', size=nb_objects, stride=max(len(payload), 1)),
                    MultiArrayDimension(label='fields', size=5, stride=5)
                ]
                msg.layout.data_offset = 0

                #self.get_logger().info(f"[PUBLISHED] Downward: {nb_objects} objects detected")
                self.detection_downward_pub.publish(msg)

                if self.profiler is not None:
                    self.profiler.record("detections_downward_interval", self.profiler.event_interval("detections_downward_interval"))
            else:
                self.get_logger().info(f"[EMPTY] Downward: No detections found")


    def process_forward(self, results, annotated_frame, depth):
    # FORWARD CAMERA PIPELINE

        # ----------- INIT VARIABLE -----------
        now = time.time()

        payload = []
        objects = {}
        slalom_tab = []
        target_tab = []

        edge_debug = np.zeros(annotated_frame.shape[:2], dtype=np.uint8)

        if self.type_yolo == 'obb':
            detection = results[0].obb
        else:
            detection = results[0].boxes

        depth_loop_t0 = time.perf_counter() if self.profiler is not None else None

        if detection is not None:
            for box in detection:

                # ----------- BOX INFORMATION-----------
                if self.type_yolo == 'obb':
                    box_cx, box_cy, x1, y1, x2, y2, half, points = obb_model_coordinates(box)
                else:
                    box_cx, box_cy, x1, y1, x2, y2, half = bbox_model_coordinates(box)
                    points = None

                #----------- BOX INSIDE FRAME - ----------
                h, w = depth.shape
                x1 = int(np.clip(x1, 0, w - 1))
                x2 = int(np.clip(x2, 0, w - 1))
                y1 = int(np.clip(y1, 0, w - 1))
                y2 = int(np.clip(y2, 0, w - 1))

                # ----------- CLASS / CONF -----------
                object_id = int(box.cls[0])
                confidence = float(box.conf[0])

                # ----------- DEPTH -----------
                if ((object_id == ObjectID.GATE_LEG_L or
                         object_id == ObjectID.GATE_LEG_CENTER or
                         object_id == ObjectID.GATE_LEG_R or
                         object_id == ObjectID.SLALOM_SIDE or
                         object_id == ObjectID.SLALOM_CENTER or
                         object_id == ObjectID.DROPPER)):

                    depth_value, filled_mask = find_depth_from_edge_detector(
                        depth_frame=depth,
                        x1=x1,
                        y1=y1,
                        x2=x2,
                        y2=y2,
                        annotated_frame = annotated_frame,
                        id = object_id,
                        edge_params = self.edge_params,
                        mode = self.mode)

                    if filled_mask is not None:
                        annotated_frame[y1:y2, x1:x2][filled_mask > 0] = [0, 0, 255] #for debug
                        edge_debug[y1:y2, x1:x2][filled_mask > 0] = 255

                    if depth_value is None:
                        depth_value = find_depth(
                            depth_frame=depth,
                            half=half,
                            bbox_cy=box_cy,
                            bbox_cx=box_cx,
                            mode=self.mode
                        )

                else:
                     depth_value = find_depth(
                            depth_frame=depth,
                            half=half,
                            bbox_cy=box_cy,
                            bbox_cx=box_cx,
                            mode=self.mode)

                if depth_value is None or not (400.0 < depth_value < self.depth_threshold):
                    #self.get_logger().info(f"[FILTERED] Object {object_id} depth={depth_value} outside range (400-{self.depth_threshold})")
                    continue

                # ----------- DIST / ANGLE -----------
                dist_center_x = find_dist_from_center_in_x(box_cx, self.mode, "forward")
                dist_center_y = find_dist_from_center_in_y(box_cy, self.mode, "forward")

                # ----------- DICT FOR ANGLE BETWEEN -----------
                if object_id == ObjectID.SLALOM_SIDE:
                    slalom_tab.append({
                        "depth": depth_value,
                        "dist_center_x": dist_center_x,
                        "dist_center_y": dist_center_y,
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

                elif object_id == ObjectID.TARGET:
                    target_tab.append({
                        "depth": depth_value,
                        "dist_center_x": dist_center_x,
                        "dist_center_y": dist_center_y,
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
                    if object_id in objects:
                        draw_detection(
                            annotated_frame,
                            self.type_yolo,
                            object_id,
                            objects[object_id]["confidence"],
                            objects[object_id]["depth"],
                            objects[object_id]["dist_center_x"],
                            objects[object_id]["box_cx"],
                            objects[object_id]["box_cy"],
                            objects[object_id]["x1"],
                            objects[object_id]["y1"],
                            objects[object_id]["x2"],
                            objects[object_id]["y2"],
                            color= COLOR_NOT_IN_DETECTION,
                            points=objects[object_id]["points"]
                        )

                    if MOVING_MEAN_ACTIVATED:
                            depth_value = self.temporal_filter.moving_median_filter(
                                key=f"depth_{object_id}",
                                new_value=depth_value)
                            
                    objects[object_id] = {
                        "depth": depth_value,
                        "box_cx": box_cx,
                        "box_cy": box_cy,
                        "dist_center_x": dist_center_x,
                        "dist_center_y": dist_center_y,
                        "confidence": confidence,
                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2,
                        "points": points}

                else:
                    draw_detection(
                        annotated_frame,
                        self.type_yolo,
                        object_id,
                        confidence,
                        depth_value,
                        dist_center_x,
                        box_cx, box_cy,
                        x1, y1, x2, y2,
                        color=COLOR_NOT_IN_DETECTION,
                        points=points
                    )

        if self.profiler is not None:
            self.profiler.record("detection_processing", (time.perf_counter() - depth_loop_t0) * 1000.0)

        for object_id, obj in objects.items():
            if obj["depth"] < self.depth_threshold:
                angle = 0.0
                if object_id == ObjectID.TORPEDO:
                    angle = find_angle_torpedo(objects)

                    torpedo_depth = find_torpedo_depth(
                        objects=objects,
                        fallback_depth=obj["depth"])

                    payload.extend([float(object_id), float(obj["dist_center_x"]), float(torpedo_depth), float(angle), float(obj["dist_center_y"])])
                else:
                    payload.extend([float(object_id),float(obj["dist_center_x"]),float(obj["depth"]),angle, float(obj["dist_center_y"])])


                draw_detection(
                    annotated_frame,
                    self.type_yolo,
                    object_id,
                    obj["confidence"],
                    obj["depth"],
                    obj["dist_center_x"],
                    obj["box_cx"],
                    obj["box_cy"],
                    obj["x1"],
                    obj["y1"],
                    obj["x2"],
                    obj["y2"],
                    color= COLOR_IN_DETECTION,
                    points=obj["points"]
                )

        # -------- SLALOM AND TARGET ORGANIZER --------
        objects, payload = slalom_organizer(slalom_tab, objects, annotated_frame, payload, self.type_yolo)

        objects, payload = target_organizer(target_tab, objects, annotated_frame, payload, self.type_yolo)

        # -------- FIND ANGLE BETWEEN TWO OBJECTS --------
        if self.profiler is not None:
            with self.profiler.span("angle_computation"):
                payload_angle = find_gate_angle(objects, self.mode)
        else:
            payload_angle = find_gate_angle(objects, self.mode)

        if MOVING_MEAN_ACTIVATED:
            for i in range(0, len(payload_angle), 5):
                group_id = int(payload_angle[i])
                angle_index = i + 3

                if group_id == int(ObjectID.SLALOM_CENTER):
                    continue

                payload_angle[angle_index] = self.temporal_filter.moving_median_filter(
                    key=f"angle_{group_id}",
                    new_value=payload_angle[angle_index])

        payload.extend(payload_angle)

        # -------- PUBLISH DETECTION --------
        publish_t0 = time.perf_counter() if self.profiler is not None else None

        msg = Float32MultiArray()
        msg.data = payload
        nb_objects = len(payload) // 5

        msg.layout.dim = [
            MultiArrayDimension(label='objects', size=nb_objects, stride=max(len(payload), 1)),
            MultiArrayDimension(label='fields', size=5, stride=5)]
        msg.layout.data_offset = 0

        if payload:
            now = time.time()
            if self.last_forward_publish_ts is not None:
                gap = now - self.last_forward_publish_ts
                if gap > 0.15:
                    #self.get_logger().warning(
                    #    f"[PUBLISH_GAP] {gap:.3f}s since last forward publish"
                    #)
                    pass
            self.last_forward_publish_ts = now
            #self.get_logger().info(f"[PUBLISHED] Forward: {nb_objects} objects detected")

        self.detection_forward_pub.publish(msg)

        if self.profiler is not None:
            self.profiler.record("publish", (time.perf_counter() - publish_t0) * 1000.0)
            self.profiler.record("detections_forward_interval", self.profiler.event_interval("detections_forward_interval"))

        return annotated_frame, edge_debug

    def destroy_node(self):
        self.get_logger().info("Destroying YOLO node")

        # Stop the inference thread before tearing down the model.
        self._inference_running = False
        if self._inference_thread is not None:
            self._inference_thread.join(timeout=2.0)

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

    # Single-threaded on purpose: this pipeline is CPU/GIL bound, so cooperative
    # single-threaded scheduling (callbacks + inference taking clean turns)
    # outperforms a MultiThreadedExecutor, which only adds GIL contention.
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
