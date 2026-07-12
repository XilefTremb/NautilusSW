#!/usr/bin/env python3

import contextlib
import os
import select
import sys
import threading
import time
from datetime import timedelta

import cv2
import depthai as dai
import gi
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, Imu

from vision.blue_filter import blue_filter

gi.require_version("Gst", "1.0")
from gi.repository import Gst


# =========================================================
# CONFIG
# =========================================================
UDP_IP = "192.168.1.10"
UDP_PORT = 5600
FPS = 15
SAVE_INTERVAL = 2.0

START_BLUE_FILTER = False
START_DEPTH_COLOR = True
START_OVERLAY = True

# Set True only when debugging camera timing.
LOG_CAMERA_TIMING = False

# If True, raw depth debug queue is drained and logged.
# Leave False in normal runtime because the queue is only for diagnostics.
LOG_RAW_DEPTH_TIMING = False

# DOWNWARD CAM SPECS
DOWNWARD_CAMERA_INDEX = 0
DOWNWARD_CAMERA_WIDTH = 1280
DOWNWARD_CAMERA_HEIGHT = 720
DOWNWARD_CAMERA_FPS = 8
DOWNWARD_ROTATE_180 = False

SAVE_DIR = os.path.expanduser("~/Documents/dataset")
RGB_OAKD_DIR = os.path.join(SAVE_DIR, "rgb_oakd")
RGB_OAK1_DIR = os.path.join(SAVE_DIR, "rgb_oak1")
DEPTH_DIR = os.path.join(SAVE_DIR, "depth")

os.makedirs(RGB_OAKD_DIR, exist_ok=True)
os.makedirs(RGB_OAK1_DIR, exist_ok=True)
os.makedirs(DEPTH_DIR, exist_ok=True)


# =========================================================
# ROS2 NODE
# =========================================================
class DualOakNode(Node):
    def __init__(self):
        super().__init__("dual_oak_node")

        self.get_logger().info("Starting Dual OAK ROS2 Node...")

        self.declare_parameter("save_images", False)
        self.save_images = self.get_parameter("save_images").value
        self.get_logger().info(f"save_images: {self.save_images}")

        self.active_stream = "oakd"
        self.bridge = CvBridge()

        # ROS Publishers
        self.rgb_pub = self.create_publisher(Image, "/oakd/camera/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/oakd/camera/depth/image_raw", 10)
        self.overlay_pub = self.create_publisher(Image, "/oakd/camera/rgb_depth_overlay", 10)
        self.depth_color_pub = self.create_publisher(Image, "/oakd/camera/depth/color", 10)
        self.imu_pub = self.create_publisher(Imu, "/oakd/imu/data_raw", 10)
        self.rpy_pub = self.create_publisher(Vector3, "/oakd/imu/rpy", 10)
        self.rgb1_pub = self.create_publisher(Image, "/oak1/camera/image_raw", 10)

        # Shared latest-frame state
        self.latest_lock = threading.Lock()
        self.rgb_oakd_latest = None
        self.rgb_oak1_latest = None
        self.depth_latest = None
        self.overlay_latest = None
        self.pending_viz_rgb = None
        self.pending_viz_depth = None

        self.last_save_time = time.time()
        self.last_downward_warn_time = 0.0

        self.shutdown_event = threading.Event()
        self.worker_threads = []

        # GStreamer init
        Gst.init(None)
        self.setup_gstreamer()

        # Devices
        self.stack = contextlib.ExitStack()
        self.devices_data = []
        self.oakd_dev = None
        self.downward_dev = None
        self.usb_cap = None
        self.setup_devices()

        # Start workers
        self.start_thread(self.keyboard_listener, "keyboard_listener")

        if self.oakd_dev is not None:
            self.start_thread(self.oakd_worker, "oakd_worker")
            self.start_thread(self.h264_worker, "h264_worker")

        if self.downward_dev is not None:
            self.start_thread(self.downward_worker, "downward_worker")

        # Slower, non-critical timers. These run in ROS, but they do NOT read OAK-D queues.
        self.viz_timer = self.create_timer(0.10, self.visualization_loop)  # 10 Hz max
        self.save_timer = self.create_timer(0.25, self.save_loop)          # check 4 Hz

    # =====================================================
    # THREAD HELPERS
    # =====================================================
    def start_thread(self, target, name):
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self.worker_threads.append(thread)

    # =====================================================
    # KEYBOARD CONTROL
    # =====================================================
    def keyboard_listener(self):
        self.get_logger().info("Press 's' to switch camera stream")

        while not self.shutdown_event.is_set():
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)

                    if key == "s":
                        self.active_stream = "oak1" if self.active_stream == "oakd" else "oakd"
                        self.get_logger().info(f"Switched UDP stream to: {self.active_stream}")
            except Exception as exc:
                self.get_logger().warn(f"Keyboard listener stopped: {exc}")
                return

    # =====================================================
    # GSTREAMER
    # =====================================================
    def setup_gstreamer(self):
        pipeline_str = (
            "appsrc name=src is-live=true do-timestamp=true format=time "
            "block=false max-buffers=4 ! "
            "queue leaky=downstream max-size-buffers=4 ! "
            "h264parse config-interval=1 ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={UDP_IP} port={UDP_PORT} sync=false async=false"
        )

        self.gst_pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.gst_pipeline.get_by_name("src")

        self.appsrc.set_property(
            "caps",
            Gst.Caps.from_string(
                f"video/x-h264,stream-format=(string)byte-stream,"
                f"alignment=(string)au,framerate={FPS}/1"
            ),
        )

        self.gst_pipeline.set_state(Gst.State.PLAYING)

    # =====================================================
    # QUEUE HELPERS
    # =====================================================
    def get_latest(self, queue):
        latest = None

        while True:
            pkt = queue.tryGet()
            if pkt is None:
                break
            latest = pkt

        return latest

    def drain_and_log_periods(self, queue, name):
        count = 0
        prev_ts = getattr(self, f"last_{name}_device_ts", None)

        while True:
            pkt = queue.tryGet()
            if pkt is None:
                break

            count += 1
            ts = pkt.getTimestampDevice()

            if prev_ts is not None:
                period_ms = (ts - prev_ts).total_seconds() * 1000.0
                self.get_logger().warn(f"{name} packet period = {period_ms:.1f} ms")

            prev_ts = ts

        if count > 1:
            self.get_logger().warn(f"{name} drained {count} packets this loop")

        setattr(self, f"last_{name}_device_ts", prev_ts)

    # =====================================================
    # PIPELINES
    # =====================================================
    def create_oakd_pipeline(self, pipeline):
        RGB_SOCKET = dai.CameraBoardSocket.CAM_A
        LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
        RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C

        RGB_SIZE = (1280, 960)
        MONO_SIZE = (640, 400)

        platform = pipeline.getDefaultDevice().getPlatform()

        camRgb = pipeline.create(dai.node.Camera).build(RGB_SOCKET)
        left = pipeline.create(dai.node.Camera).build(LEFT_SOCKET)
        right = pipeline.create(dai.node.Camera).build(RIGHT_SOCKET)

        stereo = pipeline.create(dai.node.StereoDepth)
        sync = pipeline.create(dai.node.Sync)

        align = None
        if platform == dai.Platform.RVC4:
            align = pipeline.create(dai.node.ImageAlign)

        # Original functionality preserved.
        # If you want the lighter tested config, set ExtendedDisparity(False).
        stereo.setExtendedDisparity(True)
        stereo.setLeftRightCheck(True)
        stereo.setRectification(True)

        sync.setSyncThreshold(timedelta(seconds=1 / (2 * FPS)))

        rgb_out = camRgb.requestOutput(
            size=RGB_SIZE,
            fps=FPS,
            enableUndistortion=True,
            type=dai.ImgFrame.Type.NV12,
            resizeMode=dai.ImgResizeMode.STRETCH,
        )

        left_out = left.requestOutput(size=MONO_SIZE, fps=FPS)
        right_out = right.requestOutput(size=MONO_SIZE, fps=FPS)

        left_out.link(stereo.left)
        right_out.link(stereo.right)

        rgb_out.link(sync.inputs["rgb"])

        if platform == dai.Platform.RVC4:
            stereo.depth.link(align.input)
            rgb_out.link(align.inputAlignTo)
            align.outputAligned.link(sync.inputs["depth_aligned"])
        else:
            stereo.depth.link(sync.inputs["depth_aligned"])
            rgb_out.link(stereo.inputAlignTo)

        # H264 UDP stream path, kept separate from ROS RGB/depth path.
        manip = pipeline.create(dai.node.ImageManip)
        manip.setMaxOutputFrameSize(2_000_000)
        manip.initialConfig.addRotateDeg(180)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
        rgb_out.link(manip.inputImage)

        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
        enc.setBitrate(7_000_000)
        enc.setKeyframeFrequency(FPS)
        manip.out.link(enc.input)

        sync_queue = sync.out.createOutputQueue(maxSize=4, blocking=False)
        raw_depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
        h264_queue = enc.bitstream.createOutputQueue(maxSize=4, blocking=False)

        # IMU Data OAKD
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, 200)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, 200)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 200)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        imu_queue = imu.out.createOutputQueue(maxSize=20, blocking=False)

        return sync_queue, raw_depth_queue, h264_queue, imu_queue

    # =====================================================
    # DEVICE SETUP
    # =====================================================
    def setup_devices(self):
        # Downward USB camera
        self.usb_cap = cv2.VideoCapture(DOWNWARD_CAMERA_INDEX, cv2.CAP_V4L2)

        if self.usb_cap.isOpened():
            self.usb_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.usb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, DOWNWARD_CAMERA_WIDTH)
            self.usb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DOWNWARD_CAMERA_HEIGHT)
            self.usb_cap.set(cv2.CAP_PROP_FPS, DOWNWARD_CAMERA_FPS)

            self.downward_dev = {"type": "downward_usb", "cap": self.usb_cap}
            self.devices_data.append(self.downward_dev)
            self.get_logger().info("Downward USB camera opened")
        else:
            self.get_logger().error(f"Could not open /dev/video{DOWNWARD_CAMERA_INDEX}")

        # OAK camera
        device_infos = dai.Device.getAllAvailableDevices()
        self.get_logger().info(f"Found devices: {len(device_infos)}")

        for _device_info in device_infos:
            pipeline = self.stack.enter_context(dai.Pipeline())
            device = pipeline.getDefaultDevice()
            cameras = device.getConnectedCameras()

            if len(cameras) > 1:
                sync_q, depth_q, h264_q, imu_q = self.create_oakd_pipeline(pipeline)
                pipeline.start()

                self.oakd_dev = {
                    "type": "oakd",
                    "sync": sync_q,
                    "raw_depth": depth_q,
                    "h264": h264_q,
                    "imu": imu_q,
                }
                self.devices_data.append(self.oakd_dev)
                break

    # =====================================================
    # CRITICAL OAK-D WORKER
    # =====================================================
    def oakd_worker(self):
        """Fast path: read OAK-D sync + IMU queues and publish ROS RGB/depth.

        This must stay lightweight. Do not run depth colormap, overlay, USB camera,
        saving, or GStreamer in this thread.
        """
        dev = self.oakd_dev

        while not self.shutdown_event.is_set() and rclpy.ok():
            try:
                if LOG_RAW_DEPTH_TIMING:
                    self.drain_and_log_periods(dev["raw_depth"], "raw_depth")

                sync_pkt = self.get_latest(dev["sync"])
                imu_pkt = self.get_latest(dev["imu"])

                if sync_pkt is not None:
                    self.handle_oakd_sync_packet(sync_pkt)

                if imu_pkt is not None:
                    self.handle_imu_packet(imu_pkt)

                time.sleep(0.001)

            except Exception as exc:
                self.get_logger().error(f"OAK-D worker error: {exc}")
                time.sleep(0.05)

    def handle_oakd_sync_packet(self, sync_pkt):
        rgb_msg = sync_pkt["rgb"]
        depth_msg = sync_pkt["depth_aligned"]

        if LOG_CAMERA_TIMING:
            self.log_sync_timing(rgb_msg, depth_msg)

        frame_rgb = rgb_msg.getCvFrame()
        frame_depth = depth_msg.getFrame()

        frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
        frame_depth = cv2.rotate(frame_depth, cv2.ROTATE_180)

        if START_BLUE_FILTER:
            frame_rgb = blue_filter(frame_rgb)

        now_stamp = self.get_clock().now().to_msg()

        rgb_ros_msg = self.bridge.cv2_to_imgmsg(frame_rgb, "bgr8")
        rgb_ros_msg.header.stamp = now_stamp
        rgb_ros_msg.header.frame_id = "oakd_rgb_frame"
        self.rgb_pub.publish(rgb_ros_msg)

        depth_ros_msg = self.bridge.cv2_to_imgmsg(frame_depth, "16UC1")
        depth_ros_msg.header.stamp = now_stamp
        depth_ros_msg.header.frame_id = "oakd_depth_frame"
        self.depth_pub.publish(depth_ros_msg)

        with self.latest_lock:
            self.rgb_oakd_latest = frame_rgb
            self.depth_latest = frame_depth
            self.pending_viz_rgb = frame_rgb
            self.pending_viz_depth = frame_depth

    def handle_imu_packet(self, imu_pkt):
        for packet in imu_pkt.packets:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "oakd_imu_frame"

            accel = packet.acceleroMeter
            gyro = packet.gyroscope

            imu_msg.linear_acceleration.x = accel.x
            imu_msg.linear_acceleration.y = accel.y
            imu_msg.linear_acceleration.z = accel.z

            imu_msg.angular_velocity.x = gyro.x
            imu_msg.angular_velocity.y = gyro.y
            imu_msg.angular_velocity.z = gyro.z

            rot = packet.rotationVector
            imu_msg.orientation.x = rot.i
            imu_msg.orientation.y = rot.j
            imu_msg.orientation.z = rot.k
            imu_msg.orientation.w = rot.real

            self.imu_pub.publish(imu_msg)

            r = R.from_quat([rot.i, rot.j, rot.k, rot.real])
            roll, pitch, yaw = r.as_euler("xyz", degrees=True)

            rpy_msg = Vector3()
            rpy_msg.x = roll
            rpy_msg.y = pitch
            rpy_msg.z = yaw
            self.rpy_pub.publish(rpy_msg)

    def log_sync_timing(self, rgb_msg, depth_msg):
        rgb_ts = rgb_msg.getTimestampDevice()
        depth_ts = depth_msg.getTimestampDevice()
        delta_ms = abs((rgb_ts - depth_ts).total_seconds()) * 1000.0

        last_rgb_ts = getattr(self, "last_rgb_device_ts", None)
        last_depth_ts = getattr(self, "last_depth_device_ts", None)
        last_host_ts = getattr(self, "last_sync_host_ts", None)
        host_ts = time.time()

        if last_rgb_ts is not None and last_depth_ts is not None and last_host_ts is not None:
            rgb_period_ms = (rgb_ts - last_rgb_ts).total_seconds() * 1000.0
            depth_period_ms = (depth_ts - last_depth_ts).total_seconds() * 1000.0
            host_period_ms = (host_ts - last_host_ts) * 1000.0

            self.get_logger().info(
                f"RGB period | device={rgb_period_ms:.1f} ms, host={host_period_ms:.1f} ms"
            )
            self.get_logger().warn(
                f"DEPTH period | device={depth_period_ms:.1f} ms, host={host_period_ms:.1f} ms"
            )

        self.get_logger().info(f"RGB-DEPTH delta = {delta_ms:.2f} ms")

        self.last_rgb_device_ts = rgb_ts
        self.last_depth_device_ts = depth_ts
        self.last_sync_host_ts = host_ts

    # =====================================================
    # DOWNWARD CAMERA WORKER
    # =====================================================
    def downward_worker(self):
        period = 1.0 / max(float(DOWNWARD_CAMERA_FPS), 1.0)

        while not self.shutdown_event.is_set() and rclpy.ok():
            start = time.time()

            try:
                ret, frame = self.usb_cap.read()

                if ret:
                    if DOWNWARD_ROTATE_180:
                        frame = cv2.rotate(frame, cv2.ROTATE_180)

                    msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "oak1_rgb_frame"
                    self.rgb1_pub.publish(msg)

                    with self.latest_lock:
                        self.rgb_oak1_latest = frame

                elif time.time() - self.last_downward_warn_time > 2.0:
                    self.get_logger().warn("USB downward camera: frame not received")
                    self.last_downward_warn_time = time.time()

            except Exception as exc:
                self.get_logger().error(f"Downward camera worker error: {exc}")
                time.sleep(0.05)

            elapsed = time.time() - start
            time.sleep(max(0.001, period - elapsed))

    # =====================================================
    # H264 WORKER
    # =====================================================
    def h264_worker(self):
        dev = self.oakd_dev

        while not self.shutdown_event.is_set() and rclpy.ok():
            try:
                if self.active_stream == "oakd":
                    h264_pkt = self.get_latest(dev["h264"])

                    if h264_pkt is not None:
                        data = h264_pkt.getData()

                        if data is not None and data.size > 0:
                            buf = Gst.Buffer.new_wrapped(data.tobytes())
                            self.appsrc.emit("push-buffer", buf)

                # Note: the original code only had H264 for oakd.
                # Switching to oak1 is preserved as state, but there is no oak1 encoder path here.
                time.sleep(0.001)

            except Exception as exc:
                self.get_logger().error(f"H264 worker error: {exc}")
                time.sleep(0.05)

    # =====================================================
    # VISUALIZATION TIMER
    # =====================================================
    def visualization_loop(self):
        if not START_DEPTH_COLOR:
            return

        with self.latest_lock:
            if self.pending_viz_rgb is None or self.pending_viz_depth is None:
                return

            frame_rgb = self.pending_viz_rgb.copy()
            frame_depth = self.pending_viz_depth.copy()

            # Mark consumed. If a newer frame arrives while processing, OAK thread will replace this.
            self.pending_viz_rgb = None
            self.pending_viz_depth = None

        try:
            depth_color = self.depth_to_colormap(frame_depth, max_depth_mm=10000)
            depth_color_msg = self.bridge.cv2_to_imgmsg(depth_color, "bgr8")
            depth_color_msg.header.stamp = self.get_clock().now().to_msg()
            depth_color_msg.header.frame_id = "oakd_depth_color_frame"
            self.depth_color_pub.publish(depth_color_msg)

            if START_OVERLAY:
                if depth_color.shape[:2] != frame_rgb.shape[:2]:
                    depth_color = cv2.resize(depth_color, (frame_rgb.shape[1], frame_rgb.shape[0]))

                overlay = cv2.addWeighted(frame_rgb, 0.6, depth_color, 0.4, 0)

                with self.latest_lock:
                    self.overlay_latest = overlay

                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
                overlay_msg.header.stamp = self.get_clock().now().to_msg()
                overlay_msg.header.frame_id = "oakd_rgb_depth_overlay_frame"
                self.overlay_pub.publish(overlay_msg)

        except Exception as exc:
            self.get_logger().warn(f"Visualization loop error: {exc}")

    # =====================================================
    # SAVE TIMER
    # =====================================================
    def save_loop(self):
        if not bool(self.save_images):
            return

        now = time.time()
        if now - self.last_save_time < SAVE_INTERVAL:
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")

        with self.latest_lock:
            rgb_oakd = None if self.rgb_oakd_latest is None else self.rgb_oakd_latest.copy()
            depth = None if self.depth_latest is None else self.depth_latest.copy()
            rgb_oak1 = None if self.rgb_oak1_latest is None else self.rgb_oak1_latest.copy()

        if rgb_oakd is not None and depth is not None:
            cv2.imwrite(os.path.join(RGB_OAKD_DIR, f"{timestamp}.jpg"), rgb_oakd)
            cv2.imwrite(os.path.join(DEPTH_DIR, f"{timestamp}.png"), depth)
            self.get_logger().info(f"Saved synchronized OAK-D frame set: {timestamp}")

        if rgb_oak1 is not None:
            cv2.imwrite(os.path.join(RGB_OAK1_DIR, f"{timestamp}_down.jpg"), rgb_oak1)
            self.get_logger().info(f"Saved downward frame: {timestamp}")

        self.last_save_time = now

    # =====================================================
    # CLEANUP
    # =====================================================
    def destroy_node(self):
        self.get_logger().info("Shutting down Dual OAK Node...")
        self.shutdown_event.set()

        for thread in self.worker_threads:
            if thread.is_alive():
                thread.join(timeout=1.0)

        try:
            self.gst_pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass

        if self.usb_cap is not None:
            self.usb_cap.release()

        try:
            self.stack.close()
        except Exception:
            pass

        super().destroy_node()

    def depth_to_colormap(self, depth_frame, max_depth_mm=100000):
        invalid_mask = depth_frame == 0

        try:
            valid_depth = depth_frame[depth_frame != 0]

            if valid_depth.size == 0:
                return np.zeros((depth_frame.shape[0], depth_frame.shape[1], 3), dtype=np.uint8)

            min_depth = np.percentile(valid_depth, 3)
            max_depth = np.percentile(valid_depth, 95)

            if min_depth <= 0 or max_depth <= 0 or min_depth >= max_depth:
                return np.zeros((depth_frame.shape[0], depth_frame.shape[1], 3), dtype=np.uint8)

            log_depth = np.zeros_like(depth_frame, dtype=np.float32)
            np.log(depth_frame, where=depth_frame != 0, out=log_depth)

            log_min_depth = np.log(min_depth)
            log_max_depth = np.log(max_depth)

            np.nan_to_num(
                log_depth,
                copy=False,
                nan=log_min_depth,
                posinf=log_max_depth,
                neginf=log_min_depth,
            )

            log_depth = np.clip(log_depth, log_min_depth, log_max_depth)
            depth_norm = np.interp(log_depth, (log_min_depth, log_max_depth), (0, 255))
            depth_norm = np.nan_to_num(depth_norm).astype(np.uint8)

            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            depth_color[invalid_mask] = [0, 0, 0]

            return depth_color

        except Exception:
            return np.zeros((depth_frame.shape[0], depth_frame.shape[1], 3), dtype=np.uint8)


# =========================================================
# MAIN
# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = DualOakNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
