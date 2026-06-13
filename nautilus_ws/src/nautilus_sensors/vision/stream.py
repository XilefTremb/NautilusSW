#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
from vision.blue_filter import blue_filter
from datetime import timedelta
from geometry_msgs.msg import Vector3
from scipy.spatial.transform import Rotation as R



import cv2
import depthai as dai
import gi
import os
import time
import contextlib
import threading
import sys
import select
import numpy as np


gi.require_version("Gst", "1.0")
from gi.repository import Gst


# =========================================================
# CONFIG
# =========================================================
UDP_IP = "192.168.1.10"
UDP_PORT = 5600
FPS = 10
SAVE_INTERVAL = 3.0
START_BLUE_FILTER = False
START_DEPTH_COLOR = True
START_OVERLAY = True

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
        # self.save_images = False
        # Active stream (switchable)
        self.active_stream = "oakd"

        # ROS Bridge
        self.bridge = CvBridge()

        # ROS Publishers
        self.rgb_pub = self.create_publisher(Image, "/oakd/camera/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/oakd/camera/depth/image_raw", 10)
        self.overlay_pub = self.create_publisher(Image, "/oakd/camera/rgb_depth_overlay", 10)
        self.depth_color_pub = self.create_publisher(Image, "/oakd/camera/depth/color",10)
        self.imu_pub = self.create_publisher(Imu, "/oakd/imu/data_raw", 10)
        self.rpy_pub = self.create_publisher(Vector3, "/oakd/imu/rpy",10)

        self.rgb1_pub = self.create_publisher(Image, "/oak1/camera/image_raw", 10)

        # GStreamer init
        Gst.init(None)
        self.setup_gstreamer()

        # Latest frames
        self.rgb_oakd_latest = None
        self.rgb_oak1_latest = None
        self.depth_latest = None
        self.overlay_latest = None

        self.last_save_time = time.time()

        # Devices
        self.stack = contextlib.ExitStack()
        self.devices_data = []
        self.setup_devices()

        # Keyboard thread
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

        # ROS loop
        self.timer = self.create_timer(0.01, self.main_loop)

    # =====================================================
    # KEYBOARD CONTROL
    # =====================================================
    def keyboard_listener(self):
        self.get_logger().info("Press 's' to switch camera stream")

        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)

                if key == 's':
                    if self.active_stream == "oakd":
                        self.active_stream = "oak1"
                    else:
                        self.active_stream = "oakd"

                    self.get_logger().info(f"Switched stream to: {self.active_stream}")

    # =====================================================
    # GSTREAMER
    # =====================================================
    def setup_gstreamer(self):
        pipeline_str = (
            "appsrc name=src is-live=true do-timestamp=true format=time "
            "block=true max-buffers=4 ! "
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
    # QUEUE HELPER
    # =====================================================
    def get_latest(self, queue):
        latest = None

        while True:
            pkt = queue.tryGet()
            if pkt is None:
                break
            latest = pkt

        return latest

    # =====================================================
    # PIPELINES
    # =====================================================
    def create_oak1_pipeline(self, pipeline):
        camRgb = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )

        video = camRgb.requestOutput(
            size=(1280, 704),
            fps=FPS,
            type=dai.ImgFrame.Type.NV12
        )

        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(
            FPS,
            dai.VideoEncoderProperties.Profile.H264_BASELINE
        )
        enc.setBitrate(3_000_000)

        video.link(enc.input)

        h264_queue = enc.bitstream.createOutputQueue(maxSize=16, blocking=False)
        rgb_queue = video.createOutputQueue(maxSize=1, blocking=False)

        return rgb_queue, h264_queue

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

        # Same idea as Alignement.py
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

        # H264 UDP stream
        manip = pipeline.create(dai.node.ImageManip)
        manip.setMaxOutputFrameSize(2_000_000)
        manip.initialConfig.addRotateDeg(180)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)

        rgb_out.link(manip.inputImage)

        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(
            FPS,
            dai.VideoEncoderProperties.Profile.H264_MAIN,
        )
        enc.setBitrate(7_000_000)
        enc.setKeyframeFrequency(FPS)

        manip.out.link(enc.input)

        sync_queue = sync.out.createOutputQueue(maxSize=1, blocking=False)

        raw_depth_queue = stereo.depth.createOutputQueue(maxSize=1, blocking=False)

        h264_queue = enc.bitstream.createOutputQueue(maxSize=1, blocking=False)

        # IMU Data OAKD
        imu = pipeline.create(dai.node.IMU)

        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER, 200)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED, 200)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 200)

        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        imu_queue = imu.out.createOutputQueue(maxSize=10, blocking=False)

        return sync_queue, raw_depth_queue, h264_queue, imu_queue

    # =====================================================
    # DEVICE SETUP
    # =====================================================
    def setup_devices(self):
        deviceInfos = dai.Device.getAllAvailableDevices()
        self.get_logger().info(f"Found devices: {len(deviceInfos)}")

        for deviceInfo in deviceInfos:
            pipeline = self.stack.enter_context(dai.Pipeline())
            device = pipeline.getDefaultDevice()
            cameras = device.getConnectedCameras()

            if len(cameras) > 1:
                sync_q, depth_q, h264_q, imu_q = self.create_oakd_pipeline(pipeline)
                pipeline.start()

                self.devices_data.append({
                    "type": "oakd",
                    "sync": sync_q,
                    "raw_depth": depth_q,
                    "h264": h264_q,
                    "imu": imu_q,
                })
            else:
                rgb_q, h264_q = self.create_oak1_pipeline(pipeline)
                pipeline.start()

                self.devices_data.append({
                    "type": "oak1",
                    "rgb": rgb_q,
                    "h264": h264_q
                })


    # =====================================================
    # MAIN LOOP
    # =====================================================
    def main_loop(self):
        now = time.time()

        for dev in self.devices_data:

            # =================================================
            # OAK-D
            # =================================================
            if dev["type"] == "oakd":
                sync_pkt = self.get_latest(dev["sync"])
                imu_pkt = self.get_latest(dev["imu"])

                if sync_pkt is not None:
                    rgb_msg = sync_pkt["rgb"]
                    depth_msg = sync_pkt["depth_aligned"]

                    frame_rgb = rgb_msg.getCvFrame()
                    frame_depth = depth_msg.getFrame()

                    frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
                    frame_depth = cv2.rotate(frame_depth, cv2.ROTATE_180)

                    if START_BLUE_FILTER:
                        frame_rgb = blue_filter(frame_rgb)

                    self.rgb_oakd_latest = frame_rgb
                    self.depth_latest = frame_depth

                    # Publish RGB
                    self.rgb_pub.publish(
                        self.bridge.cv2_to_imgmsg(frame_rgb, "bgr8")
                    )

                    # Publish aligned depth
                    self.depth_pub.publish(
                        self.bridge.cv2_to_imgmsg(frame_depth, "16UC1")
                    )

                    # Publish color depth
                    if START_DEPTH_COLOR:
                        depth_color = self.depth_to_colormap(
                            frame_depth,
                            max_depth_mm=10000,
                        )

                        self.depth_color_pub.publish(
                            self.bridge.cv2_to_imgmsg(depth_color, "bgr8")
                        )

                        # Publish overlay RGB + depth
                        if START_OVERLAY:
                            if depth_color.shape[:2] != frame_rgb.shape[:2]:
                                depth_color = cv2.resize(
                                    depth_color,
                                    (frame_rgb.shape[1], frame_rgb.shape[0]),
                                )

                            overlay = cv2.addWeighted(
                                frame_rgb,
                                0.6,
                                depth_color,
                                0.4,
                                0,
                            )

                            self.overlay_latest = overlay

                            self.overlay_pub.publish(
                                self.bridge.cv2_to_imgmsg(overlay, "bgr8")
                            )
                    

                if imu_pkt is not None:
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
                        
                        self.imu_pub.publish(imu_msg)

                        rot = packet.rotationVector

                        imu_msg.orientation.x = rot.i
                        imu_msg.orientation.y = rot.j
                        imu_msg.orientation.z = rot.k
                        imu_msg.orientation.w = rot.real

                        r = R.from_quat([
                            rot.i,
                            rot.j,
                            rot.k,
                            rot.real
                        ])

                        roll, pitch, yaw = r.as_euler('xyz', degrees=True)

                        rpy_msg = Vector3()
                        rpy_msg.x = roll
                        rpy_msg.y = pitch
                        rpy_msg.z = yaw

                        self.rpy_pub.publish(rpy_msg)


            # =================================================
            # OAK-1
            # =================================================
            elif dev["type"] == "oak1":
                rgb_pkt = self.get_latest(dev["rgb"])

                if rgb_pkt is not None:
                    frame = rgb_pkt.getCvFrame()
                    frame = cv2.rotate(frame, cv2.ROTATE_180)

                    self.rgb_oak1_latest = frame

                    self.rgb1_pub.publish(
                        self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    )

            # =================================================
            # H264 UDP stream switchable
            # =================================================
            if "h264" in dev and dev["type"] == self.active_stream:
                h264_pkt = self.get_latest(dev["h264"])

                if h264_pkt is not None:
                    data = h264_pkt.getData()

                    if data is not None and data.size > 0:
                        buf = Gst.Buffer.new_wrapped(data.tobytes())
                        self.appsrc.emit("push-buffer", buf)

        # =====================================================
        # SAVE
        # =====================================================
        if bool(self.save_images) and (now - self.last_save_time >= SAVE_INTERVAL):
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            
            if self.rgb_oakd_latest is not None and self.depth_latest is not None:

                cv2.imwrite(os.path.join(RGB_OAKD_DIR, f"{timestamp}.jpg"), self.rgb_oakd_latest)
                cv2.imwrite(os.path.join(DEPTH_DIR, f"{timestamp}.png"), self.depth_latest)

                self.get_logger().info(f"Saved synchronized frame set: {timestamp}")

            if self.rgb_oak1_latest is not None and self.depth_latest is not None:

                cv2.imwrite(os.path.join(RGB_OAK1_DIR, f"{timestamp}.jpg"), self.rgb_oak1_latest)

                self.get_logger().info(f"Saved synchronized frame set: {timestamp}")

            self.last_save_time = now

    # =====================================================
    # CLEANUP
    # =====================================================
    def destroy_node(self):
        self.get_logger().info("Shutting down Dual OAK Node...")
        self.gst_pipeline.set_state(Gst.State.NULL)
        self.stack.close()
        super().destroy_node()

    def depth_to_colormap(self, depth_frame, max_depth_mm=10000):
        invalid_mask = depth_frame == 0

        try:
            valid_depth = depth_frame[depth_frame != 0]

            if valid_depth.size == 0:
                return np.zeros(
                    (depth_frame.shape[0], depth_frame.shape[1], 3),
                    dtype=np.uint8
                )

            min_depth = np.percentile(valid_depth, 3)
            max_depth = np.percentile(valid_depth, 95)

            # Évite log(0) ou log de valeurs invalides
            if min_depth <= 0 or max_depth <= 0 or min_depth >= max_depth:
                return np.zeros(
                    (depth_frame.shape[0], depth_frame.shape[1], 3),
                    dtype=np.uint8
                )

            log_depth = np.zeros_like(depth_frame, dtype=np.float32)

            np.log(
                depth_frame,
                where=depth_frame != 0,
                out=log_depth
            )

            log_min_depth = np.log(min_depth)
            log_max_depth = np.log(max_depth)

            np.nan_to_num(
                log_depth,
                copy=False,
                nan=log_min_depth,
                posinf=log_max_depth,
                neginf=log_min_depth
            )

            log_depth = np.clip(
                log_depth,
                log_min_depth,
                log_max_depth
            )

            depth_norm = np.interp(
                log_depth,
                (log_min_depth, log_max_depth),
                (0, 255)
            )

            depth_norm = np.nan_to_num(depth_norm)
            depth_norm = depth_norm.astype(np.uint8)

            depth_color = cv2.applyColorMap(
                depth_norm,
                cv2.COLORMAP_JET
            )
            depth_color[invalid_mask] = [0, 0, 0]

            return depth_color

        except Exception:
            return np.zeros(
                (depth_frame.shape[0], depth_frame.shape[1], 3),
                dtype=np.uint8
            )

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
