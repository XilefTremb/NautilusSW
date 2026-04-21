import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import depthai as dai
import gi
import os
import time
import contextlib

gi.require_version("Gst", "1.0")
from gi.repository import Gst


# =========================================================
# CONFIG
# =========================================================
UDP_IP = "192.168.1.10"
UDP_PORT = 5600
FPS = 15
SAVE_INTERVAL = 1000.0  # seconds

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
        
        self.declare_parameter("save_images", True)
        self.save_images = self.get_parameter("save_images").value

        # ROS Bridge
        self.bridge = CvBridge()

        # ROS Publishers
        self.rgb_pub = self.create_publisher(
            Image,
            "/oakd/camera/image_raw",
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            "/oakd/camera/depth/image_raw",
            10
        )

        # GStreamer init
        Gst.init(None)
        self.setup_gstreamer()

        # Latest synchronized frames
        self.rgb_oakd_latest = None
        self.rgb_oak1_latest = None
        self.depth_latest = None

        self.last_save_time = time.time()

        # Start DepthAI devices
        self.stack = contextlib.ExitStack()
        self.devices_data = []
        self.setup_devices()

        # ROS timer loop
        self.timer = self.create_timer(0.01, self.main_loop)

    # =====================================================
    # GSTREAMER
    # =====================================================
    def setup_gstreamer(self):
        pipeline_str = (
            "appsrc name=src is-live=true do-timestamp=true format=time "
            "block=true max-buffers=8 ! "
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
    # PIPELINES
    # =====================================================
    def create_oak1_pipeline(self, pipeline):
        camRgb = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )

        rgb_out = camRgb.requestOutput(
            size=(1280, 704),
            fps=FPS,
            type=dai.ImgFrame.Type.BGR888p
        )

        rgb_queue = rgb_out.createOutputQueue(
            maxSize=4,
            blocking=False
        )

        return rgb_queue

    def create_oakd_pipeline(self, pipeline):
        # RGB camera
        camRgb = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A
        )

        manip = pipeline.create(dai.node.ImageManip)
        manip.setMaxOutputFrameSize(1500000)
        manip.initialConfig.addRotateDeg(180)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)

        cam_rgb_out = camRgb.requestOutput(
            size=(1280, 704),
            fps=FPS,
            type=dai.ImgFrame.Type.NV12
        )

        cam_rgb_out.link(manip.inputImage)

        # Encoder
        enc = pipeline.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(
            FPS,
            dai.VideoEncoderProperties.Profile.H264_MAIN
        )
        enc.setBitrate(7_000_000)
        enc.setKeyframeFrequency(FPS * 2)

        manip.out.link(enc.input)

        h264_queue = enc.bitstream.createOutputQueue(
            maxSize=16,
            blocking=False
        )

        # Stereo depth
        monoLeft = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B
        )
        monoRight = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C
        )

        stereo = pipeline.create(dai.node.StereoDepth)

        monoLeftOut = monoLeft.requestOutput(size=(1280, 720))
        monoRightOut = monoRight.requestOutput(size=(1280, 720))

        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)

        stereo.setRectification(True)
        stereo.setExtendedDisparity(True)
        stereo.setLeftRightCheck(True)

        depth_queue = stereo.depth.createOutputQueue(
            maxSize=4,
            blocking=False
        )
        
        rgb_queue = cam_rgb_out.createOutputQueue(
            maxSize=4,
            blocking=False
        )

        return rgb_queue, depth_queue, h264_queue

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

            self.get_logger().info(
                f"Connected: {deviceInfo.getDeviceId()} cams: {len(cameras)}"
            )

            if len(cameras) > 1:
                # OAK-D S2
                rgb_q, depth_q, h264_q = self.create_oakd_pipeline(pipeline)
                pipeline.start()

                self.devices_data.append({
                    "type": "oakd",
                    "rgb": rgb_q,
                    "depth": depth_q,
                    "h264": h264_q
                })

            else:
                # OAK-1
                rgb_q = self.create_oak1_pipeline(pipeline)
                pipeline.start()

                self.devices_data.append({
                    "type": "oak1",
                    "rgb": rgb_q
                })

    # =====================================================
    # MAIN LOOP
    # =====================================================
    def main_loop(self):
        now = time.time()

        for dev in self.devices_data:

            # ---------------- RGB ----------------
            rgb_pkt = dev["rgb"].tryGet()
            if rgb_pkt is not None:
                frame = rgb_pkt.getCvFrame()
                frame = cv2.rotate(frame, cv2.ROTATE_180)

                if dev["type"] == "oakd":
                    self.rgb_oakd_latest = frame

                    # Publish RGB ROS topic
                    rgb_msg = self.bridge.cv2_to_imgmsg(
                        frame,
                        encoding="bgr8"
                    )
                    self.rgb_pub.publish(rgb_msg)

                else:
                    self.rgb_oak1_latest = frame

            # ---------------- OAK-D ONLY ----------------
            if dev["type"] == "oakd":

                # H264 UDP Streaming
                h264_pkt = dev["h264"].tryGet()
                if h264_pkt is not None:
                    data = h264_pkt.getData()
                    if data is not None and data.size > 0:
                        buf = Gst.Buffer.new_wrapped(data.tobytes())
                        ret = self.appsrc.emit("push-buffer", buf)
                        if ret != Gst.FlowReturn.OK:
                            continue

                # Depth
                depth_pkt = dev["depth"].tryGet()
                if depth_pkt is not None:
                    self.depth_latest = depth_pkt.getFrame()
                    self.depth_latest = cv2.rotate(self.depth_latest, cv2.ROTATE_180)

                    # Publish Depth ROS topic
                    depth_msg = self.bridge.cv2_to_imgmsg(
                        self.depth_latest,
                        encoding="16UC1"
                    )
                    self.depth_pub.publish(depth_msg)

        # =================================================
        # SYNCHRONIZED SAVE
        # =================================================
        if self.save_images and (now - self.last_save_time >= SAVE_INTERVAL):
            if (
                self.rgb_oakd_latest is not None and
                self.rgb_oak1_latest is not None and
                self.depth_latest is not None
            ):
                timestamp = time.strftime("%Y%m%d_%H%M%S")

                oakd_rgb_path = os.path.join(
                    RGB_OAKD_DIR,
                    f"{timestamp}.jpg"
                )
                oak1_rgb_path = os.path.join(
                    RGB_OAK1_DIR,
                    f"{timestamp}.jpg"
                )
                depth_path = os.path.join(
                    DEPTH_DIR,
                    f"{timestamp}.png"
                )

                cv2.imwrite(
                    oakd_rgb_path,
                    self.rgb_oakd_latest,
                    [cv2.IMWRITE_JPEG_QUALITY, 90]
                )

                cv2.imwrite(
                    oak1_rgb_path,
                    self.rgb_oak1_latest,
                    [cv2.IMWRITE_JPEG_QUALITY, 90]
                )

                cv2.imwrite(depth_path, self.depth_latest)

                self.get_logger().info(
                    f"Saved synchronized frame set: {timestamp}"
                )

            self.last_save_time = now

    # =====================================================
    # CLEANUP
    # =====================================================
    def destroy_node(self):
        self.get_logger().info("Shutting down Dual OAK Node...")

        self.gst_pipeline.set_state(Gst.State.NULL)
        self.stack.close()

        super().destroy_node()


# =========================================================
# MAIN ENTRY
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
