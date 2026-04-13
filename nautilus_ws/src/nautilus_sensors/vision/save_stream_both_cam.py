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
SAVE_INTERVAL = 2.0  # seconds

SAVE_DIR = os.path.expanduser("~/Documents/dataset")
RGB_OAKD_DIR = os.path.join(SAVE_DIR, "rgb_oakd")
RGB_OAK1_DIR = os.path.join(SAVE_DIR, "rgb_oak1")
DEPTH_DIR = os.path.join(SAVE_DIR, "depth")

os.makedirs(RGB_OAKD_DIR, exist_ok=True)
os.makedirs(RGB_OAK1_DIR, exist_ok=True)
os.makedirs(DEPTH_DIR, exist_ok=True)

# =========================================================
# GSTREAMER INIT
# =========================================================
Gst.init(None)

pipeline_str = (
    "appsrc name=src is-live=true do-timestamp=true format=time block=true max-buffers=8 ! "
    "queue leaky=downstream max-size-buffers=4 ! "
    "h264parse config-interval=1 ! "
    "rtph264pay config-interval=1 pt=96 ! "
    f"udpsink host={UDP_IP} port={UDP_PORT} sync=false async=false"
)

gst_pipeline = Gst.parse_launch(pipeline_str)
appsrc = gst_pipeline.get_by_name("src")
appsrc.set_property(
    "caps",
    Gst.Caps.from_string(
        f"video/x-h264,stream-format=(string)byte-stream,"
        f"alignment=(string)au,framerate={FPS}/1"
    ),
)
gst_pipeline.set_state(Gst.State.PLAYING)

# =========================================================
# PIPELINE BUILDERS
# =========================================================
def create_oak1_pipeline(pipeline):
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    rgb_out = camRgb.requestOutput(
        size=(1280, 704),
        fps=FPS,
        type=dai.ImgFrame.Type.BGR888p
    )

    rgb_queue = rgb_out.createOutputQueue(maxSize=4, blocking=False)
    return rgb_queue


def create_oakd_pipeline(pipeline):
    # RGB camera
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

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

    rgb_queue = cam_rgb_out.createOutputQueue(maxSize=4, blocking=False)

    # Encoder
    enc = pipeline.create(dai.node.VideoEncoder)
    enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
    enc.setBitrate(7_000_000)
    enc.setKeyframeFrequency(FPS * 2)

    manip.out.link(enc.input)
    h264_queue = enc.bitstream.createOutputQueue(maxSize=16, blocking=False)

    # Stereo depth
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)

    monoLeftOut = monoLeft.requestOutput(size=(1280, 720))
    monoRightOut = monoRight.requestOutput(size=(1280, 720))

    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)

    depth_queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

    return rgb_queue, depth_queue, h264_queue


# =========================================================
# MAIN
# =========================================================
with contextlib.ExitStack() as stack:
    deviceInfos = dai.Device.getAllAvailableDevices()
    print("Found devices:", len(deviceInfos))

    devices_data = []

    for deviceInfo in deviceInfos:
        pipeline = stack.enter_context(dai.Pipeline())
        device = pipeline.getDefaultDevice()

        cameras = device.getConnectedCameras()
        print("Connected:", deviceInfo.getDeviceId(), "cams:", len(cameras))

        if len(cameras) > 1:
            # OAK-D S2
            rgb_q, depth_q, h264_q = create_oakd_pipeline(pipeline)
            pipeline.start()

            devices_data.append({
                "type": "oakd",
                "rgb": rgb_q,
                "depth": depth_q,
                "h264": h264_q
            })

        else:
            # OAK-1
            rgb_q = create_oak1_pipeline(pipeline)
            pipeline.start()

            devices_data.append({
                "type": "oak1",
                "rgb": rgb_q
            })

    # =====================================================
    # FRAME STORAGE FOR SYNCHRONIZATION
    # =====================================================
    rgb_oakd_latest = None
    depth_latest = None
    rgb_oak1_latest = None

    last_save_time = time.time()

    # =====================================================
    # LOOP
    # =====================================================
    while True:
        now = time.time()

        for idx, dev in enumerate(devices_data):

            # ---------------- RGB ----------------
            rgb_pkt = dev["rgb"].tryGet()
            if rgb_pkt is not None:
                frame = rgb_pkt.getCvFrame()

                if dev["type"] == "oakd":
                    rgb_oakd_latest = frame
                    #cv2.imshow("OAK-D RGB", frame)
                else:
                    rgb_oak1_latest = frame
                    #cv2.imshow("OAK-1 RGB", frame)

            # ---------------- OAK-D ONLY ----------------
            if dev["type"] == "oakd":

                # H264 UDP streaming
                h264_pkt = dev["h264"].tryGet()
                if h264_pkt is not None:
                    data = h264_pkt.getData()
                    if data is not None and data.size > 0:
                        buf = Gst.Buffer.new_wrapped(data.tobytes())
                        appsrc.emit("push-buffer", buf)

                # Depth
                depth_pkt = dev["depth"].tryGet()
                if depth_pkt is not None:
                    depth_latest = depth_pkt.getFrame()

                    depth_vis = cv2.normalize(
                        depth_latest, None, 0, 255, cv2.NORM_MINMAX
                    ).astype("uint8")

                    #cv2.imshow("OAK-D Depth", depth_vis)

        # =================================================
        # SYNCHRONIZED SAVE
        # Save only when all 3 latest frames exist
        # =================================================
        if now - last_save_time >= SAVE_INTERVAL:
            if (
                rgb_oakd_latest is not None and
                rgb_oak1_latest is not None and
                depth_latest is not None
            ):
                timestamp = time.strftime("%Y%m%d_%H%M%S")

                oakd_rgb_path = os.path.join(
                    RGB_OAKD_DIR, f"rgb_oakd_{timestamp}.jpg"
                )
                oak1_rgb_path = os.path.join(
                    RGB_OAK1_DIR, f"rgb_oak1_{timestamp}.jpg"
                )
                depth_path = os.path.join(
                    DEPTH_DIR, f"depth_{timestamp}.png"
                )

                cv2.imwrite(
                    oakd_rgb_path,
                    rgb_oakd_latest,
                    [cv2.IMWRITE_JPEG_QUALITY, 90]
                )
                cv2.imwrite(
                    oak1_rgb_path,
                    rgb_oak1_latest,
                    [cv2.IMWRITE_JPEG_QUALITY, 90]
                )
                cv2.imwrite(depth_path, depth_latest)

                print("Saved synchronized frame set:", timestamp)

            last_save_time = now

        # Exit
        if cv2.waitKey(1) == ord('q'):
            break

# =========================================================
# CLEANUP
# =========================================================
gst_pipeline.set_state(Gst.State.NULL)
cv2.destroyAllWindows()
