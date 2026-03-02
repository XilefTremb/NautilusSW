import cv2
import depthai as dai
import gi
import os
import time

gi.require_version("Gst", "1.0")
from gi.repository import Gst

# ---------------- GStreamer ----------------
Gst.init(None)

UDP_IP = "192.168.1.10"
UDP_PORT = 5600
FPS = 15

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
        f"video/x-h264,stream-format=(string)byte-stream,alignment=(string)au,framerate={FPS}/1"
    ),
)
gst_pipeline.set_state(Gst.State.PLAYING)

# ---------------- DepthAI pipeline ----------------
pipeline = dai.Pipeline()

# Caméra RGB
cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
cam_rgb_out = cam_rgb.requestOutput(
    size=(1280, 720),
    fps=FPS,
    type=dai.ImgFrame.Type.NV12,
    # type here is RAW/NV12-ish internally; the encoder will accept it
)

enc = pipeline.create(dai.node.VideoEncoder)
enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
enc.setBitrate(7_000_000)
enc.setKeyframeFrequency(FPS * 2)  # ~2 seconds

# Link camera frames into encoder
cam_rgb_out.link(enc.input)
rgbQueue = cam_rgb_out.createOutputQueue(maxSize=4)
h264_out = enc.bitstream.createOutputQueue(maxSize=16, blocking=False)

monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

monoLeftOut = monoLeft.requestOutput(size=(1280, 720))
monoLeftOut.link(stereo.left)

monoRightOut = monoRight.requestOutput(size=(1280, 720))
monoRightOut.link(stereo.right)

stereo.setRectification(True)
stereo.setExtendedDisparity(True)
stereo.setLeftRightCheck(True)

disparityQueue = stereo.disparity.createOutputQueue()

SAVE_DIR = os.path.expanduser("~/Documents/dataset")
RGB_DIR = os.path.join(SAVE_DIR, "rgb")
DEPTH_DIR = os.path.join(SAVE_DIR, "depth")

os.makedirs(RGB_DIR, exist_ok=True)
os.makedirs(DEPTH_DIR, exist_ok=True)

save_interval = 2.0  # seconds
last_save_time = time.time()

# ---------------- Run ----------------
with pipeline:
    pipeline.start()

    while pipeline.isRunning():
        pkt = h264_out.get()
        if pkt is None:
            continue

        data = pkt.getData()
        if data is None or data.size == 0:
            continue

        buf = Gst.Buffer.new_wrapped(data.tobytes())
        ret = appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            # downstream not accepting, drop
            continue

        # frame BGR
        frameRGB = rgbQueue.get().getCvFrame()
        frameDepth = disparityQueue.get().getFrame()

        now = time.time()
        if now - last_save_time >= save_interval:
            timestamp = time.strftime("%Y%m%d_%H%M%S")

            rgb_path = os.path.join(RGB_DIR, f"rgb_{timestamp}.jpg")
            depth_path = os.path.join(DEPTH_DIR, f"depth_{timestamp}.png")

            cv2.imwrite(rgb_path, frameRGB, [cv2.IMWRITE_JPEG_QUALITY, 90])
            cv2.imwrite(depth_path, frameDepth)
            print(f"Saved {timestamp}")
            last_save_time = now

gst_pipeline.set_state(Gst.State.NULL)