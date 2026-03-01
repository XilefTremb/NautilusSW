import depthai as dai
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

# ---------------- GStreamer ----------------
Gst.init(None)

UDP_IP = "192.168.1.10"
UDP_PORT = 5600
FPS = 15

# appsrc will receive already-encoded H264 access units (AU)
pipeline_str = (
    "appsrc name=src is-live=true do-timestamp=true format=time block=true max-buffers=4 ! "
    "queue leaky=downstream max-size-buffers=2 ! "
    "h264parse config-interval=1 ! "
    "rtph264pay config-interval=1 pt=96 ! "
    f"udpsink host={UDP_IP} port={UDP_PORT} sync=false async=false"
)

gst_pipeline = Gst.parse_launch(pipeline_str)
appsrc = gst_pipeline.get_by_name("src")

# Caps for H264 bytestream; alignment=au is important for RTP payloader
appsrc.set_property(
    "caps",
    Gst.Caps.from_string(
        f"video/x-h264,stream-format=(string)byte-stream,alignment=(string)au,framerate={FPS}/1"
    ),
)

gst_pipeline.set_state(Gst.State.PLAYING)

# ---------------- DepthAI pipeline (H264 on camera) ----------------
pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.ColorCamera)
cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setFps(FPS)

# Encoder input is NV12 by default from video output (efficient)
enc = pipeline.create(dai.node.VideoEncoder)
enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
# Optional: tune bitrate (bits per second)
enc.setBitrate(7_000_000)
# Force periodic keyframes (helps join stream / recovery)
enc.setKeyframeFrequency(FPS * 2)  # every ~2 seconds

cam.video.link(enc.input)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("h264")
enc.bitstream.link(xout.input)

# Small queue, non-blocking to avoid piling up if downstream stalls
h264Q = None

with dai.Device(pipeline) as device:
    h264Q = device.getOutputQueue(name="h264", maxSize=8, blocking=False)

    while True:
        pkt = h264Q.get()  # could be None if non-blocking; but .get() blocks in python wrapper
        if pkt is None:
            continue

        data = pkt.getData()  # bytes-like (encoded H264)
        if not data:
            continue

        # Wrap and push into GStreamer
        buf = Gst.Buffer.new_wrapped(bytes(data))
        ret = appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            # downstream not accepting; drop
            continue

gst_pipeline.set_state(Gst.State.NULL)