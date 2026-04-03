import depthai as dai
import gi
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
	Gst.Caps.from_string(f"video/x-h264,stream-format=(string)byte-stream,alignment=(string)au,framerate={FPS}/1"),
)
gst_pipeline.set_state(Gst.State.PLAYING)

# ---------------- DepthAI 3.x pipeline ----------------
pipeline = dai.Pipeline()

# New API Camera node (replaces deprecated ColorCamera)
cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
cam_out = cam.requestOutput(
	size=(1280, 720),
	fps=FPS,
	type=dai.ImgFrame.Type.NV12
	# type here is RAW/NV12-ish internally; the encoder will accept it
)

enc = pipeline.create(dai.node.VideoEncoder)
enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H264_MAIN)
enc.setBitrate(7_000_000)
enc.setKeyframeFrequency(FPS * 2)  # ~2 seconds

# Link camera frames into encoder
cam_out.link(enc.input)

# Request the encoded bitstream as an output queue
h264_out = enc.bitstream.createOutputQueue(maxSize=16, blocking=False)

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

gst_pipeline.set_state(Gst.State.NULL)
