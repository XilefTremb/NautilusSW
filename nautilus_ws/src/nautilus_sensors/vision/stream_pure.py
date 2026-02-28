import cv2
import depthai as dai
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

UDP_IP = "192.168.1.10"
UDP_PORT = 5600
FPS = 15

pipeline_str = (
    "appsrc name=src is-live=true do-timestamp=true format=time block=true max-buffers=2 ! "
    "queue leaky=downstream max-size-buffers=1 ! "
    "videoconvert ! "
    "x264enc bitrate=7000 speed-preset=ultrafast tune=zerolatency key-int-max=30 ! "
    "h264parse ! rtph264pay config-interval=1 pt=96 ! "
    f"udpsink host={UDP_IP} port={UDP_PORT} sync=false async=false"
)

gst_pipeline = Gst.parse_launch(pipeline_str)
appsrc = gst_pipeline.get_by_name("src")
appsrc.set_property("caps", Gst.Caps.from_string(
    f"video/x-raw,format=BGR,width=1280,height=720,framerate={FPS}/1"
))
gst_pipeline.set_state(Gst.State.PLAYING)

pipeline = dai.Pipeline()
rgbCam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
rgbOut = rgbCam.requestOutput(size=(1280, 720), type=dai.ImgFrame.Type.BGR888i)
rgbQueue = rgbOut.createOutputQueue(maxSize=4)

SHOW = False  # <-- IMPORTANT: keep False when using ssh -X

with pipeline:
    pipeline.start()

    while pipeline.isRunning():
        frame = rgbQueue.get().getCvFrame()

        # Wrap bytes and push (bounded appsrc + leaky queue prevents runaway RAM)
        buf = Gst.Buffer.new_wrapped(frame.tobytes())
        ret = appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            continue

        if SHOW:
            cv2.imshow("RGB", frame)
            if cv2.waitKey(1) == 27:
                break

gst_pipeline.set_state(Gst.State.NULL)
cv2.destroyAllWindows()