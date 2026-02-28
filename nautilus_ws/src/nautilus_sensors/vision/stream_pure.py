import cv2
import depthai as dai
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# ---------------- GStreamer ----------------
Gst.init(None)
UDP_IP = "192.168.1.10"  # L'adresse de ton PC QGC
UDP_PORT = 5600
FPS = 15

pipeline_str = (
    f"appsrc name=src ! videoconvert ! "
    f"x264enc bitrate=7000 speed-preset=ultrafast tune=zerolatency ! "
    f"h264parse ! rtph264pay config-interval=1 pt=96 ! "
    f"udpsink host={UDP_IP} port={UDP_PORT} sync=false"
)

gst_pipeline = Gst.parse_launch(pipeline_str)
appsrc = gst_pipeline.get_by_name("src")
appsrc.set_property("caps", Gst.Caps.from_string(
    f"video/x-raw,format=BGR,width=1280,height=720,framerate={FPS}/1"
))
gst_pipeline.set_state(Gst.State.PLAYING)

# ---------------- DepthAI pipeline ----------------
pipeline = dai.Pipeline()

# Caméra RGB (fonctionnelle avec Orin Nano et DepthAI 3.x)
rgbCam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
rgbOut = rgbCam.requestOutput(
    size=(1280, 720),  # <-- force la sortie à 720p
    type=dai.ImgFrame.Type.BGR888i
)
rgbQueue = rgbOut.createOutputQueue(maxSize=4)
# ---------------- Run ----------------
with pipeline:
    pipeline.start()
    frame = rgbQueue.get().getCvFrame()
    
    while pipeline.isRunning():
        # frame BGR
        frame = rgbQueue.get().getCvFrame()
        # push vers GStreamer / QGC
        buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
        buf.fill(0, frame.tobytes())
        appsrc.emit("push-buffer", buf)

        # optionnel : aperçu local pour debug
        cv2.imshow("RGB",frame)
        if cv2.waitKey(1) == 27:  # ESC pour quitter
            break

gst_pipeline.set_state(Gst.State.NULL)
cv2.destroyAllWindows()

