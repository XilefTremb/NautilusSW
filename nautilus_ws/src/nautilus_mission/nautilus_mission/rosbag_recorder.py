import subprocess
import signal
import shutil
import time
import cv2

from pathlib import Path
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


# =====================================================
# CONFIG
# =====================================================
SAVE_TARGET = "computer"  # "usb" or "computer"

USB_DEVICE = "/dev/sda1"
USB_MOUNT_POINT = "/media/nautilus/95B9-46D3"
USB_OUTPUT_DIR = "/media/nautilus/95B9-46D3/rosbags"

COMPUTER_OUTPUT_DIR = "~/home/nautilus/rosbags"



class RosbagRecorder:
    def __init__(self, node=None):
        self.node = node
        self.process = None
        self.bag_path = None

        self.bridge = CvBridge()
        self.annotated_frames = deque(maxlen=100)

        if self.node is not None:
            self.annotated_sub = self.node.create_subscription(Image, "/yolo/image_annotated_fwd_cam", self.annotated_callback, 10)

        self.output_dir = Path(USB_OUTPUT_DIR) if SAVE_TARGET == "usb" else Path(COMPUTER_OUTPUT_DIR)

        self.topics = [
            "/yolo/detections_forward",
            "/yolo/detections_downward",
            "/control/vision_errors/yaw",
            "/control/vision_errors/forward",
            "/control/vision_errors/lateral",
            "/control/vision_errors/forward_ekf",
            "/control/cmd/yaw",
            "/control/cmd/forward",
            "/control/cmd/lateral",
            "/dvl/twist",
        ]

    def mount_usb(self):
        mount_point = Path(USB_MOUNT_POINT)
        mount_point.mkdir(parents=True, exist_ok=True)

        if mount_point.is_mount():
            print(f"[ROSBAG] USB already mounted at {mount_point}")
            return True

        try:
            subprocess.run(
                ["sudo", "mount", USB_DEVICE, str(mount_point)],
                check=True
            )
            print(f"[ROSBAG] USB mounted: {USB_DEVICE} -> {mount_point}")
            return True

        except Exception as e:
            print(f"[ROSBAG] Failed to mount USB: {e}")
            print("[ROSBAG] Recording disabled.")
            return False

    def start(self):
        if SAVE_TARGET == "usb":
            if not self.mount_usb():
                return

        parent_dir = self.output_dir.parent

        if not parent_dir.exists():
            print(
                f"[ROSBAG] Output path does not exist:\n"
                f"         {parent_dir}\n"
                f"         Recording disabled."
            )
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.bag_path = self.output_dir / f"mission_debug_{timestamp}"

        cmd = [
            "ros2", "bag", "record",
            "--compression-mode", "file",
            "--compression-format", "zstd",
            "-o", str(self.bag_path),
            *self.topics
        ]

        try:
            self.process = subprocess.Popen(cmd)
            print(f"[ROSBAG] Recording started: {self.bag_path}")

        except Exception as e:
            print(f"[ROSBAG] Failed to start rosbag: {e}")
            self.process = None

    def stop(self):
        if self.process is None:
            return

        print("[ROSBAG] Stopping recording...")
        self.process.send_signal(signal.SIGINT)

        try:
            self.process.wait(timeout=360)
        except subprocess.TimeoutExpired:
            print("[ROSBAG] Timeout while stopping. Killing rosbag...")
            self.process.kill()
            self.process.wait()

        self.process = None
        self.save_annotated_frames()
        print("[ROSBAG] Recording stopped.")
        
    def ask_keep_or_delete(self):
        if self.bag_path is None:
            return

        try:
            answer = input(f"\nDo you want to keep this rosbag? {self.bag_path} [y/N]: ").strip().lower()
        except EOFError:
            print("[ROSBAG] No stdin available. Keeping rosbag by default.")
            return

        if answer not in ["y", "yes", "o", "oui"]:
            print("[ROSBAG] Deleting rosbag...")
            shutil.rmtree(self.bag_path, ignore_errors=True)
            print("[ROSBAG] Deleted.")
        else:
            print(f"[ROSBAG] Saved: {self.bag_path}")

    def annotated_callback(self, msg):
        self.annotated_frames.append(msg)


    def save_annotated_frames(self):
        if self.bag_path is None:
            return

        output_dir = self.bag_path / "last_annotated_frames"
        output_dir.mkdir(parents=True, exist_ok=True)

        if not self.annotated_frames:
            print("[ROSBAG] No annotated frames to save.")
            return

        print(f"[ROSBAG] Saving {len(self.annotated_frames)} annotated frames...")

        for i, msg in enumerate(self.annotated_frames):
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                cv2.imwrite(str(output_dir / f"annotated_{i:03d}.jpg"), cv_img)
            except Exception as e:
                print(f"[ROSBAG] Failed to save annotated frame {i}: {e}")


