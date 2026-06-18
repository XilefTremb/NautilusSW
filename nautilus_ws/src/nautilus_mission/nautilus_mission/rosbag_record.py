import subprocess
import signal
import shutil
import time
from pathlib import Path


# =====================================================
# CONFIG
# =====================================================
SAVE_TARGET = "usb"  # "usb" or "computer"

USB_DEVICE = "/dev/sda1"
USB_MOUNT_POINT = "/media/usb"

COMPUTER_OUTPUT_DIR = "/home/rosbags"
USB_OUTPUT_DIR = "/media/usb/rosbags"


class RosbagRecorder:
    def __init__(self):
        self.process = None
        self.bag_path = None

        if SAVE_TARGET == "usb":
            self.output_dir = Path(USB_OUTPUT_DIR)
        else:
            self.output_dir = Path(COMPUTER_OUTPUT_DIR)

        self.topics = [
            "/oakd/camera/image_raw",
            "/oakd/camera/depth/image_raw",
            "/oak1/camera/image_raw",
            "/yolo/detections_forward",
            "/yolo/detections_downward",
            "/yolo/mean_depth_forward_cam",
            "/mission/state",
            "/control/vision_errors/yaw",
            "/control/vision_errors/forward",
            "/control/vision_errors/lateral",
            "/control/cmd/yaw",
            "/control/cmd/forward",
            "/control/cmd/lateral",
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

        try:
            print("[ROSBAG] Stopping recording...")
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=10)

        except Exception as e:
            print(f"[ROSBAG] Error stopping rosbag: {e}")

        self.process = None
        print("[ROSBAG] Recording stopped.")

    def ask_keep_or_delete(self):
        if self.bag_path is None:
            return

        answer = input(
            f"\nDo you want to keep this rosbag? {self.bag_path} [y/N]: "
        ).strip().lower()

        if answer not in ["y", "yes", "o", "oui"]:
            print("[ROSBAG] Deleting rosbag...")
            shutil.rmtree(self.bag_path, ignore_errors=True)
            print("[ROSBAG] Deleted.")
        else:
            print(f"[ROSBAG] Saved: {self.bag_path}")