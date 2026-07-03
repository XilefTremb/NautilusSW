#!/usr/bin/env python3

import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
import json
from pathlib import Path


repo_root = Path.home() / "NautilusSW"

CONFIG_FILE = (
    repo_root
    / "nautilus_ws"
    / "src"
    / "nautilus_sensors"
    / "vision"
    / "edge_params.json"
)

CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
print(CONFIG_FILE)

def load_params_edge_detector_json():
    default_params = {
        "dark_threshold": 150,
        "light_min_brightness": 200,
        "light_bright_percentile": 85,
        "light_min_brightness_torpedo": 200,
        "light_bright_percentile_torpedo": 85,
        "min_pixel_count": 30,
    }

    if not CONFIG_FILE.exists():
        save_params(default_params)
        return default_params

    try:
        with open(CONFIG_FILE, "r") as f:
            saved = json.load(f)

        default_params.update(saved)
        return default_params

    except Exception:
        save_params(default_params)
        return default_params

def save_params(params):
    with open(CONFIG_FILE, "w") as f:
        json.dump(params, f, indent=4)


class EdgeSliderNode(Node):
    def __init__(self):
        super().__init__("edge_slider_gui")
        self.pub = self.create_publisher(Int32MultiArray, "/yolo/edge_params", 10)

    def publish_params(self, values):
        msg = Int32MultiArray()
        msg.data = values
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = EdgeSliderNode()

    root = tk.Tk()
    root.title("YOLO Edge Detector Thresholds")

    sliders = {}

    saved_params = load_params_edge_detector_json()

    params = {
        "dark_threshold": (0, 255, saved_params["dark_threshold"]),
        "light_min_brightness": (0, 255, saved_params["light_min_brightness"]),
        "light_bright_percentile": (0, 100, saved_params["light_bright_percentile"]),
        "light_min_brightness_torpedo": (0, 255, saved_params["light_min_brightness_torpedo"]),
        "light_bright_percentile_torpedo": (0, 100, saved_params["light_bright_percentile_torpedo"]),
        "min_pixel_count": (0, 500, saved_params["min_pixel_count"]),
    }

    def update(_=None):
        current_params = {
            "dark_threshold": sliders["dark_threshold"].get(),
            "light_min_brightness": sliders["light_min_brightness"].get(),
            "light_bright_percentile": sliders["light_bright_percentile"].get(),
            "light_min_brightness_torpedo": sliders["light_min_brightness_torpedo"].get(),
            "light_bright_percentile_torpedo": sliders["light_bright_percentile_torpedo"].get(),
            "min_pixel_count": sliders["min_pixel_count"].get(),
        }

        save_params(current_params)

        values = [
            current_params["dark_threshold"],
            current_params["light_min_brightness"],
            current_params["light_bright_percentile"],
            current_params["light_min_brightness_torpedo"],
            current_params["light_bright_percentile_torpedo"],
            current_params["min_pixel_count"],
        ]

        node.publish_params(values)

    for name, (min_v, max_v, default) in params.items():
        label = tk.Label(root, text=name)
        label.pack()

        slider = tk.Scale(
            root,
            from_=min_v,
            to=max_v,
            orient=tk.HORIZONTAL,
            length=400,
            command=update
        )
        slider.set(default)
        slider.pack()

        sliders[name] = slider

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()