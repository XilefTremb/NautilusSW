#!/usr/bin/env python3

from enum import IntEnum
import curses
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from enums import ObjectID


class YoloDetectionViewer(Node):
    def __init__(self):
        super().__init__("yolo_detection_viewer")

        self.declare_parameter("detection_topic","/yolo/detections_forward")

        topic = self.get_parameter("detection_topic").get_parameter_value().string_value

        self.sub = self.create_subscription(Float32MultiArray,topic,self.detection_callback,10)

        self.topic_name = topic

        self.latest = {}
        self.last_msg_time = None


    def detection_callback(self, msg: Float32MultiArray):
        detections = {}

        if len(msg.data) % 5 != 0:
            self.get_logger().warn(
                f"Malformed detection array: len={len(msg.data)}, expected multiple of 5"
            )
            return

        for i in range(0, len(msg.data), 5):
            object_id = int(msg.data[i])
            detections[object_id] = {
                "x": msg.data[i + 1],
                "depth": msg.data[i + 2],
                "angle": msg.data[i + 3],
                "y": msg.data[i + 4],
            }

        self.latest = detections
        self.last_msg_time = time.time()


def draw(stdscr, node: YoloDetectionViewer):
    curses.curs_set(0)
    stdscr.nodelay(True)

    curses.start_color()
    curses.use_default_colors()

    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_BLACK, -1)
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    curses.init_pair(4, curses.COLOR_RED, -1)
    curses.init_pair(5, curses.COLOR_WHITE, -1)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

        stdscr.erase()

        height, width = stdscr.getmaxyx()

        title = f"Viewer: {node.topic_name}"
        stdscr.addstr(0, 0, title[:width], curses.A_BOLD)

        if node.last_msg_time is None:
            age_text = "No message received yet"
            age_color = curses.color_pair(4)
        else:
            age = time.time() - node.last_msg_time
            age_text = f"Last msg: {age:.2f}s ago | detected: {len(node.latest)}"
            age_color = curses.color_pair(3) if age > 1.0 else curses.color_pair(1)

        stdscr.addstr(1, 0, age_text[:width], age_color)

        header = f"{'ID':>3}  {'NAME':<22} {'SEEN':<5} {'X(px)':>9} {'Y(px)':>9} {'DEPTH(mm)':>11} {'ANGLE':>8}"
        stdscr.addstr(3, 0, header[:width], curses.A_BOLD)
        stdscr.addstr(4, 0, "-" * min(width, len(header)))

        row = 5

        for obj in ObjectID:
            if row >= height - 1:
                break

            detected = int(obj) in node.latest

            if detected:
                d = node.latest[int(obj)]
                line = (
                    f"{int(obj):>3}  {obj.name:<22} "
                    f"{'YES':<5} "
                    f"{d['x']:>9.1f} "
                    f"{d['y']:>9.1f} "
                    f"{d['depth']:>11.1f} "
                    f"{d['angle']:>8.1f}"
                )
                color = curses.color_pair(1)
            else:
                line = (
                    f"{int(obj):>3}  {obj.name:<22} "
                    f"{'no':<5} "
                    f"{'-':>9} "
                    f"{'-':>9} "
                    f"{'-':>11} "
                    f"{'-':>8}"
                )
                color = curses.color_pair(5)

            stdscr.addstr(row, 0, line[:width], color)
            row += 1

        stdscr.addstr(height - 1, 0, "Press q to quit"[:width])

        key = stdscr.getch()
        if key == ord("q"):
            break

        stdscr.refresh()


def main():
    rclpy.init()
    node = YoloDetectionViewer()

    try:
        curses.wrapper(draw, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()