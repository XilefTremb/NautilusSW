#!/usr/bin/env python3

import curses
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from contextlib import contextmanager


# Must stay in sync with PROFILING_STAGES in yolo_pipeline.py.
PROFILING_STAGES = [
    "camera",
    "image_cam_interval",
    "yolo_inference",
    "detection_processing",
    "angle_computation",
    "process_total",
    "publish",
    "detections_forward_interval",
    "detections_downward_interval",
    "image_annotated_fwd_cam_interval",
    "image_annotated_dwd_cam_interval",
    "end_2_end",
]

# Stages shown in the table (camera is used only for routing, not displayed).
DISPLAY_STAGES = [s for s in PROFILING_STAGES if s != "camera"]

# The *_interval publish stages are topic/camera specific, so each camera only
# displays the ones relevant to it (in place, between "publish" and "end_2_end").
CAMERA_INTERVAL_STAGES = {
    0: ["detections_forward_interval", "image_annotated_fwd_cam_interval"],
    1: ["detections_downward_interval", "image_annotated_dwd_cam_interval"],
}
_ALL_INTERVAL_STAGES = [s for stages in CAMERA_INTERVAL_STAGES.values() for s in stages]


def display_stages_for(camera):
    stages = []
    for s in DISPLAY_STAGES:
        if s in _ALL_INTERVAL_STAGES:
            continue
        if s == "end_2_end":
            stages.extend(CAMERA_INTERVAL_STAGES.get(camera, []))
        stages.append(s)
    return stages


# Width of the STAGE column, sized to fit the longest stage name.
STAGE_WIDTH = max(len(s) for s in DISPLAY_STAGES) + 2

CAMERA_LABELS = {0: "FORWARD CAM", 1: "DOWNWARD CAM"}

WINDOW = 100

class StageProfiler:
    """Lightweight per-stage timing collector.

    Records named time deltas (in milliseconds) for the current frame, keeps a
    rolling history per stage for console stats, and can serialize the current
    frame into the fixed PROFILING_STAGES order for publishing.
    """

    def __init__(self, node, stages, window=100):
        self.node = node
        self.stages = stages
        self.history = {s: deque(maxlen=window) for s in stages}
        self.frame = {}
        self._starts = {}
        self._last_event = {}

    def reset_frame(self):
        self.frame = {}

    def record(self, name, dt_ms):
        if dt_ms is None:
            return
        self.frame[name] = dt_ms
        if name in self.history:
            self.history[name].append(dt_ms)

    @contextmanager
    def span(self, name):
        t = time.perf_counter()
        try:
            yield
        finally:
            self.record(name, (time.perf_counter() - t) * 1000.0)

    def event_interval(self, name):
        """Return ms elapsed since the previous call with the same name."""
        now = time.perf_counter()
        last = self._last_event.get(name)
        self._last_event[name] = now
        if last is None:
            return None
        return (now - last) * 1000.0

    def build_msg_data(self):
        return [float(self.frame.get(s, float('nan'))) for s in self.stages]

    def log_console(self):
        parts = []
        for s in self.stages:
            if s == "camera":
                continue
            v = self.frame.get(s)
            if v is not None:
                parts.append(f"{s}={v:.1f}ms")
        if parts:
            self.node.get_logger().info("[PROFILE] " + "  ".join(parts))


class YoloProfilingViewer(Node):
    def __init__(self):
        super().__init__("yolo_profiling_viewer")

        self.declare_parameter("profiling_topic", "/yolo/profiling")
        topic = self.get_parameter("profiling_topic").get_parameter_value().string_value

        self.sub = self.create_subscription(
            Float32MultiArray, topic, self.profiling_callback, 10
        )

        self.topic_name = topic

        # history[camera][stage] -> deque of ms values
        self.history = {
            0: {s: deque(maxlen=WINDOW) for s in DISPLAY_STAGES},
            1: {s: deque(maxlen=WINDOW) for s in DISPLAY_STAGES},
        }
        self.last_value = {0: {}, 1: {}}
        self.last_msg_time = {0: None, 1: None}
        self.frame_count = {0: 0, 1: 0}

    def profiling_callback(self, msg: Float32MultiArray):
        if len(msg.data) != len(PROFILING_STAGES):
            self.get_logger().warn(
                f"Unexpected profiling array size: {len(msg.data)} "
                f"(expected {len(PROFILING_STAGES)})"
            )
            return

        values = dict(zip(PROFILING_STAGES, msg.data))
        camera = int(values.get("camera", 0.0))
        if camera not in self.history:
            return

        for stage in DISPLAY_STAGES:
            v = values.get(stage, float("nan"))
            if v == v:  # not NaN
                self.history[camera][stage].append(v)
                self.last_value[camera][stage] = v

        self.last_msg_time[camera] = time.time()
        self.frame_count[camera] += 1


def _stats(values):
    if not values:
        return None, None, None
    return values[-1], sum(values) / len(values), max(values)


def _color_for(stage, last, end_2_end_mean):
    """Green/yellow/red heuristic per stage."""
    if last is None:
        return curses.color_pair(5)

    # image_cam_interval and end_2_end depend on the pipeline/camera rate, which
    # may be tuned intentionally, so they are left neutral to avoid confusion.
    # The *_interval stages are also rates (not slices of end_2_end), so keep them neutral.
    if stage in ("image_cam_interval", "end_2_end") or stage.endswith("_interval"):
        return curses.color_pair(5)

    # For work stages, flag the ones eating most of the frame budget.
    if end_2_end_mean and end_2_end_mean > 0:
        share = last / end_2_end_mean
        if share > 0.5:
            return curses.color_pair(4)
        if share > 0.25:
            return curses.color_pair(3) | curses.A_BOLD
    return curses.color_pair(1)


def draw(stdscr, node: YoloProfilingViewer):
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

        title = f"YOLO Profiling Viewer: {node.topic_name}"
        stdscr.addstr(0, 0, title[:width], curses.A_BOLD)

        row = 2

        for camera in (0, 1):
            if row >= height - 1:
                break

            label = CAMERA_LABELS.get(camera, f"CAM {camera}")

            last_time = node.last_msg_time[camera]
            if last_time is None:
                status = f"{label}: no data yet"
                status_color = curses.color_pair(5)
            else:
                age = time.time() - last_time
                end_2_end = node.history[camera]["end_2_end"]
                _, ft_mean, _ = _stats(list(end_2_end))
                hz = (1000.0 / ft_mean) if ft_mean else 0.0
                status = (
                    f"{label}: last {age:.2f}s ago | frames {node.frame_count[camera]} "
                    f"| ~{hz:.1f} Hz"
                )
                status_color = (curses.color_pair(3) | curses.A_BOLD) if age > 1.0 else curses.color_pair(1)

            stdscr.addstr(row, 0, status[:width], curses.A_BOLD | status_color)
            row += 1

            header = (
                f"  {'STAGE':<{STAGE_WIDTH}} {'LAST(ms)':>10} {'MEAN(ms)':>10} "
                f"{'MAX(ms)':>10} {'N':>5}"
            )
            stdscr.addstr(row, 0, header[:width], curses.A_BOLD)
            row += 1
            stdscr.addstr(row, 0, ("  " + "-" * (len(header) - 2))[:width])
            row += 1

            _, ft_mean, _ = _stats(list(node.history[camera]["end_2_end"]))

            for stage in display_stages_for(camera):
                if row >= height - 1:
                    break

                vals = list(node.history[camera][stage])
                last, mean, mx = _stats(vals)

                if last is None:
                    line = (
                        f"  {stage:<{STAGE_WIDTH}} {'-':>10} {'-':>10} {'-':>10} {0:>5}"
                    )
                    color = curses.color_pair(5)
                else:
                    line = (
                        f"  {stage:<{STAGE_WIDTH}} {last:>10.2f} {mean:>10.2f} "
                        f"{mx:>10.2f} {len(vals):>5}"
                    )
                    color = _color_for(stage, last, ft_mean)

                stdscr.addstr(row, 0, line[:width], color)
                row += 1

            row += 1  # blank line between cameras

        stdscr.addstr(height - 1, 0, "Press q to quit"[:width])

        key = stdscr.getch()
        if key == ord("q"):
            break

        stdscr.refresh()


def main():
    rclpy.init()
    node = YoloProfilingViewer()

    try:
        curses.wrapper(draw, node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
