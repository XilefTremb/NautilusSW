# pages/profiling.py

import time
from collections import deque

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor


PROFILING_STAGES = [
    "camera", "image_cam_interval", "yolo_inference", "detection_processing",
    "angle_computation", "process_total", "publish",
    "detections_forward_interval", "detections_downward_interval",
    "image_annotated_fwd_cam_interval", "image_annotated_dwd_cam_interval",
    "end_2_end"
]

DISPLAY_STAGES = [s for s in PROFILING_STAGES if s != "camera"]

CAMERA_INTERVAL_STAGES = {
    0: ["detections_forward_interval", "image_annotated_fwd_cam_interval"],
    1: ["detections_downward_interval", "image_annotated_dwd_cam_interval"]
}

ALL_INTERVAL_STAGES = [s for stages in CAMERA_INTERVAL_STAGES.values() for s in stages]
CAMERA_LABELS = {0: "FORWARD CAM", 1: "DOWNWARD CAM"}
WINDOW = 100


def display_stages_for(camera):
    stages = []

    for stage in DISPLAY_STAGES:
        if stage in ALL_INTERVAL_STAGES:
            continue
        if stage == "end_2_end":
            stages.extend(CAMERA_INTERVAL_STAGES.get(camera, []))
        stages.append(stage)

    return stages


def stats(values):
    if not values:
        return None, None, None

    return values[-1], sum(values) / len(values), max(values)


class CameraProfilingWidget(QFrame):

    def __init__(self, camera):
        super().__init__()

        self.camera = camera
        self.setObjectName("ProfilingCard")

        self.history = {stage: deque(maxlen=WINDOW) for stage in DISPLAY_STAGES}
        self.last_msg_time = None
        self.frame_count = 0

        main = QVBoxLayout(self)

        # HEADER
        header = QHBoxLayout()

        self.camera_name = QLabel(CAMERA_LABELS[camera])
        self.camera_name.setObjectName("ProfilingCameraTitle")

        self.status = QLabel("● NO DATA")
        self.status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        header.addWidget(self.camera_name)
        header.addStretch()
        header.addWidget(self.status)

        main.addLayout(header)

        # STATS
        stats_layout = QHBoxLayout()

        self.frequency = self.create_stat("FREQUENCY", "-- Hz")
        self.frames = self.create_stat("FRAMES", "0")
        self.last_message = self.create_stat("LAST MESSAGE", "-- s")
        self.end_to_end = self.create_stat("END TO END", "-- ms")

        for widget in [self.frequency, self.frames, self.last_message, self.end_to_end]:
            stats_layout.addWidget(widget)

        main.addLayout(stats_layout)

        # TABLE
        stages = display_stages_for(camera)

        self.table = QTableWidget(len(stages), 5)
        self.table.setHorizontalHeaderLabels(["STAGE", "LAST (ms)", "MEAN (ms)", "MAX (ms)", "N"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.setAlternatingRowColors(True)

        header_view = self.table.horizontalHeader()
        header_view.setSectionResizeMode(0, QHeaderView.Stretch)

        for column in range(1, 5):
            header_view.setSectionResizeMode(column, QHeaderView.ResizeToContents)

        self.stage_rows = {}

        for row, stage in enumerate(stages):
            self.stage_rows[stage] = row

            item = QTableWidgetItem(stage)
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(row, 0, item)

            for column in range(1, 5):
                item = QTableWidgetItem("-")
                item.setTextAlignment(Qt.AlignCenter)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, column, item)

        main.addWidget(self.table)

    def create_stat(self, title, value):
        frame = QFrame()
        frame.setObjectName("ProfilingStat")

        layout = QVBoxLayout(frame)

        title_label = QLabel(title)
        title_label.setObjectName("ProfilingStatTitle")

        value_label = QLabel(value)
        value_label.setObjectName("ProfilingStatValue")
        value_label.setAlignment(Qt.AlignCenter)

        layout.addWidget(title_label)
        layout.addWidget(value_label)

        frame.value_label = value_label

        return frame

    def add_data(self, values):
        for stage in DISPLAY_STAGES:
            value = values.get(stage)

            if value is not None:
                self.history[stage].append(value)

        self.last_msg_time = time.time()
        self.frame_count += 1

        self.update_display()

    def update_display(self):
        if self.last_msg_time is None:
            self.status.setText("● NO DATA")
            self.status.setStyleSheet("color: #8391a7; font-weight: bold;")
            return

        age = time.time() - self.last_msg_time

        if age > 1:
            self.status.setText("● STALE")
            self.status.setStyleSheet("color: #FFC107; font-weight: bold;")
        else:
            self.status.setText("● LIVE")
            self.status.setStyleSheet("color: #00C853; font-weight: bold;")

        last_e2e, mean_e2e, _ = stats(list(self.history["end_2_end"]))
        hz = 1000 / mean_e2e if mean_e2e else 0

        self.frequency.value_label.setText(f"{hz:.1f} Hz")
        self.frames.value_label.setText(str(self.frame_count))
        self.last_message.value_label.setText(f"{age:.2f} s")

        if last_e2e is not None:
            self.end_to_end.value_label.setText(f"{last_e2e:.1f} ms")

        for stage in display_stages_for(self.camera):
            values = list(self.history[stage])
            last, mean, maximum = stats(values)

            if last is None:
                continue

            row = self.stage_rows[stage]

            self.table.item(row, 1).setText(f"{last:.2f}")
            self.table.item(row, 2).setText(f"{mean:.2f}")
            self.table.item(row, 3).setText(f"{maximum:.2f}")
            self.table.item(row, 4).setText(str(len(values)))

            color = self.get_stage_color(stage, last, mean_e2e)

            for column in range(5):
                self.table.item(row, column).setForeground(QColor(color))

    def get_stage_color(self, stage, last, end_2_end_mean):
        if stage == "image_cam_interval" or stage == "end_2_end" or stage.endswith("_interval"):
            return "#dce6f2"

        if end_2_end_mean:
            share = last / end_2_end_mean

            if share > 0.5:
                return "#FF5252"

            if share > 0.25:
                return "#FFC107"

        return "#00C853"


class ProfilingPage(QWidget):

    def __init__(self, data_manager):
        super().__init__()

        self.data = data_manager

        main = QVBoxLayout(self)

        # TITLE
        title_layout = QHBoxLayout()

        title = QLabel("NAUTILUS // YOLO PROFILING")
        title.setObjectName("PageTitle")

        topic_label = QLabel("/yolo/profiling")
        topic_label.setStyleSheet("color: #8391a7;")

        title_layout.addWidget(title)
        title_layout.addStretch()
        title_layout.addWidget(topic_label)

        main.addLayout(title_layout)

        # CAMERAS
        cameras = QHBoxLayout()

        self.forward = CameraProfilingWidget(0)
        self.downward = CameraProfilingWidget(1)

        cameras.addWidget(self.forward)
        cameras.addWidget(self.downward)

        main.addLayout(cameras)

        # DATA MANAGER
        self.data.profiling_updated.connect(self.update_profiling)

        # REFRESH
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_status)
        self.refresh_timer.start(100)

    def update_profiling(self, camera, values):
        if camera == 0:
            self.forward.add_data(values)

        elif camera == 1:
            self.downward.add_data(values)

    def refresh_status(self):
        self.forward.update_display()
        self.downward.update_display()