import random
import time
from collections import deque

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor


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

DISPLAY_STAGES = [s for s in PROFILING_STAGES if s != "camera"]

CAMERA_INTERVAL_STAGES = {
    0: ["detections_forward_interval", "image_annotated_fwd_cam_interval"],
    1: ["detections_downward_interval", "image_annotated_dwd_cam_interval"],
}

_ALL_INTERVAL_STAGES = [
    s for stages in CAMERA_INTERVAL_STAGES.values()
    for s in stages
]

CAMERA_LABELS = {
    0: "FORWARD CAM",
    1: "DOWNWARD CAM"
}

WINDOW = 100


def display_stages_for(camera):
    stages = []

    for stage in DISPLAY_STAGES:
        if stage in _ALL_INTERVAL_STAGES:
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

        self.history = {
            stage: deque(maxlen=WINDOW)
            for stage in DISPLAY_STAGES
        }

        self.last_msg_time = None
        self.frame_count = 0

        main = QVBoxLayout(self)

        # =========================================
        # HEADER
        # =========================================

        header = QHBoxLayout()

        self.camera_name = QLabel(CAMERA_LABELS[camera])
        self.camera_name.setObjectName("ProfilingCameraTitle")

        self.status = QLabel("● NO DATA")
        self.status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        header.addWidget(self.camera_name)
        header.addStretch()
        header.addWidget(self.status)

        main.addLayout(header)

        # =========================================
        # STATS
        # =========================================

        stats_layout = QHBoxLayout()

        self.frequency = self.create_stat("FREQUENCY", "-- Hz")
        self.frames = self.create_stat("FRAMES", "0")
        self.last_message = self.create_stat("LAST MESSAGE", "-- s")
        self.end_to_end = self.create_stat("END TO END", "-- ms")

        stats_layout.addWidget(self.frequency)
        stats_layout.addWidget(self.frames)
        stats_layout.addWidget(self.last_message)
        stats_layout.addWidget(self.end_to_end)

        main.addLayout(stats_layout)

        # =========================================
        # TABLE
        # =========================================

        stages = display_stages_for(camera)

        self.table = QTableWidget(len(stages), 5)
        self.table.setHorizontalHeaderLabels([
            "STAGE",
            "LAST (ms)",
            "MEAN (ms)",
            "MAX (ms)",
            "N"
        ])

        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.setAlternatingRowColors(True)

        header_view = self.table.horizontalHeader()
        header_view.setSectionResizeMode(0, QHeaderView.Stretch)
        header_view.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header_view.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header_view.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        header_view.setSectionResizeMode(4, QHeaderView.ResizeToContents)

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
        # =========================================
        # CAMERA STATUS
        # =========================================

        if self.last_msg_time is None:
            self.status.setText("● NO DATA")
            self.status.setStyleSheet("color: #8391a7; font-weight: bold;")
            return

        age = time.time() - self.last_msg_time

        if age > 1.0:
            self.status.setText("● STALE")
            self.status.setStyleSheet("color: #FFC107; font-weight: bold;")
        else:
            self.status.setText("● LIVE")
            self.status.setStyleSheet("color: #00C853; font-weight: bold;")

        # =========================================
        # GLOBAL STATS
        # =========================================

        end_values = list(self.history["end_2_end"])
        last_e2e, mean_e2e, max_e2e = stats(end_values)

        hz = 1000.0 / mean_e2e if mean_e2e else 0

        self.frequency.value_label.setText(f"{hz:.1f} Hz")
        self.frames.value_label.setText(str(self.frame_count))
        self.last_message.value_label.setText(f"{age:.2f} s")

        if last_e2e is not None:
            self.end_to_end.value_label.setText(f"{last_e2e:.1f} ms")

        # =========================================
        # TABLE
        # =========================================

        for stage in display_stages_for(self.camera):
            values = list(self.history[stage])
            last, mean, maximum = stats(values)

            row = self.stage_rows[stage]

            if last is None:
                continue

            self.table.item(row, 1).setText(f"{last:.2f}")
            self.table.item(row, 2).setText(f"{mean:.2f}")
            self.table.item(row, 3).setText(f"{maximum:.2f}")
            self.table.item(row, 4).setText(str(len(values)))

            color = self.get_stage_color(stage, last, mean_e2e)

            for column in range(5):
                self.table.item(row, column).setForeground(QColor(color))

    def get_stage_color(self, stage, last, end_2_end_mean):
        # Même logique que ton viewer curses

        if stage == "image_cam_interval" or stage == "end_2_end":
            return "#dce6f2"

        if stage.endswith("_interval"):
            return "#dce6f2"

        if end_2_end_mean and end_2_end_mean > 0:
            share = last / end_2_end_mean

            if share > 0.5:
                return "#FF5252"

            if share > 0.25:
                return "#FFC107"

        return "#00C853"


class ProfilingPage(QWidget):

    def __init__(self):
        super().__init__()

        main = QVBoxLayout(self)

        # =========================================
        # TITLE
        # =========================================

        title_layout = QHBoxLayout()

        title = QLabel("NAUTILUS // YOLO PROFILING")
        title.setObjectName("PageTitle")

        self.topic_label = QLabel("/yolo/profiling")
        self.topic_label.setStyleSheet("color: #8391a7;")

        title_layout.addWidget(title)
        title_layout.addStretch()
        title_layout.addWidget(self.topic_label)

        main.addLayout(title_layout)

        # =========================================
        # CAMERAS
        # =========================================

        cameras = QHBoxLayout()

        self.forward = CameraProfilingWidget(0)
        self.downward = CameraProfilingWidget(1)

        cameras.addWidget(self.forward)
        cameras.addWidget(self.downward)

        main.addLayout(cameras)

        # =========================================
        # REFRESH TIMER
        # =========================================

        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.refresh_status)
        self.refresh_timer.start(100)

        # =========================================
        # WINDOWS TEST MODE
        # =========================================

        self.simulation_timer = QTimer()
        self.simulation_timer.timeout.connect(self.generate_fake_data)
        self.simulation_timer.start(100)

    def refresh_status(self):
        self.forward.update_display()
        self.downward.update_display()

    def generate_fake_data(self):
        """
        Seulement pour tester l'affichage sous Windows.
        À enlever/désactiver lorsque ROS sera connecté.
        """

        for camera_widget in [self.forward, self.downward]:

            yolo = random.uniform(35, 70)
            detection = random.uniform(2, 8)
            angle = random.uniform(0.5, 3)
            publish = random.uniform(1, 5)

            process_total = yolo + detection + angle

            values = {
                "image_cam_interval": random.uniform(90, 110),
                "yolo_inference": yolo,
                "detection_processing": detection,
                "angle_computation": angle,
                "process_total": process_total,
                "publish": publish,
                "end_2_end": random.uniform(90, 120),
            }

            if camera_widget.camera == 0:
                values["detections_forward_interval"] = random.uniform(90, 110)
                values["image_annotated_fwd_cam_interval"] = random.uniform(90, 110)

            else:
                values["detections_downward_interval"] = random.uniform(90, 110)
                values["image_annotated_dwd_cam_interval"] = random.uniform(90, 110)

            camera_widget.add_data(values)