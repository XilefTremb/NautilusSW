import subprocess

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QRadioButton, QButtonGroup, QFrame
)


class NodeRow(QFrame):
    def __init__(self, name, command, kill_pattern):
        super().__init__()
        self.command, self.kill_pattern, self.process = command, kill_pattern, None

        layout = QHBoxLayout(self)
        self.title = QLabel(name)
        self.status = QLabel("● STOPPED")
        self.button = QPushButton("START")

        self.title.setObjectName("NodeTitle")
        self.status.setObjectName("NodeStopped")
        self.button.setObjectName("NodeButton")
        self.button.clicked.connect(self.toggle)

        layout.addWidget(self.title)
        layout.addStretch()
        layout.addWidget(self.status)
        layout.addWidget(self.button)

    def toggle(self):
        if self.process is None:
            try:
                command = self.command() if callable(self.command) else self.command
                self.process = subprocess.Popen(command)
                self.set_status(True)
            except Exception as e:
                print(f"[ERROR] {self.title.text()}: {e}")
        else:
            self.stop()

    def stop(self):
        subprocess.run(
            ["pkill", "-9", "-f", self.kill_pattern],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        self.process = None
        self.set_status(False)

    def set_status(self, running):
        self.status.setText("● RUNNING" if running else "● STOPPED")
        self.status.setObjectName("NodeRunning" if running else "NodeStopped")
        self.button.setText("STOP" if running else "START")
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)


class NodesPage(QWidget):
    def __init__(self):
        super().__init__()
        main = QVBoxLayout(self)

        title = QLabel("NAUTILUS // NODE CONTROL")
        title.setObjectName("PageTitle")
        main.addWidget(title)

        mode = QHBoxLayout()
        mode_title = QLabel("MODE")
        mode_title.setObjectName("SectionTitle")

        self.sim_button = QRadioButton("SIMULATION")
        self.real_button = QRadioButton("REAL")
        self.real_button.setChecked(True)

        group = QButtonGroup(self)
        group.addButton(self.sim_button)
        group.addButton(self.real_button)

        mode.addWidget(mode_title)
        mode.addStretch()
        mode.addWidget(self.sim_button)
        mode.addWidget(self.real_button)
        main.addLayout(mode)

        nodes = [
            ("MAVPROXY",
             ["ros2", "launch", "nautilus_bringup", "mavproxy_router.launch.py"],
             "mavproxy"),

            ("CUBE INTERFACE",
             ["ros2", "run", "nautilus_controls", "cube_interface", "--auv"],
             "cube_interface"),

            ("DVL",
             ["ros2", "run", "nautilus_sensors", "dvl_sensor_node"],
             "dvl_sensor_node"),

            ("IMU",
             ["ros2", "run", "xsens_mti_ros2_driver", "xsens_mti_node"],
             "xsens_mti_node"),

            ("CAMERA",
             ["ros2", "run", "nautilus_sensors", "stream_threaded"],
             "stream_threaded"),

            ("YOLO", self.yolo_command, "yolo_pipeline")
        ]

        self.nodes = []
        for name, command, kill_pattern in nodes:
            node = NodeRow(name, command, kill_pattern)
            self.nodes.append(node)
            main.addWidget(node)

        main.addStretch()

    def yolo_command(self):
        mode = "--sim" if self.sim_button.isChecked() else "--real"
        return [
            "ros2", "run", "nautilus_mission", "yolo_pipeline",
            "--", "--bbox", mode]