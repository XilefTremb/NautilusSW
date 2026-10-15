import subprocess

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QRadioButton, QButtonGroup, QFrame
)


class NodeRow(QFrame):
    def __init__(self, name, command_function):
        super().__init__()

        self.name = name
        self.command_function = command_function
        self.process = None

        layout = QHBoxLayout(self)

        self.title = QLabel(name)
        self.title.setObjectName("NodeTitle")

        self.status = QLabel("● STOPPED")
        self.status.setObjectName("NodeStopped")

        self.button = QPushButton("START")
        self.button.setObjectName("NodeButton")
        self.button.clicked.connect(self.toggle_node)

        layout.addWidget(self.title)
        layout.addStretch()
        layout.addWidget(self.status)
        layout.addWidget(self.button)

    def toggle_node(self):
        if self.process is None:
            self.start_node()
        else:
            self.stop_node()

    def start_node(self):
        command = self.command_function()

        try:
            self.process = subprocess.Popen(command)
            self.status.setText("● RUNNING")
            self.status.setObjectName("NodeRunning")
            self.button.setText("STOP")
            self.refresh_style()

        except Exception as e:
            print(f"[ERROR] {self.name}: {e}")
            self.process = None

    def stop_node(self):
        if self.process:
            self.process.terminate()
            self.process.wait(timeout=3)
            self.process = None

        self.status.setText("● STOPPED")
        self.status.setObjectName("NodeStopped")
        self.button.setText("START")
        self.refresh_style()

    def refresh_style(self):
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)


class NodesPage(QWidget):
    def __init__(self):
        super().__init__()

        main = QVBoxLayout(self)

        title = QLabel("NAUTILUS // NODE CONTROL")
        title.setObjectName("PageTitle")
        main.addWidget(title)

        # MODE
        mode_layout = QHBoxLayout()

        mode_title = QLabel("MODE")
        mode_title.setObjectName("SectionTitle")

        self.sim_button = QRadioButton("SIMULATION")
        self.real_button = QRadioButton("REAL")
        self.real_button.setChecked(True)

        self.mode_group = QButtonGroup(self)
        self.mode_group.addButton(self.sim_button)
        self.mode_group.addButton(self.real_button)

        mode_layout.addWidget(mode_title)
        mode_layout.addStretch()
        mode_layout.addWidget(self.sim_button)
        mode_layout.addWidget(self.real_button)

        main.addLayout(mode_layout)

        # NODES
        self.dvl = NodeRow("DVL", self.dvl_command)
        self.imu = NodeRow("IMU", self.imu_command)
        self.camera = NodeRow("CAMERA", self.camera_command)
        self.yolo = NodeRow("YOLO", self.yolo_command)

        for node in [self.dvl, self.imu, self.camera, self.yolo]:
            main.addWidget(node)

        main.addStretch()

    def is_sim(self):
        return self.sim_button.isChecked()

    def dvl_command(self):
        return [
            "ros2", "run",
            "nautilus_sensors",
            "dvl_sensor_node"
        ]

    def imu_command(self):
        return [
            "ros2", "run",
            "xsens_mti_ros2_driver",
            "xsens_mti_node"
        ]

    def camera_command(self):
        return [
            "ros2", "run",
            "nautilus_sensors",
            "stream_threaded"
        ]

    def yolo_command(self):
        mode = "--sim" if self.is_sim() else "--real"

        return [
            "ros2", "run",
            "nautilus_mission",
            "yolo_pipeline",
            "--",
            "--bbox",
            mode
        ]