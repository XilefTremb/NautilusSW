# pages/dashboard.py

from unicodedata import name

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel
from widgets import DataCard, MotorCard


class DashboardPage(QWidget):

    def __init__(self, data_manager):
        super().__init__()

        self.data = data_manager
        main = QVBoxLayout(self)

        # TITLE
        title = QLabel("NAUTILUS // LIVE TELEMETRY")
        title.setObjectName("PageTitle")
        main.addWidget(title)

        # MAIN DATA
        top = QHBoxLayout()

        self.depth = DataCard("DEPTH", "m")
        self.voltage = DataCard("BATTERY", "V")
        self.current = DataCard("TOTAL CURRENT", "A")

        for widget in [self.depth, self.voltage, self.current]:
            top.addWidget(widget)

        main.addLayout(top)

        # IMU
        imu_widget = QWidget()
        imu_layout = QGridLayout(imu_widget)

        imu_title = QLabel("IMU")
        imu_title.setObjectName("SectionTitle")

        self.roll = DataCard("ROLL", "°")
        self.pitch = DataCard("PITCH", "°")
        self.yaw = DataCard("YAW", "°")

        imu_layout.addWidget(imu_title, 0, 0, 1, 3)

        for col, widget in enumerate([self.roll, self.pitch, self.yaw]):
            imu_layout.addWidget(widget, 1, col)

        # DVL
        dvl_widget = QWidget()
        dvl_layout = QGridLayout(dvl_widget)

        dvl_title = QLabel("DVL")
        dvl_title.setObjectName("SectionTitle")

        self.dvl_x = DataCard("VELOCITY X", "m/s")
        self.dvl_y = DataCard("VELOCITY Y", "m/s")
        self.dvl_z = DataCard("VELOCITY Z", "m/s")

        dvl_layout.addWidget(dvl_title, 0, 0, 1, 3)

        for col, widget in enumerate([self.dvl_x, self.dvl_y, self.dvl_z]):
            dvl_layout.addWidget(widget, 1, col)

        # SENSORS
        sensors = QHBoxLayout()
        sensors.addWidget(imu_widget)
        sensors.addWidget(dvl_widget)
        main.addLayout(sensors)

        # CAMERA
        camera_widget = QWidget()
        camera_layout = QGridLayout(camera_widget)

        camera_title = QLabel("CAMERA")
        camera_title.setObjectName("SectionTitle")

        self.camera_depth = DataCard("CAMERA DEPTH", "m")
        self.inference_mode = DataCard("INFERENCE MODE", "")
        self.detection_forward = DataCard("DETECTION FORWARD", "")
        self.detection_downward = DataCard("DETECTION DOWNWARD", "")

        camera_layout.addWidget(camera_title, 0, 0, 1, 4)

        for col, widget in enumerate([self.camera_depth, self.inference_mode, self.detection_forward, self.detection_downward]):
            camera_layout.addWidget(widget, 1, col)
            
        main.addWidget(camera_widget)

        # MOTORS
        motor_title = QLabel("THRUSTERS")
        motor_title.setObjectName("SectionTitle")
        main.addWidget(motor_title)

        motor_grid = QGridLayout()
        self.motors = []

        for i in range(8):
            motor = MotorCard(i + 1)
            self.motors.append(motor)
            motor_grid.addWidget(motor, i // 4, i % 4)

        main.addLayout(motor_grid)
        main.addStretch()

        self.data.data_updated.connect(self.update_data)

    def update_data(self, name, value):
        mapping = {
            "Depth": self.depth,
            "Battery Voltage": self.voltage,
            "Battery Current": self.current,
            "IMU Roll": self.roll,
            "IMU Pitch": self.pitch,
            "IMU Yaw": self.yaw,
            "DVL X": self.dvl_x,
            "DVL Y": self.dvl_y,
            "DVL Z": self.dvl_z
        }

        if name in mapping:
            mapping[name].set_value(value)

        for i, motor in enumerate(self.motors, 1):
            if name == f"Motor {i} PWM":
                motor.set_pwm(value)
            elif name == f"Motor {i} Current":
                motor.set_current(value)

        if name == "Battery Voltage":
            self.voltage.set_alert(value < 12.4)

        elif name == "Battery Current":
            self.current.set_alert(value > 30)

        elif name == "Depth":
            self.depth.set_alert(value > 5)