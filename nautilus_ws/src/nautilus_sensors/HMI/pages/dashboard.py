# pages/dashboard.py

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
)

from widgets import DataCard, MotorCard


class DashboardPage(QWidget):

    def __init__(self, data_manager):

        super().__init__()

        self.data = data_manager

        main = QVBoxLayout(self)

        # ===============================
        # TITLE
        # ===============================

        title = QLabel("NAUTILUS // LIVE TELEMETRY")
        title.setObjectName("PageTitle")

        main.addWidget(title)

        # ===============================
        # MAIN DATA
        # ===============================

        top = QHBoxLayout()

        self.depth = DataCard(
            "DEPTH",
            "m"
        )

        self.voltage = DataCard(
            "BATTERY",
            "V"
        )

        self.current = DataCard(
            "TOTAL CURRENT",
            "A"
        )

        top.addWidget(self.depth)
        top.addWidget(self.voltage)
        top.addWidget(self.current)

        main.addLayout(top)

        # ===============================
        # IMU + DVL
        # ===============================

        sensors = QHBoxLayout()

        imu_widget = QWidget()
        imu_layout = QGridLayout(imu_widget)

        imu_title = QLabel("IMU")
        imu_title.setObjectName("SectionTitle")

        self.roll = DataCard(
            "ROLL",
            "°"
        )

        self.pitch = DataCard(
            "PITCH",
            "°"
        )

        self.yaw = DataCard(
            "YAW",
            "°"
        )

        imu_layout.addWidget(
            imu_title,
            0, 0, 1, 3
        )

        imu_layout.addWidget(
            self.roll,
            1, 0
        )

        imu_layout.addWidget(
            self.pitch,
            1, 1
        )

        imu_layout.addWidget(
            self.yaw,
            1, 2
        )

        dvl_widget = QWidget()
        dvl_layout = QGridLayout(dvl_widget)

        dvl_title = QLabel("DVL")
        dvl_title.setObjectName("SectionTitle")

        self.dvl_x = DataCard(
            "VELOCITY X",
            "m/s"
        )

        self.dvl_y = DataCard(
            "VELOCITY Y",
            "m/s"
        )

        self.dvl_z = DataCard(
            "VELOCITY Z",
            "m/s"
        )

        dvl_layout.addWidget(
            dvl_title,
            0, 0, 1, 3
        )

        dvl_layout.addWidget(
            self.dvl_x,
            1, 0
        )

        dvl_layout.addWidget(
            self.dvl_y,
            1, 1
        )

        dvl_layout.addWidget(
            self.dvl_z,
            1, 2
        )

        sensors.addWidget(imu_widget)
        sensors.addWidget(dvl_widget)

        main.addLayout(sensors)

        # ===============================
        # MOTORS
        # ===============================

        motor_title = QLabel(
            "THRUSTERS"
        )

        motor_title.setObjectName(
            "SectionTitle"
        )

        main.addWidget(motor_title)

        motor_grid = QGridLayout()

        self.motors = []

        for i in range(8):

            motor = MotorCard(i + 1)

            self.motors.append(motor)

            row = i // 4
            col = i % 4

            motor_grid.addWidget(
                motor,
                row,
                col
            )

        main.addLayout(motor_grid)

        main.addStretch()

        self.data.data_updated.connect(
            self.update_data
        )

    # ==========================================================
    # DATA
    # ==========================================================

    def update_data(self, name, value):

        mapping = {

            "Depth":
                self.depth,

            "Battery Voltage":
                self.voltage,

            "Battery Current":
                self.current,

            "IMU Roll":
                self.roll,

            "IMU Pitch":
                self.pitch,

            "IMU Yaw":
                self.yaw,

            "DVL X":
                self.dvl_x,

            "DVL Y":
                self.dvl_y,

            "DVL Z":
                self.dvl_z,
        }

        if name in mapping:
            mapping[name].set_value(value)

        for i in range(8):

            if name == f"Motor {i+1} PWM":
                self.motors[i].set_pwm(
                    value
                )

            if name == f"Motor {i+1} Current":
                self.motors[i].set_current(
                    value
                )