import sys
import random

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from data_manager import DataManager
from main_window import MainWindow


def generate_fake_data(data):

    # Sous-marin
    data.update("Depth", random.uniform(1.8, 2.5))
    data.update("Battery Voltage", random.uniform(14.5, 16.0))
    data.update("Battery Current", random.uniform(5, 25))

    # IMU
    data.update("IMU Roll", random.uniform(-5, 5))
    data.update("IMU Pitch", random.uniform(-4, 4))
    data.update("IMU Yaw", random.uniform(175, 185))

    data.update("IMU Accel X", random.uniform(-0.2, 0.2))
    data.update("IMU Accel Y", random.uniform(-0.2, 0.2))
    data.update("IMU Accel Z", random.uniform(9.6, 10.0))

    # DVL
    data.update("DVL X", random.uniform(0.3, 0.5))
    data.update("DVL Y", random.uniform(-0.1, 0.1))
    data.update("DVL Z", random.uniform(-0.05, 0.05))

    # 8 T200
    for i in range(1, 9):

        data.update(
            f"Motor {i} PWM",
            random.uniform(1300, 1700)
        )

        data.update(
            f"Motor {i} Current",
            random.uniform(0, 15)
        )


def main():

    app = QApplication(sys.argv)

    data_manager = DataManager()

    window = MainWindow(data_manager)
    window.show()

    # Simulation des données à 10 Hz
    timer = QTimer()

    timer.timeout.connect(
        lambda: generate_fake_data(data_manager)
    )

    timer.start(100)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()