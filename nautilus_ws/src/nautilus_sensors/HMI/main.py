import sys
import random
import threading

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from data_manager import DataManager
from main_window import MainWindow


def select_mode():
    print("\n==============================")
    print("       NAUTILUS HMI")
    print("==============================")
    print("[w] Windows - Simulation")
    print("[l] Linux   - ROS2 Live")
    print("==============================")

    while True:
        mode = input("Mode: ").strip().lower()

        if mode in ("w", "l"):
            return mode

        print("Choix invalide. Entrer 'w' ou 'l'.")


def start_simulation(data):
    print("[HMI] Simulation mode")

    timer = QTimer()

    def generate_fake_data():
        # Profondeur
        data.update("Depth", random.uniform(0, 10))

        # Batterie
        data.update("Battery Voltage", random.uniform(11.0, 16.8))
        data.update("Battery Current", random.uniform(5, 35))

        # IMU
        data.update("IMU Roll", random.uniform(-15, 15))
        data.update("IMU Pitch", random.uniform(-15, 15))
        data.update("IMU Yaw", random.uniform(-180, 180))

        data.update("IMU Accel X", random.uniform(-1, 1))
        data.update("IMU Accel Y", random.uniform(-1, 1))
        data.update("IMU Accel Z", random.uniform(9, 10))

        # DVL
        data.update("DVL X", random.uniform(-1, 1))
        data.update("DVL Y", random.uniform(-1, 1))
        data.update("DVL Z", random.uniform(-0.5, 0.5))

        # Moteurs
        for i in range(1, 9):
            data.update(f"Motor {i} PWM", random.uniform(1200, 1800))
            data.update(f"Motor {i} Current", random.uniform(0, 20))

    timer.timeout.connect(generate_fake_data)
    timer.start(100)

    return timer


def start_ros(data):
    print("[HMI] ROS2 live mode")

    try:
        import rclpy
        from ros_node import NautilusROSNode

    except ImportError as e:
        print("[ERROR] Impossible d'importer ROS2.")
        print("Assure-toi d'être sur Ubuntu et d'avoir sourcé ROS2.")
        print(e)
        sys.exit(1)

    rclpy.init()

    ros_node = NautilusROSNode(data)

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True
    )

    ros_thread.start()

    return rclpy, ros_node


def main():
    mode = select_mode()

    app = QApplication(sys.argv)
    data = DataManager()

    simulation_timer = None
    rclpy_module = None
    ros_node = None

    # =========================================
    # WINDOWS / SIMULATION
    # =========================================

    if mode == "w" or mode == "W":
        simulation_timer = start_simulation(data)

    # =========================================
    # LINUX / ROS2
    # =========================================

    elif mode == "l" or mode == "L":
        rclpy_module, ros_node = start_ros(data)

    # =========================================
    # HMI
    # =========================================

    window = MainWindow(data)
    window.show()

    exit_code = app.exec_()

    # =========================================
    # SHUTDOWN ROS
    # =========================================

    if mode == "l" or mode == "L":
        ros_node.destroy_node()
        rclpy_module.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()