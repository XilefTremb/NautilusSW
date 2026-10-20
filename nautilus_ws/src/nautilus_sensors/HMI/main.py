import sys
import random
import threading
import argparse

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from data_manager import DataManager
from main_window import MainWindow


def select_mode():
    parser = argparse.ArgumentParser(
        description="Nautilus HMI"
    )

    group = parser.add_mutually_exclusive_group()

    group.add_argument(
        "-w",
        action="store_true",
        help="Windows simulation mode"
    )

    group.add_argument(
        "-l",
        action="store_true",
        help="Linux ROS2 live mode"
    )

    args = parser.parse_args()

    if args.w:
        return "w"

    if args.l:
        return "l"

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
        # Depth
        data.update("Depth", random.uniform(0, 10))

        # Battery
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

        # Motors
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
        print("[ERROR] Unable to import ROS2.")
        print("Make sure you are on Ubuntu and have sourced ROS2.")
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

    app = QApplication([sys.argv[0]])
    data = DataManager()

    simulation_timer = None
    rclpy_module = None
    ros_node = None

    # =========================================
    # WINDOWS / SIMULATION
    # =========================================

    if mode == "w":
        simulation_timer = start_simulation(data)

    # =========================================
    # LINUX / ROS2
    # =========================================

    elif mode == "l":
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

    if mode == "l":
        ros_node.destroy_node()
        rclpy_module.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()