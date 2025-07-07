#!/usr/bin/env python3

import sys
import threading
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


class SensorGui(Node):
    def __init__(self):
        super().__init__('sensor_gui')

        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Sous-marin - Données capteurs")

        self.layout = QVBoxLayout()

        self.temp_label = QLabel("Température: -- °C")
        self.humidity_label = QLabel("Humidité: -- %")
        self.motor_label = QLabel("Courants moteurs: --")

        self.layout.addWidget(self.temp_label)
        self.layout.addWidget(self.humidity_label)
        self.layout.addWidget(self.motor_label)

        self.window.setLayout(self.layout)
        self.window.show()

        # Abonnements ROS2
        self.create_subscription(Float32, 'temperature', self.temp_callback, 10)
        self.create_subscription(Float32, 'humidity', self.humidity_callback, 10)
        self.create_subscription(Float64MultiArray, 'motor_currents', self.motor_callback, 10)

    def temp_callback(self, msg):
        # Met à jour le label dans le thread GUI via signal/slot ou QTimer, ici simplifié :
        self.temp_label.setText(f"Température: {msg.data:.2f} °C")

    def humidity_callback(self, msg):
        self.humidity_label.setText(f"Humidité: {msg.data:.2f} %")

    def motor_callback(self, msg):
        currents_str = ", ".join(f"{c:.2f}" for c in msg.data)
        self.motor_label.setText(f"Courants moteurs: {currents_str}")


def ros_spin(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    sensor_gui = SensorGui()

    # Spin ROS dans thread séparé
    ros_thread = threading.Thread(target=ros_spin, args=(sensor_gui,), daemon=True)
    ros_thread.start()

    # Exécute la boucle PyQt dans le thread principal
    ret = sensor_gui.app.exec_()

    sensor_gui.destroy_node()
    rclpy.shutdown()

    ros_thread.join()

    sys.exit(ret)


if __name__ == '__main__':
    main()
