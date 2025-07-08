import sys
import threading
import random
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLineEdit, QProgressBar, QGroupBox, QRadioButton, QComboBox
)
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


class SubmarineGUI(Node):
    def __init__(self):
        super().__init__('submarine_gui')

        # ROS 2 Subscribers
        self.create_subscription(Float32, 'temperature', self.temp_callback, 10)
        self.create_subscription(Float32, 'humidity', self.humidity_callback, 10)
        self.create_subscription(Float64MultiArray, 'motor_currents', self.motor_callback, 10)

        # PyQt5 App
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('GUI Sous-Marin')
        self.layout = QVBoxLayout(self.window)
       

        self.build_top_panel()
        self.build_motors_panel()
        self.build_video_and_depth_panel()

        self.window.setLayout(self.layout)
        self.window.show()

    def build_top_panel(self):
        hbox = QHBoxLayout()

        # ----- Groupe Panneau de contrôle -----
        group_box = QGroupBox("Panneau de contrôle")
        grid_layout = QGridLayout()

        label = QLabel("Sélectionnez un programme")
        combo = QComboBox()
        combo.addItems(["Option 1", "Option 2", "Option 3", "Option 4"])

        self.go_btn = QPushButton("Go")
        self.stop_btn = QPushButton("Stop")
        self.manuel_btn = QPushButton("Manuel")
        self.automatique_btn = QPushButton("Auto")

        grid_layout.addWidget(label, 0, 0, 1, 2)
        grid_layout.addWidget(combo, 0, 2, 1, 2)
        grid_layout.addWidget(self.go_btn, 2, 2)
        grid_layout.addWidget(self.stop_btn, 2, 3)
        grid_layout.addWidget(self.manuel_btn, 3, 0)
        grid_layout.addWidget(self.automatique_btn, 3, 1)

        group_box.setLayout(grid_layout)

        # ----- Groupe Operation (à droite) -----
        operation_box = QGroupBox("Opérations")
        grid_layout_op = QGridLayout()
        self.temp_label = QLabel("Temp: -- °C")
        self.humidity_label = QLabel("Hum: -- %")
        self.battery_label = QLabel("Batterie:")
        self.batterie_bar = QProgressBar()

        grid_layout_op.addWidget(self.temp_label,2,0,2,1)
        grid_layout_op.addWidget(self.humidity_label,1,0,2,1)
        grid_layout_op.addWidget(self.battery_label,0,0,2,1)
        grid_layout_op.addWidget(self.batterie_bar,0,2,2,1)
        
        operation_box.setLayout(grid_layout_op)

        # Ajouter les deux groupes dans le layout horizontal
        hbox.addWidget(group_box)
        hbox.addWidget(operation_box)

        self.layout.addLayout(hbox)

    def build_motors_panel(self):
        grid = QGridLayout()
        self.motor_bars = []
        for i in range(8):
            amp = QProgressBar()
            amp.setOrientation(Qt.Vertical)
            amp.setRange(0, 40)
            amp.setValue(0)
            amp.setFixedSize(80, 200)
            label = QLabel(f"Moteur {i+1}")
            self.pwm = QLabel(f"PWM:")
            vbox = QVBoxLayout()
            vbox.addWidget(amp)
            vbox.addWidget(label)
            vbox.addWidget(self.pwm)
            container = QWidget()
            container.setLayout(vbox)
            grid.addWidget(container, 0, i)
            self.motor_bars.append(amp)

        group = QGroupBox("Moteurs")
        group.setLayout(grid)
        self.layout.addWidget(group)

    def build_video_and_depth_panel(self):
        layout = QHBoxLayout()

        group_box = QGroupBox("Camera")
        grid_layout = QGridLayout()

        self.cam1 = QLabel("Camera 1")
        self.cam1.setFixedSize(400, 300)
        self.cam1.setStyleSheet("background-color: gray")

        self.camera1_button= QPushButton("Camera 1")
        self.camera2_button= QPushButton("Camera 2")

        """
        self.depth_bar = QProgressBar()
        self.depth_bar.setOrientation(Qt.Vertical)
        self.depth_bar.setRange(0, 100)
        self.depth_bar.setValue(30)
        self.depth_bar.setFormat("Profondeur")
        """

        grid_layout.addWidget(self.cam1, 0, 0, 1, 4)
        grid_layout.addWidget(self.camera1_button, 1, 0)
        grid_layout.addWidget(self.camera2_button, 1, 1)

        group_box.setLayout(grid_layout)  # <== Ajout manquant

        layout.addWidget(group_box)

        self.layout.addLayout(layout)


    # ROS Callbacks
    def temp_callback(self, msg):
        self.temp_label.setText(f"Temp: {msg.data:.1f} °C")

    def humidity_callback(self, msg):
        self.humidity_label.setText(f"Hum: {msg.data:.1f} %")

    def motor_callback(self, msg):
        for i in range(min(8, len(msg.data))):
            self.motor_bars[i].setValue(int(msg.data[i] * 10))

    def run(self):
        sys.exit(self.app.exec_())
        


def main():
    rclpy.init()
    gui = SubmarineGUI()
    ros_thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    ros_thread.start()
    gui.run()
    gui.destroy_node()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == '__main__':
    main()
