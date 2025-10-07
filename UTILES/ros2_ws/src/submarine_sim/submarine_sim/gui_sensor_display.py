import sys
import threading
import subprocess
import time
from PySide2.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QProgressBar, QGroupBox, QComboBox, QFrame, QVBoxLayout, QWidget
)
from PySide2.QtCore import Qt, QTimer, QObject, Signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


class GUISignals(QObject):
    update_motor = Signal(int, float)
    update_temp = Signal(float)
    update_humidity = Signal(float)


class SubmarineGUI(Node):
    def __init__(self):
        super().__init__('submarine_gui')

        # ROS 2 Subscribers
        self.create_subscription(Float32, 'temperature', self.temp_callback, 10)
        self.create_subscription(Float32, 'humidity', self.humidity_callback, 10)
        self.create_subscription(Float64MultiArray, 'motor_currents', self.motor_callback, 10)

        # PySide2 App setup
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('GUI Sous-Marin')
        self.layout = QVBoxLayout(self.window)

        self.signals = GUISignals()
        self.signals.update_motor.connect(self.set_motor_value)
        self.signals.update_temp.connect(self.set_temp)
        self.signals.update_humidity.connect(self.set_humidity)

        self.build_top_panel()
        self.build_motors_panel()
        self.build_down_panel()

        self.window.setLayout(self.layout)
        self.window.show()

        QTimer.singleShot(3000, self.move_window_right)  # décaler après affichage

    def build_top_panel(self):
        hbox = QHBoxLayout()

        # Groupe Box pour le panneau de contrôle
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

        # GRoupe Box pour les opérations
        operation_box = QGroupBox("Opérations")
        grid_layout_op = QGridLayout()

        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)
        line.setStyleSheet("background-color: black; width: 2px;")
        
        self.tempboit_label = QLabel("Temperature boite: -- °C")
        self.tempbatt_label = QLabel("Temperature batterie: -- °C")
        self.humidity_label = QLabel("Humidite boite: -- %")
        self.battery_label = QLabel("Batterie:")
        self.batterie_bar = QProgressBar()

        grid_layout_op.addWidget(self.tempboit_label, 2, 0,1,2)
        grid_layout_op.addWidget(self.humidity_label, 1, 0,1,2)
        grid_layout_op.addWidget(self.battery_label, 0, 3)
        grid_layout_op.addWidget(self.batterie_bar, 0, 4)
        grid_layout_op.addWidget(self.tempbatt_label, 0, 0)
        grid_layout_op.addWidget(line, 0, 2, 4, 1)
        
        operation_box.setLayout(grid_layout_op)

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
            label = QLabel(f"Moteur {i+1}: -- N")
            pwm_label = QLabel("PWM:")
            vbox = QVBoxLayout()
            vbox.addWidget(amp)
            vbox.addWidget(label)
            vbox.addWidget(pwm_label)
            container = QWidget()
            container.setLayout(vbox)
            grid.addWidget(container, 0, i)
            self.motor_bars.append(amp)

        group = QGroupBox("Moteurs")
        group.setLayout(grid)
        self.layout.addWidget(group)

    def build_down_panel(self):
        layout = QHBoxLayout()

        # Groupe Box pour le panneau de caméra
        group_box = QGroupBox("Camera")
        grid_layout = QGridLayout()

        self.cam1 = QLabel("Camera 1")
        self.cam1.setFixedSize(400, 300)
        self.cam1.setStyleSheet("background-color: gray")

        self.camera1_button = QPushButton("Camera 1")
        self.camera2_button = QPushButton("Camera 2")

        grid_layout.addWidget(self.cam1, 0, 0, 1, 4)
        grid_layout.addWidget(self.camera1_button, 1, 0)
        grid_layout.addWidget(self.camera2_button, 1, 1)

        group_box.setLayout(grid_layout)
        layout.addWidget(group_box)
        self.layout.addLayout(layout)

        # Groupe Box pour le panneau de instrumentaion
        group_box_int = QGroupBox("Instrumentation")
        grid_layout_int = QGridLayout()

        self.x_label = QLabel("x: -- m")
        self.y_label = QLabel("y: -- m")
        self.z_label = QLabel("z: -- m")
        self.pitch_label = QLabel("Pitch: -- °")
        self.roll_label = QLabel("Roll: -- °")
        self.yaw_label = QLabel("Yaw: -- °")
        self.vitesse_label = QLabel("Vitesse: -- m/s")
        self.proondeur_label = QLabel("Profondeur: -- m")

        grid_layout_int.addWidget(self.x_label, 0, 0, 1, 1)
        grid_layout_int.addWidget(self.y_label, 1, 0, 1, 1)
        grid_layout_int.addWidget(self.z_label, 2, 0, 1, 1)
        grid_layout_int.addWidget(self.pitch_label, 0, 1, 1, 1)
        grid_layout_int.addWidget(self.roll_label, 1, 1, 1, 1)
        grid_layout_int.addWidget(self.yaw_label, 2, 1, 1, 1)
        grid_layout_int.addWidget(self.vitesse_label, 3, 0, 1, 1)
        grid_layout_int.addWidget(self.proondeur_label, 4, 0, 1, 1)      

        group_box_int.setLayout(grid_layout_int)
        layout.addWidget(group_box_int)


        self.layout.addLayout(layout)


    # --- ROS callbacks ---
    def temp_callback(self, msg):
        self.signals.update_temp.emit(msg.data)

    def humidity_callback(self, msg):
        self.signals.update_humidity.emit(msg.data)

    def motor_callback(self, msg):
        for i in range(min(8, len(msg.data))):
            self.signals.update_motor.emit(i, msg.data[i])

    def set_temp(self, val):
        self.tempboit_label.setText(f"Temp: {val:.1f} °C")

    def set_humidity(self, val):
        self.humidity_label.setText(f"Hum: {val:.1f} %")

    def set_motor_value(self, index, value):
        self.motor_bars[index].setValue(int(value * 10))

    def move_window_right(self):
        # Utilise wmctrl pour déplacer la fenêtre PyQt à droite
        subprocess.call("wmctrl -r 'GUI Sous-Marin' -e 0,960,0,960,1080", shell=True)

    def run(self):
        sys.exit(self.app.exec_())


def main(args=None):
    # Lance RViz2 dans une autre fenêtre
    subprocess.Popen([
    "gnome-terminal", "--", "bash", "-c",
    "source /opt/ros/humble/setup.bash && rviz2 -d /home/xavier/Documents/GitHub/NautilusSW/UTILES/NAUTILUS_RVIZ.rviz"])

    # Essaie de déplacer RViz2 à gauche
    subprocess.call("wmctrl -r 'RViz' -e 0,0,0,960,1080", shell=True)

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
