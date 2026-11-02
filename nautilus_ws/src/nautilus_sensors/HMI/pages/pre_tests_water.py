import sys
import subprocess
import time
import signal
import os

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFrame
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt


# =========================
# CONFIGURATION
# =========================

JETSON_IP = "192.168.0.10"
DVL_IP = "192.168.1.3"

DVL_TOPIC = "/dvl/twist"
FRONT_CAM_TOPIC = "/oakd/camera/image_raw"
BOTTOM_CAM_TOPIC = "/oak1/camera/image_raw"
IMU_TOPIC = "/imu"


# =========================
# TEST WORKER
# =========================

class TestWorker(QThread):
    test_started = pyqtSignal(int)
    test_finished = pyqtSignal(int, bool, str)
    all_finished = pyqtSignal()
    startup_status = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.mavproxy_process = None
        self.cube_process = None
        self.dvl_process = None
        self.camera_process = None

    # -------------------------
    # PROCESSUS
    # -------------------------

    def start_main_ros_nodes(self):
        self.startup_status.emit("Démarrage du MAVProxy Router...")
        self.mavproxy_process = subprocess.Popen(
            ["ros2", "launch", "nautilus_bringup", "mavproxy_router.launch.py"],
            stdout=None, stderr=None, preexec_fn=os.setsid
        )

        time.sleep(3)

        self.startup_status.emit("Démarrage de l'interface Cube...")
        self.cube_process = subprocess.Popen(
            ["ros2", "run", "nautilus_controls", "cube_interface", "--auv"],
            stdout=None, stderr=None, preexec_fn=os.setsid
        )

        time.sleep(3)
        self.startup_status.emit("Début des tests...")

    def start_ros_node(self, executable):
        return subprocess.Popen(
            ["ros2", "run", "nautilus_sensors", executable],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )

    def stop_process(self, process):
        if process is None or process.poll() is not None:
            return

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=3)

        except ProcessLookupError:
            pass

        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait(timeout=1)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass

    def stop_camera_node(self):
        self.stop_process(self.camera_process)
        self.camera_process = None

        subprocess.run(
            ["pkill", "-9", "-f", "stream_threaded"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL)

    def stop_main_ros_nodes(self):
        self.stop_process(self.cube_process)
        self.stop_process(self.mavproxy_process)

        self.cube_process = None
        self.mavproxy_process = None

        # Nettoyage de sécurité
        subprocess.run(
            ["pkill", "-9", "-f", "cube_interface"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        subprocess.run(
            ["pkill", "-9", "-f", "mavproxy"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
    # -------------------------
    # RUN
    # -------------------------

    def run(self):
        try:
            self.start_main_ros_nodes()
        except Exception as e:
            print(f"Erreur démarrage ROS : {e}")

        tests = [
            self.test_ping_jetson,
            self.test_ping_dvl,
            self.test_dvl_data,
            self.test_front_camera,
            self.test_bottom_camera,
            self.test_imu,
            self.test_pixhawk
        ]

        for i, test in enumerate(tests):
            self.test_started.emit(i)

            try:
                success, message = test()
            except Exception as e:
                success, message = False, str(e)

            self.test_finished.emit(i, success, message)
            time.sleep(0.3)

        self.startup_status.emit("Arrêt de MAVProxy et Cube Interface...")
        self.stop_main_ros_nodes()
        self.all_finished.emit()

    # -------------------------
    # OUTILS
    # -------------------------

    def ping(self, ip):
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "2", ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        return result.returncode == 0

    def ros_topic_test(self, topic, timeout=10):
        start = time.time()

        while time.time() - start < timeout:
            try:
                result = subprocess.run(
                    ["ros2", "topic", "echo", topic, "--once"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=2
                )

                if result.returncode == 0 and result.stdout.strip():
                    return True

            except subprocess.TimeoutExpired:
                pass

            time.sleep(0.5)

        return False

    # -------------------------
    # TESTS
    # -------------------------

    def test_ping_jetson(self):
        if self.ping(JETSON_IP):
            return True, f"Jetson détecté ({JETSON_IP})"
        return False, f"Aucune réponse du Jetson ({JETSON_IP})"

    def test_ping_dvl(self):
        if self.ping(DVL_IP):
            return True, f"DVL détecté ({DVL_IP})"
        return False, f"Aucune réponse du DVL ({DVL_IP})"

    def test_dvl_data(self):
        self.dvl_process = self.start_ros_node("dvl_sensor_node")

        try:
            success = self.ros_topic_test(DVL_TOPIC, timeout=60)

        finally:
            self.stop_process(self.dvl_process)
            self.dvl_process = None

            # S'assure qu'aucun dvl_sensor_node ne reste actif
            subprocess.run(
                ["pkill", "-9", "-f", "dvl_sensor_node"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

        if success:
            return True, "DVL lancé - Données reçues"

        return False, "Aucune donnée DVL après 60 s"

    def test_front_camera(self):
        self.camera_process = self.start_ros_node("stream_threaded")
        success = self.ros_topic_test(FRONT_CAM_TOPIC, timeout=15)

        if success:
            return True, "Caméra avant - Image reçue"

        return False, "Aucune image après 15 s"

    def test_bottom_camera(self):
        try:
            success = self.ros_topic_test(BOTTOM_CAM_TOPIC, timeout=5)
        finally:
            self.stop_camera_node()

        if success:
            return True, "Caméra dessous - Image reçue"
        return False, "Aucune image après 5 s"

    def test_imu(self):
        if self.ros_topic_test(IMU_TOPIC, timeout=10):
            return True, "Données IMU reçues"

        return False, "Aucune donnée IMU"

    def test_pixhawk(self):
        if self.mavproxy_process and self.mavproxy_process.poll() is None:
            return True, "Pixhawk connecté - MAVProxy actif"

        return False, "Pixhawk non détecté - MAVProxy non actif"


# =========================
# INTERFACE
# =========================

class PreDivePage(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Nautilus - Pré-mise à l'eau")
        self.resize(650, 600)

        self.test_names = [
            "Ping Jetson",
            "Ping DVL",
            "Data DVL",
            "Data caméra avant",
            "Data caméra dessous",
            "Data IMU",
            "Connexion Pixhawk"
        ]

        self.status_labels = []
        self.detail_labels = []
        self.worker = None

        self.create_ui()

    def create_ui(self):
        layout = QVBoxLayout()

        title = QLabel("NAUTILUS")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size:30px; font-weight:bold; margin:10px;"
        )

        subtitle = QLabel("Tests pré-mise à l'eau")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet(
            "font-size:18px; color:#777; margin-bottom:20px;"
        )

        layout.addWidget(title)
        layout.addWidget(subtitle)

        for name in self.test_names:
            frame = QFrame()
            frame.setStyleSheet("""
                QFrame {
                    border: 1px solid #555;
                    border-radius: 8px;
                    padding: 5px;
                }
            """)

            row = QHBoxLayout(frame)

            name_label = QLabel(name)
            name_label.setStyleSheet(
                "font-size:17px; font-weight:bold; border:none;"
            )

            detail = QLabel("En attente")
            detail.setAlignment(Qt.AlignRight)
            detail.setStyleSheet("color:#888; border:none;")

            status = QLabel()
            status.setFixedSize(28, 28)
            self.set_status(status, "waiting")

            row.addWidget(name_label)
            row.addStretch()
            row.addWidget(detail)
            row.addWidget(status)

            self.status_labels.append(status)
            self.detail_labels.append(detail)
            layout.addWidget(frame)

        self.start_button = QPushButton("LANCER LES TESTS")
        self.start_button.setMinimumHeight(60)
        self.start_button.setStyleSheet("""
            QPushButton {
                font-size:18px;
                font-weight:bold;
                border-radius:10px;
                background-color:#1976D2;
                color:white;
            }
            QPushButton:hover { background-color:#1565C0; }
            QPushButton:disabled { background-color:#555; }
        """)

        self.start_button.clicked.connect(self.start_tests)

        self.global_status = QLabel("En attente")
        self.global_status.setAlignment(Qt.AlignCenter)
        self.global_status.setStyleSheet(
            "font-size:18px; font-weight:bold; margin-top:10px;"
        )

        layout.addSpacing(15)
        layout.addWidget(self.start_button)
        layout.addWidget(self.global_status)

        self.setLayout(layout)

    # -------------------------
    # STATUS
    # -------------------------

    def set_status(self, label, state):
        colors = {
            "waiting": "#666666",
            "running": "#FFC107",
            "success": "#00C853",
            "failed": "#D50000"
        }

        label.setStyleSheet(
            f"background-color:{colors[state]};"
            "border-radius:14px;"
            "border:2px solid #333;"
        )

    # -------------------------
    # LANCEMENT
    # -------------------------

    def start_tests(self):
        self.start_button.setEnabled(False)
        self.global_status.setText("Démarrage du système...")
        self.global_status.setStyleSheet(
            "color:#FFC107; font-size:18px; font-weight:bold;"
        )

        for i in range(len(self.test_names)):
            self.set_status(self.status_labels[i], "waiting")
            self.detail_labels[i].setText("En attente")

        self.worker = TestWorker()
        self.worker.startup_status.connect(self.update_startup_status)
        self.worker.test_started.connect(self.test_started)
        self.worker.test_finished.connect(self.test_finished)
        self.worker.all_finished.connect(self.tests_finished)
        self.worker.start()

    def update_startup_status(self, message):
        self.global_status.setText(message)

    def test_started(self, index):
        self.global_status.setText("Tests en cours...")
        self.set_status(self.status_labels[index], "running")
        self.detail_labels[index].setText("Test en cours...")

    def test_finished(self, index, success, message):
        state = "success" if success else "failed"
        self.set_status(self.status_labels[index], state)
        self.detail_labels[index].setText(message)

    def tests_finished(self):
        self.start_button.setEnabled(True)

        failures = [
            self.test_names[i]
            for i, status in enumerate(self.status_labels)
            if "#D50000" in status.styleSheet()
        ]

        if failures:
            self.global_status.setText(
                f"⚠ {len(failures)} TEST(S) EN ÉCHEC"
            )
            self.global_status.setStyleSheet(
                "color:#D50000; font-size:20px; font-weight:bold;"
            )
        else:
            self.global_status.setText(
                "✓ SYSTÈME PRÊT POUR LA MISE À L'EAU"
            )
            self.global_status.setStyleSheet(
                "color:#00C853; font-size:20px; font-weight:bold;"
            )


# =========================
# MAIN
# =========================

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PreDivePage()
    window.show()
    sys.exit(app.exec_())
