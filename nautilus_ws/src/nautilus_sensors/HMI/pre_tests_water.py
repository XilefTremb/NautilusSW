# Tests pré-mise à l'eau
#
# 1. Ping Jetson
# 2. Ping DVL
# 3. Data DVL
# 4. Data caméra avant
# 5. Data caméra arrière
# 6. Data IMU
# 7. Connexion Pixhawk

import sys
import subprocess
import time

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QFrame,
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt


# ============================================================
# CONFIGURATION
# ============================================================

JETSON_IP = "192.168.0.10"       # À MODIFIER
DVL_IP = "192.168.1.3"           # DVL IP
DVL_TOPIC = "/dvl/twist"           
FRONT_CAM_TOPIC = "/oakd/camera/image_raw"
BOTTOM_CAM_TOPIC = "/oak1/camera/image_raw"
IMU_TOPIC = "/imu/data"

PIXHAWK_TOPIC = "/mavros/state"   # À MODIFIER selon votre setup

# ============================================================
# THREAD QUI EXÉCUTE LES TESTS
# ============================================================

class TestWorker(QThread):

    test_started = pyqtSignal(int)
    test_finished = pyqtSignal(int, bool, str)
    all_finished = pyqtSignal()

    def run(self):

        tests = [
            self.test_ping_jetson,
            self.test_ping_dvl,
            self.test_dvl_data,
            self.test_front_camera,
            self.test_bottom_camera,
            self.test_imu,
            self.test_pixhawk,
        ]

        for index, test in enumerate(tests):

            self.test_started.emit(index)

            try:
                success, message = test()
            except Exception as e:
                success = False
                message = str(e)

            self.test_finished.emit(index, success, message)

            # Petite pause visuelle entre les tests
            time.sleep(0.3)

        self.all_finished.emit()

    def start_ros_node(self, executable):

        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "nautilus_sensors",
                executable
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        return process

    # --------------------------------------------------------
    # PING
    # --------------------------------------------------------

    def ping(self, ip):

        result = subprocess.run(
            ["ping", "-c", "1", "-W", "2", ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        return result.returncode == 0

    def test_ping_jetson(self):

        success = self.ping(JETSON_IP)

        if success:
            return True, f"Jetson détecté ({JETSON_IP})"

        return False, f"Aucune réponse du Jetson ({JETSON_IP})"

    def test_ping_dvl(self):

        success = self.ping(DVL_IP)

        if success:
            return True, f"DVL détecté ({DVL_IP})"

        return False, f"Aucune réponse du DVL ({DVL_IP})"

    # --------------------------------------------------------
    # ROS 2
    # --------------------------------------------------------

    def ros_topic_test(self, topic, timeout=10):

        start_time = time.time()

        while time.time() - start_time < timeout:

            try:
                result = subprocess.run(
                    [
                        "ros2",
                        "topic",
                        "echo",
                        topic,
                        "--once"
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=2
                )

                # Si un message ROS est reçu → succès immédiat
                if result.returncode == 0 and result.stdout.strip():
                    return True

            except subprocess.TimeoutExpired:
                # Pas de message, on continue d'essayer
                pass

            # Attend 0.5 s avant de réessayer
            time.sleep(0.5)

        # Le timeout global est atteint
        return False
    # --------------------------------------------------------
    # DVL
    # --------------------------------------------------------

    def test_dvl_data(self):

        self.dvl_process = self.start_ros_node(
            "dvl_sensor_node"
        )

        # Attend jusqu'à 10 secondes qu'un message arrive
        success = self.ros_topic_test(
            DVL_TOPIC,
            timeout=10
        )

        if success:
            return True, "DVL lancé - Données reçues"

        return False, "Aucune donnée DVL après 10 s"

    # --------------------------------------------------------
    # CAMÉRA AVANT
    # --------------------------------------------------------

    def test_front_camera(self):

        self.camera_process = self.start_ros_node(
            "stream_threaded"
        )

        success = self.ros_topic_test(
            FRONT_CAM_TOPIC,
            timeout=15
        )

        if success:
            return True, "Caméra avant - Image reçue"

        return False, "Aucune image après 15 s"

    # --------------------------------------------------------
    # CAMÉRA DESSOUS
    # --------------------------------------------------------

    def test_bottom_camera(self):

        success = self.ros_topic_test(
            BOTTOM_CAM_TOPIC,
            timeout=5
        )

        if success:
            return True, "Caméra dessous - Image reçue"

        return False, "Aucune image après 5 s"

    # --------------------------------------------------------
    # IMU
    # --------------------------------------------------------

    def test_imu(self):

        success = self.ros_topic_test(IMU_TOPIC)

        if success:
            return True, "Données IMU reçues"

        return False, "Aucune donnée IMU"

    # --------------------------------------------------------
    # PIXHAWK
    # --------------------------------------------------------

    def test_pixhawk(self):

        success = self.ros_topic_test(PIXHAWK_TOPIC)

        if success:
            return True, "Pixhawk connecté"

        return False, "Pixhawk non détecté"


# ============================================================
# INTERFACE
# ============================================================

class PreDiveHMI(QWidget):

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
            "Connexion Pixhawk",
        ]

        self.status_labels = []
        self.detail_labels = []

        self.create_ui()

    def create_ui(self):

        main_layout = QVBoxLayout()

        # ----------------------------------------------------
        # TITRE
        # ----------------------------------------------------

        title = QLabel("NAUTILUS")
        title.setAlignment(Qt.AlignCenter)

        title.setStyleSheet("""
            font-size: 30px;
            font-weight: bold;
            margin: 10px;
        """)

        subtitle = QLabel("Tests pré-mise à l'eau")
        subtitle.setAlignment(Qt.AlignCenter)

        subtitle.setStyleSheet("""
            font-size: 18px;
            color: #777;
            margin-bottom: 20px;
        """)

        main_layout.addWidget(title)
        main_layout.addWidget(subtitle)

        # ----------------------------------------------------
        # TESTS
        # ----------------------------------------------------

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

            # Nom
            name_label = QLabel(name)

            name_label.setStyleSheet("""
                font-size: 17px;
                font-weight: bold;
                border: none;
            """)

            # Détail
            detail = QLabel("En attente")

            detail.setAlignment(Qt.AlignRight)

            detail.setStyleSheet("""
                color: #888;
                border: none;
            """)

            # Voyant
            status = QLabel()

            status.setFixedSize(28, 28)

            self.set_status(status, "waiting")

            row.addWidget(name_label)
            row.addStretch()
            row.addWidget(detail)
            row.addWidget(status)

            self.status_labels.append(status)
            self.detail_labels.append(detail)

            main_layout.addWidget(frame)

        # ----------------------------------------------------
        # BOUTON
        # ----------------------------------------------------

        self.start_button = QPushButton("LANCER LES TESTS")

        self.start_button.setMinimumHeight(60)

        self.start_button.setStyleSheet("""
            QPushButton {
                font-size: 18px;
                font-weight: bold;
                border-radius: 10px;
                background-color: #1976D2;
                color: white;
            }

            QPushButton:hover {
                background-color: #1565C0;
            }

            QPushButton:disabled {
                background-color: #555;
            }
        """)

        self.start_button.clicked.connect(self.start_tests)

        main_layout.addSpacing(15)
        main_layout.addWidget(self.start_button)

        # ----------------------------------------------------
        # STATUS GLOBAL
        # ----------------------------------------------------

        self.global_status = QLabel("En attente")

        self.global_status.setAlignment(Qt.AlignCenter)

        self.global_status.setStyleSheet("""
            font-size: 18px;
            font-weight: bold;
            margin-top: 10px;
        """)

        main_layout.addWidget(self.global_status)

        self.setLayout(main_layout)

    # ========================================================
    # COULEURS DES VOYANTS
    # ========================================================

    def set_status(self, label, state):

        colors = {
            "waiting": "#666666",
            "running": "#FFC107",
            "success": "#00C853",
            "failed": "#D50000",
        }

        color = colors[state]

        label.setStyleSheet(f"""
            background-color: {color};
            border-radius: 14px;
            border: 2px solid #333;
        """)

    # ========================================================
    # LANCEMENT
    # ========================================================

    def start_tests(self):

        self.start_button.setEnabled(False)

        self.global_status.setText("Tests en cours...")

        # Reset
        for i in range(len(self.test_names)):

            self.set_status(
                self.status_labels[i],
                "waiting"
            )

            self.detail_labels[i].setText("En attente")

        # Thread
        self.worker = TestWorker()

        self.worker.test_started.connect(
            self.test_started
        )

        self.worker.test_finished.connect(
            self.test_finished
        )

        self.worker.all_finished.connect(
            self.tests_finished
        )

        self.worker.start()

    # ========================================================
    # SIGNALS
    # ========================================================

    def test_started(self, index):

        self.set_status(
            self.status_labels[index],
            "running"
        )

        self.detail_labels[index].setText(
            "Test en cours..."
        )

    def test_finished(self, index, success, message):

        if success:

            self.set_status(
                self.status_labels[index],
                "success"
            )

        else:

            self.set_status(
                self.status_labels[index],
                "failed"
            )

        self.detail_labels[index].setText(message)

    def tests_finished(self):

        self.start_button.setEnabled(True)

        failures = []

        for i, status in enumerate(self.status_labels):

            if "#D50000" in status.styleSheet():
                failures.append(self.test_names[i])

        if failures:

            self.global_status.setText(
                f"⚠ {len(failures)} TEST(S) EN ÉCHEC"
            )

            self.global_status.setStyleSheet("""
                color: #D50000;
                font-size: 20px;
                font-weight: bold;
            """)

        else:

            self.global_status.setText(
                "✓ SYSTÈME PRÊT POUR LA MISE À L'EAU"
            )

            self.global_status.setStyleSheet("""
                color: #00C853;
                font-size: 20px;
                font-weight: bold;
            """)


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    app = QApplication(sys.argv)

    window = PreDiveHMI()

    window.show()

    sys.exit(app.exec_())