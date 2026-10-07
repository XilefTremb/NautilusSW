# main_window.py

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QPushButton, QStackedWidget, QLabel
)

from PyQt5.QtCore import Qt

from pages.dashboard import DashboardPage
from pages.graphs import GraphsPage
from pages.pre_tests_water import PreDivePage


class MainWindow(QMainWindow):

    def __init__(self, data_manager):
        super().__init__()

        self.data = data_manager
        self.sidebar_open = True

        self.setWindowTitle("NAUTILUS Control Interface")
        self.resize(1500, 900)

        central = QWidget()
        self.setCentralWidget(central)

        main = QHBoxLayout(central)
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(0)

        # ==================================
        # SIDEBAR
        # ==================================

        self.sidebar = QWidget()
        self.sidebar.setObjectName("Sidebar")
        self.sidebar.setFixedWidth(220)

        side = QVBoxLayout(self.sidebar)

        logo = QLabel("NAUTILUS")
        logo.setObjectName("Logo")
        side.addWidget(logo)

        self.toggle_button = QPushButton("☰   MENU")
        self.toggle_button.clicked.connect(self.toggle_sidebar)
        side.addWidget(self.toggle_button)

        self.dashboard_button = QPushButton("◉   Dashboard")
        self.graph_button = QPushButton("⌁   Graphs")
        self.pre_dive_button = QPushButton("✓   Pre-Dive")

        side.addWidget(self.dashboard_button)
        side.addWidget(self.graph_button)
        side.addWidget(self.pre_dive_button)

        side.addStretch()

        version = QLabel("NAUTILUS HMI\nv1.0")
        version.setObjectName("Version")
        side.addWidget(version)

        main.addWidget(self.sidebar)

        # ==================================
        # PAGES
        # ==================================

        self.pages = QStackedWidget()

        self.dashboard = DashboardPage(data_manager)
        self.graphs = GraphsPage(data_manager)
        self.pre_dive = PreDivePage()

        self.pages.addWidget(self.dashboard)
        self.pages.addWidget(self.graphs)
        self.pages.addWidget(self.pre_dive)

        main.addWidget(self.pages)

        self.dashboard_button.clicked.connect(
            lambda: self.pages.setCurrentWidget(self.dashboard)
        )

        self.graph_button.clicked.connect(
            lambda: self.pages.setCurrentWidget(self.graphs)
        )

        self.pre_dive_button.clicked.connect(
            lambda: self.pages.setCurrentWidget(self.pre_dive)
        )

        self.apply_style()

    # ==========================================================
    # SIDEBAR
    # ==========================================================

    def toggle_sidebar(self):
        self.sidebar_open = not self.sidebar_open

        if self.sidebar_open:
            self.sidebar.setFixedWidth(220)
            self.toggle_button.setText("☰   MENU")
            self.dashboard_button.setText("◉   Dashboard")
            self.graph_button.setText("⌁   Graphs")
            self.pre_dive_button.setText("✓   Pre-Dive")

        else:
            self.sidebar.setFixedWidth(70)
            self.toggle_button.setText("☰")
            self.dashboard_button.setText("◉")
            self.graph_button.setText("⌁")
            self.pre_dive_button.setText("✓")

    # ==========================================================
    # STYLE
    # ==========================================================

    def apply_style(self):
        self.setStyleSheet("""
        QMainWindow {
            background: #0b0e14;
        }

        QWidget {
            background: #0b0e14;
            color: #dce6f2;
            font-family: Arial;
            font-size: 14px;
        }

        #Sidebar {
            background: #111722;
            border-right: 1px solid #253044;
        }

        #Logo {
            font-size: 24px;
            font-weight: bold;
            color: #4fc3f7;
            padding: 20px 10px;
        }

        QPushButton {
            background: transparent;
            border: none;
            padding: 14px;
            text-align: left;
            border-radius: 6px;
        }

        QPushButton:hover {
            background: #1b2636;
        }

        QPushButton:checked {
            background: #25354a;
        }

        #PageTitle {
            font-size: 24px;
            font-weight: bold;
            padding: 15px;
        }

        #SectionTitle {
            font-size: 16px;
            font-weight: bold;
            color: #7dd3fc;
            padding-top: 10px;
        }

        #DataCard {
            background: #121925;
            border: 1px solid #253044;
            border-radius: 10px;
            padding: 10px;
        }

        #DataCard:hover {
            border: 1px solid #4fc3f7;
        }

        #CardTitle {
            color: #8391a7;
            font-size: 12px;
            font-weight: bold;
        }

        #CardValue {
            font-size: 28px;
            font-weight: bold;
            color: white;
        }

        #CardUnit {
            color: #8391a7;
        }

        #MotorCard {
            background: #121925;
            border: 1px solid #253044;
            border-radius: 8px;
            padding: 8px;
        }

        #MotorTitle {
            color: #4fc3f7;
            font-weight: bold;
        }

        #Version {
            color: #526174;
            padding: 10px;
        }

        QListWidget {
            background: #121925;
            border: 1px solid #253044;
            border-radius: 6px;
        }

        QComboBox {
            background: #121925;
            border: 1px solid #253044;
            padding: 8px;
            border-radius: 5px;
        }
        """)
