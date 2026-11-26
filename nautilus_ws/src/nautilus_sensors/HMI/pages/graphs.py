# pages/graphs.py

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QListWidget, QListWidgetItem, QComboBox
)
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg

from config import GRAPH_UPDATE_MS


DEFAULT_SIGNALS = [
    "Depth",
    "Battery Voltage", "Battery Current",
    "IMU Roll", "IMU Pitch", "IMU Yaw",
    "IMU Accel X", "IMU Accel Y", "IMU Accel Z",
    "DVL X", "DVL Y", "DVL Z"
]

for i in range(1, 9):
    DEFAULT_SIGNALS += [f"Motor {i} PWM", f"Motor {i} Current"]


class GraphsPage(QWidget):

    def __init__(self, data_manager):
        super().__init__()

        self.data = data_manager
        self.curves = {}

        main = QVBoxLayout(self)

        title = QLabel("NAUTILUS // DATA ANALYSIS")
        title.setObjectName("PageTitle")
        main.addWidget(title)

        # CONTROLS
        content = QHBoxLayout()
        controls = QVBoxLayout()

        signal_title = QLabel("Signals")
        signal_title.setObjectName("SectionTitle")
        controls.addWidget(signal_title)

        self.signal_list = QListWidget()

        for signal in DEFAULT_SIGNALS:
            item = QListWidgetItem(signal)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            self.signal_list.addItem(item)

        controls.addWidget(self.signal_list)
        controls.addWidget(QLabel("Time Window"))

        self.time_window = QComboBox()
        self.time_window.addItems(["5 s", "10 s", "30 s", "60 s", "120 s", "300 s"])
        self.time_window.setCurrentText("30 s")
        controls.addWidget(self.time_window)

        self.pause_button = QPushButton("PAUSE")
        self.pause_button.setCheckable(True)
        controls.addWidget(self.pause_button)

        clear_button = QPushButton("CLEAR GRAPH")
        clear_button.clicked.connect(self.clear_graph)
        controls.addWidget(clear_button)

        # GRAPH
        self.graph = pg.PlotWidget()
        self.graph.setBackground("#10141c")
        self.graph.showGrid(x=True, y=True, alpha=0.15)
        self.graph.setLabel("bottom", "Time", "s")
        self.graph.addLegend()

        content.addLayout(controls, 1)
        content.addWidget(self.graph, 4)
        main.addLayout(content)

        # TIMER
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graph)
        self.timer.start(GRAPH_UPDATE_MS)

    def update_graph(self):
        if self.pause_button.isChecked():
            self.pause_button.setText("RESUME")
            return

        self.pause_button.setText("PAUSE")
        seconds = int(self.time_window.currentText().split()[0])

        selected = [
            self.signal_list.item(i).text()
            for i in range(self.signal_list.count())
            if self.signal_list.item(i).checkState() == Qt.Checked
        ]

        # Remove unused curves
        for signal in list(self.curves):
            if signal not in selected:
                self.graph.removeItem(self.curves[signal])
                del self.curves[signal]

        # Draw selected data
        for index, signal in enumerate(selected):
            x, y = self.data.get_history(signal, seconds)

            if signal not in self.curves:
                pen = pg.mkPen(pg.intColor(index, max(len(selected), 1)), width=2)
                self.curves[signal] = self.graph.plot(x, y, pen=pen, name=signal)
            else:
                self.curves[signal].setData(x, y)

        self.graph.setXRange(-seconds, 0)

    def clear_graph(self):
        self.graph.clear()
        self.curves = {}
        self.graph.addLegend()