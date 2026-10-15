# pages/leak_test.py

import pyqtgraph as pg

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel
)
from PyQt5.QtCore import QTimer


class LeakTestPage(QWidget):

    def __init__(self, data_manager):
        super().__init__()

        self.data = data_manager

        # DATA MANAGER SIGNAL NAMES
        self.pressure_1 = "Pressure 1"
        self.pressure_2 = "Pressure 2"

        # Display last 5 minutes
        self.window_seconds = 300

        main = QVBoxLayout(self)

        # ============================================================
        # TITLE
        # ============================================================

        title_layout = QHBoxLayout()

        title = QLabel("NAUTILUS // LEAK TEST")
        title.setObjectName("PageTitle")

        self.pressure_value_1 = QLabel("BAROMETER 1: --")
        self.pressure_value_1.setObjectName("SectionTitle")

        self.pressure_value_2 = QLabel("BAROMETER 2: --")
        self.pressure_value_2.setObjectName("SectionTitle")

        title_layout.addWidget(title)
        title_layout.addStretch()
        title_layout.addWidget(self.pressure_value_1)
        title_layout.addWidget(self.pressure_value_2)

        main.addLayout(title_layout)

        # ============================================================
        # PRESSURE GRAPH
        # ============================================================

        pressure_title = QLabel("PRESSURE")
        pressure_title.setObjectName("SectionTitle")
        main.addWidget(pressure_title)

        self.pressure_graph = pg.PlotWidget()

        self.pressure_graph.setLabel(
            "left",
            "Pressure",
            units="kPa"
        )

        self.pressure_graph.setLabel(
            "bottom",
            "Time",
            units="s"
        )

        self.pressure_graph.showGrid(
            x=True,
            y=True,
            alpha=0.3
        )

        self.pressure_graph.addLegend()

        self.pressure_curve_1 = self.pressure_graph.plot(
            name="Barometer 1",
            pen=pg.mkPen("#4fc3f7", width=2)
        )

        self.pressure_curve_2 = self.pressure_graph.plot(
            name="Barometer 2",
            pen=pg.mkPen("#ff9800", width=2)
        )

        main.addWidget(self.pressure_graph)

        # ============================================================
        # PRESSURE DERIVATIVE GRAPH
        # ============================================================

        derivative_title = QLabel("PRESSURE DERIVATIVE")
        derivative_title.setObjectName("SectionTitle")
        main.addWidget(derivative_title)

        self.derivative_graph = pg.PlotWidget()

        self.derivative_graph.setLabel(
            "left",
            "dP/dt",
            units="kPa/s"
        )

        self.derivative_graph.setLabel(
            "bottom",
            "Time",
            units="s"
        )

        self.derivative_graph.showGrid(
            x=True,
            y=True,
            alpha=0.3
        )

        self.derivative_graph.addLegend()

        self.derivative_curve_1 = self.derivative_graph.plot(
            name="Barometer 1",
            pen=pg.mkPen("#4fc3f7", width=2)
        )

        self.derivative_curve_2 = self.derivative_graph.plot(
            name="Barometer 2",
            pen=pg.mkPen("#ff9800", width=2)
        )

        main.addWidget(self.derivative_graph)

        # ============================================================
        # REFRESH TIMER
        # ============================================================

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_graphs)

        # Refresh display every 100 ms
        self.timer.start(100)

        self.derivative_window_seconds = 5.0

    # ================================================================
    # DERIVATIVE
    # ================================================================

    def calculate_derivative(self, times, pressures):

        derivative_times = []
        derivatives = []

        for i in range(len(pressures)):

            # Find the oldest point inside the derivative window
            j = i

            while (
                j > 0 and
                times[i] - times[j] < self.derivative_window_seconds
            ):
                j -= 1

            dt = times[i] - times[j]

            if dt <= 0:
                continue

            dp = pressures[i] - pressures[j]

            derivative_times.append(times[i])
            derivatives.append(dp / dt)

        return derivative_times, derivatives
    # ================================================================
    # UPDATE GRAPHS
    # ================================================================

    def update_graphs(self):

        # Get Barometer 1 history
        times_1, pressures_1 = self.data.get_history(
            self.pressure_1,
            seconds=self.window_seconds
        )

        # Get Barometer 2 history
        times_2, pressures_2 = self.data.get_history(
            self.pressure_2,
            seconds=self.window_seconds
        )

        # ============================================================
        # PRESSURE
        # ============================================================

        if pressures_1:

            self.pressure_curve_1.setData(
                times_1,
                pressures_1
            )

            self.pressure_value_1.setText(
                f"BAROMETER 1: {pressures_1[-1]:.2f} kPa"
            )

        if pressures_2:

            self.pressure_curve_2.setData(
                times_2,
                pressures_2
            )

            self.pressure_value_2.setText(
                f"BAROMETER 2: {pressures_2[-1]:.2f} kPa"
            )

            



        # ============================================================
        # DERIVATIVE - BAROMETER 1
        # ============================================================

        if len(pressures_1) >= 2:

            derivative_times_1, derivatives_1 = self.calculate_derivative(
                times_1,
                pressures_1
            )

            self.derivative_curve_1.setData(
                derivative_times_1,
                derivatives_1
            )

        # ============================================================
        # DERIVATIVE - BAROMETER 2
        # ============================================================

        if len(pressures_2) >= 2:

            derivative_times_2, derivatives_2 = self.calculate_derivative(
                times_2,
                pressures_2
            )

            self.derivative_curve_2.setData(
                derivative_times_2,
                derivatives_2
            )