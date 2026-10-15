# pages/leak_test.py

import time

import pyqtgraph as pg
import matplotlib.pyplot as plt

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QFileDialog
)
from PyQt5.QtCore import QTimer


class LeakTestPage(QWidget):

    def __init__(self, data_manager):
        super().__init__()

        self.data = data_manager

        # ============================================================
        # SETTINGS
        # ============================================================

        self.pressure_1 = "Pressure 1"
        self.pressure_2 = "Pressure 2"

        # Maximum time displayed on screen
        self.window_seconds = 600.0

        # Window used to calculate pressure derivative
        self.derivative_window_seconds = 5.0

        # Start time of current leak test
        self.test_start_time = time.time()

        main = QVBoxLayout(self)

        # ============================================================
        # TITLE / CURRENT VALUES / BUTTONS
        # ============================================================

        title_layout = QHBoxLayout()

        title = QLabel("NAUTILUS // LEAK TEST")
        title.setObjectName("PageTitle")

        self.pressure_value_1 = QLabel("BAROMETER 1: --")
        self.pressure_value_1.setObjectName("SectionTitle")

        self.pressure_value_2 = QLabel("BAROMETER 2: --")
        self.pressure_value_2.setObjectName("SectionTitle")

        self.clear_button = QPushButton("CLEAR")
        self.clear_button.clicked.connect(self.clear_graphs)

        self.save_button = QPushButton("SAVE PDF")
        self.save_button.clicked.connect(self.save_pdf)

        title_layout.addWidget(title)
        title_layout.addStretch()
        title_layout.addWidget(self.pressure_value_1)
        title_layout.addWidget(self.pressure_value_2)
        title_layout.addWidget(self.clear_button)
        title_layout.addWidget(self.save_button)

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

    # ================================================================
    # GET TEST DATA
    # ================================================================

    def get_test_data(self, signal_name):

        raw_data = self.data.get_raw_history(signal_name)

        times = []
        values = []

        for timestamp, value in raw_data:

            # Ignore data from before the last CLEAR
            if timestamp < self.test_start_time:
                continue

            elapsed_time = timestamp - self.test_start_time

            times.append(elapsed_time)
            values.append(value)

        return times, values

    # ================================================================
    # DERIVATIVE
    # ================================================================

    def calculate_derivative(self, times, pressures):

        derivative_times = []
        derivatives = []

        for i in range(len(pressures)):

            # We need enough history to calculate a useful derivative
            if times[i] < self.derivative_window_seconds:
                continue

            j = i

            # Find the sample approximately 5 seconds earlier
            while (
                j > 0
                and times[i] - times[j]
                < self.derivative_window_seconds
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
    # GET DISPLAY WINDOW
    # ================================================================

    def get_display_data(self, times, values):

        if not times:
            return [], []

        current_time = times[-1]

        # Before 600 s -> display everything
        if current_time <= self.window_seconds:
            return times, values

        # After 600 s -> display only last 600 s
        minimum_time = current_time - self.window_seconds

        display_times = []
        display_values = []

        for t, value in zip(times, values):

            if t >= minimum_time:
                display_times.append(t)
                display_values.append(value)

        return display_times, display_values

    # ================================================================
    # UPDATE GRAPHS
    # ================================================================

    def update_graphs(self):

        # Get ALL data since last CLEAR
        times_1, pressures_1 = self.get_test_data(
            self.pressure_1
        )

        times_2, pressures_2 = self.get_test_data(
            self.pressure_2
        )

        # ============================================================
        # CURRENT PRESSURE VALUES
        # ============================================================

        if pressures_1:
            self.pressure_value_1.setText(
                f"BAROMETER 1: {pressures_1[-1]:.2f} kPa"
            )

        if pressures_2:
            self.pressure_value_2.setText(
                f"BAROMETER 2: {pressures_2[-1]:.2f} kPa"
            )

        # ============================================================
        # PRESSURE DISPLAY WINDOW
        # ============================================================

        display_times_1, display_pressures_1 = self.get_display_data(
            times_1,
            pressures_1
        )

        display_times_2, display_pressures_2 = self.get_display_data(
            times_2,
            pressures_2
        )

        self.pressure_curve_1.setData(
            display_times_1,
            display_pressures_1
        )

        self.pressure_curve_2.setData(
            display_times_2,
            display_pressures_2
        )

        # ============================================================
        # DERIVATIVES
        # ============================================================

        derivative_times_1, derivatives_1 = self.calculate_derivative(
            times_1,
            pressures_1
        )

        derivative_times_2, derivatives_2 = self.calculate_derivative(
            times_2,
            pressures_2
        )

        # ============================================================
        # DERIVATIVE DISPLAY WINDOW
        # ============================================================

        display_derivative_times_1, display_derivatives_1 = (
            self.get_display_data(
                derivative_times_1,
                derivatives_1
            )
        )

        display_derivative_times_2, display_derivatives_2 = (
            self.get_display_data(
                derivative_times_2,
                derivatives_2
            )
        )

        self.derivative_curve_1.setData(
            display_derivative_times_1,
            display_derivatives_1
        )

        self.derivative_curve_2.setData(
            display_derivative_times_2,
            display_derivatives_2
        )

    # ================================================================
    # CLEAR
    # ================================================================

    def clear_graphs(self):

        # New test starts now
        self.test_start_time = time.time()

        self.data.clear_history(self.pressure_1)
        self.data.clear_history(self.pressure_2)

        self.pressure_curve_1.clear()
        self.pressure_curve_2.clear()

        self.derivative_curve_1.clear()
        self.derivative_curve_2.clear()

        self.pressure_value_1.setText("BAROMETER 1: --")
        self.pressure_value_2.setText("BAROMETER 2: --")

        # Reset automatic scaling
        self.pressure_graph.enableAutoRange()
        self.derivative_graph.enableAutoRange()

    # ================================================================
    # SAVE PDF
    # ================================================================

    def save_pdf(self):

        # Get ALL data since last CLEAR
        times_1, pressures_1 = self.get_test_data(
            self.pressure_1
        )

        times_2, pressures_2 = self.get_test_data(
            self.pressure_2
        )

        if not pressures_1 and not pressures_2:
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Leak Test",
            "leak_test.pdf",
            "PDF Files (*.pdf)"
        )

        if not file_path:
            return

        if not file_path.lower().endswith(".pdf"):
            file_path += ".pdf"

        # ============================================================
        # DERIVATIVES
        # ============================================================

        derivative_times_1, derivatives_1 = self.calculate_derivative(
            times_1,
            pressures_1
        )

        derivative_times_2, derivatives_2 = self.calculate_derivative(
            times_2,
            pressures_2
        )

        # ============================================================
        # PDF X AXIS
        # ============================================================

        maximum_time = 0.0

        if times_1:
            maximum_time = max(maximum_time, times_1[-1])

        if times_2:
            maximum_time = max(maximum_time, times_2[-1])

        # PDF is always at least 600 seconds wide.
        # If the test is longer, show the entire test.
        pdf_max_time = max(
            self.window_seconds,
            maximum_time
        )

        # ============================================================
        # CREATE FIGURE
        # ============================================================

        fig, axes = plt.subplots(
            2,
            1,
            figsize=(11, 8.5)
        )

        # ============================================================
        # PRESSURE
        # ============================================================

        if pressures_1:
            axes[0].plot(
                times_1,
                pressures_1,
                label="Barometer 1"
            )

        if pressures_2:
            axes[0].plot(
                times_2,
                pressures_2,
                label="Barometer 2"
            )

        axes[0].set_title("Pressure")

        axes[0].set_ylabel(
            "Pressure [kPa]"
        )

        axes[0].set_xlim(
            0,
            pdf_max_time
        )

        axes[0].grid(True)
        axes[0].legend()

        # ============================================================
        # DERIVATIVE
        # ============================================================

        if derivatives_1:
            axes[1].plot(
                derivative_times_1,
                derivatives_1,
                label="Barometer 1"
            )

        if derivatives_2:
            axes[1].plot(
                derivative_times_2,
                derivatives_2,
                label="Barometer 2"
            )

        axes[1].set_title(
            f"Pressure Derivative - "
            f"{self.derivative_window_seconds:.0f} s Window"
        )

        axes[1].set_xlabel(
            "Time [s]"
        )

        axes[1].set_ylabel(
            "dP/dt [kPa/s]"
        )

        axes[1].set_xlim(
            0,
            pdf_max_time
        )

        axes[1].grid(True)
        axes[1].legend()

        # ============================================================
        # TITLE
        # ============================================================

        fig.suptitle(
            "NAUTILUS - Leak Test",
            fontsize=16,
            fontweight="bold"
        )

        fig.tight_layout()

        # ============================================================
        # SAVE
        # ============================================================

        fig.savefig(
            file_path,
            format="pdf",
            bbox_inches="tight"
        )

        plt.close(fig)