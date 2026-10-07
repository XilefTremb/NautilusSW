# widgets.py

from PyQt5.QtWidgets import (
    QFrame,
    QLabel,
    QVBoxLayout,
    QHBoxLayout,
)

from PyQt5.QtCore import Qt


class DataCard(QFrame):

    def __init__(
        self,
        title,
        unit="",
        decimals=2,
        parent=None
    ):
        super().__init__(parent)

        self.unit = unit
        self.decimals = decimals

        self.setObjectName("DataCard")

        layout = QVBoxLayout(self)

        self.title = QLabel(title)
        self.title.setObjectName("CardTitle")

        self.value = QLabel("--")
        self.value.setObjectName("CardValue")

        self.unit_label = QLabel(unit)
        self.unit_label.setObjectName("CardUnit")

        layout.addWidget(self.title)

        value_layout = QHBoxLayout()

        value_layout.addWidget(self.value)
        value_layout.addWidget(
            self.unit_label,
            alignment=Qt.AlignBottom
        )

        value_layout.addStretch()

        layout.addLayout(value_layout)

    def set_value(self, value):

        self.value.setText(
            f"{value:.{self.decimals}f}"
        )


class MotorCard(QFrame):

    def __init__(self, motor_number):

        super().__init__()

        self.motor_number = motor_number

        self.setObjectName("MotorCard")

        layout = QVBoxLayout(self)

        title = QLabel(
            f"MOTOR {motor_number}"
        )

        title.setObjectName("MotorTitle")

        self.pwm = QLabel("PWM: --")
        self.current = QLabel("Current: -- A")

        layout.addWidget(title)
        layout.addWidget(self.pwm)
        layout.addWidget(self.current)

    def set_pwm(self, value):

        self.pwm.setText(
            f"PWM: {value:.0f}"
        )

    def set_current(self, value):

        self.current.setText(
            f"Current: {value:.2f} A"
        )