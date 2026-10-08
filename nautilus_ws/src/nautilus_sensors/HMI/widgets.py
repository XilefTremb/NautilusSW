# widgets.py

from PyQt5.QtWidgets import (
    QWidget, QFrame, QLabel, QVBoxLayout, QHBoxLayout
)

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QPen


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
class PWMBar(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        self.center_pwm = 1500

        self.setMinimumHeight(35)

    def set_pwm(self, value):
        self.pwm = max(self.min_pwm, min(self.max_pwm, value))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        width = self.width()
        height = self.height()

        margin = 4
        bar_height = 16
        bar_y = (height - bar_height) // 2

        usable_width = width - 2 * margin
        center_x = width // 2

        # Fond
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor("#252d3a"))
        painter.drawRoundedRect(margin, bar_y, usable_width, bar_height, 5, 5)

        # PWM positif
        if self.pwm > self.center_pwm:
            ratio = (self.pwm - self.center_pwm) / (self.max_pwm - self.center_pwm)
            bar_width = int((usable_width / 2) * ratio)

            painter.setBrush(QColor("#00C853"))
            painter.drawRoundedRect(center_x, bar_y, bar_width, bar_height, 4, 4)

        # PWM négatif
        elif self.pwm < self.center_pwm:
            ratio = (self.center_pwm - self.pwm) / (self.center_pwm - self.min_pwm)
            bar_width = int((usable_width / 2) * ratio)

            painter.setBrush(QColor("#D50000"))
            painter.drawRoundedRect(center_x - bar_width, bar_y, bar_width, bar_height, 4, 4)

        # Ligne centrale = PWM 1500
        painter.setPen(QPen(QColor("#FFFFFF"), 2))
        painter.drawLine(center_x, bar_y - 3, center_x, bar_y + bar_height + 3)

class MotorCard(QFrame):

    def __init__(self, motor_number):
        super().__init__()

        self.motor_number = motor_number
        self.pwm_value = 1500

        self.setObjectName("MotorCard")

        layout = QVBoxLayout(self)

        # Titre
        title = QLabel(f"MOTOR {motor_number}")
        title.setObjectName("MotorTitle")
        layout.addWidget(title)

        # Valeur PWM
        self.pwm = QLabel("PWM: 1500")
        self.pwm.setAlignment(Qt.AlignCenter)
        self.pwm.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(self.pwm)

        # Barre PWM personnalisée
        self.pwm_bar = PWMBar()
        self.pwm_bar.setMinimumHeight(35)
        layout.addWidget(self.pwm_bar)

        # Courant
        self.current = QLabel("Current: -- A")
        self.current.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.current)

    def set_pwm(self, value):
        self.pwm_value = value
        self.pwm.setText(f"PWM: {value:.0f}")
        self.pwm_bar.set_pwm(value)

    def set_current(self, value):
        self.current.setText(f"Current: {value:.2f} A")