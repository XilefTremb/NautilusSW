# data_manager.py

import time
from collections import defaultdict, deque

from PyQt5.QtCore import QObject, pyqtSignal


class DataManager(QObject):

    data_updated = pyqtSignal(str, float)

    def __init__(self, history_seconds=60, max_frequency=100):
        super().__init__()

        max_samples = history_seconds * max_frequency

        self.values = {}
        self.history = defaultdict(
            lambda: deque(maxlen=max_samples)
        )

    def update(self, name, value):
        try:
            value = float(value)
        except (ValueError, TypeError):
            return

        timestamp = time.time()

        self.values[name] = value
        self.history[name].append(
            (timestamp, value)
        )

        self.data_updated.emit(name, value)

    def get(self, name, default=0.0):
        return self.values.get(name, default)

    def get_history(self, name, seconds=None):

        data = list(self.history[name])

        if not data:
            return [], []

        if seconds is not None:
            minimum_time = time.time() - seconds
            data = [
                point
                for point in data
                if point[0] >= minimum_time
            ]

        if not data:
            return [], []

        timestamps = [p[0] for p in data]
        values = [p[1] for p in data]

        # Temps relatif
        now = time.time()
        timestamps = [
            t - now
            for t in timestamps
        ]

        return timestamps, values

    def available_signals(self):
        return sorted(self.values.keys())