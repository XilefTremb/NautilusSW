from collections import deque
import numpy as np
import time


class TemporalFilter:
    def __init__(self, node, maxlen=10, spike_threshold_mm=1000, reset_after_sec=1):

        self.node = node

        self.maxlen = maxlen
        self.spike_threshold_mm = spike_threshold_mm
        self.reset_after_sec = reset_after_sec

        self.history = {}
        self.last_seen = {}

    def moving_median_filter(self, key, new_value):

        if key not in self.history:
            self.history[key] = deque(maxlen=self.maxlen)
            self.last_seen[key] = None

        filtered_value = self.spike_filter_with_timeout(
            new_value,
            self.history[key],
            key
        )

        self.history[key].append(filtered_value)

        return float(np.median(self.history[key]))

    def spike_filter_with_timeout(self, new_value, history, key):

        now = time.monotonic()

        last_seen = self.last_seen[key]

        if last_seen is None or len(history) == 0:
            self.last_seen[key] = now
            return new_value

        time_since_seen = now - last_seen
        self.last_seen[key] = now

        if time_since_seen > self.reset_after_sec:
            return new_value

        last_value = history[-1]

        if abs(new_value - last_value) > self.spike_threshold_mm:
            return last_value

        return new_value