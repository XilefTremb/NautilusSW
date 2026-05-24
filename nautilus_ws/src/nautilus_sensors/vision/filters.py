from collections import deque
import numpy as np
import time


class TemporalFilter:
    def __init__(self, node, maxlen=10, spike_threshold_mm=1000, reset_after_sec=1):

        self.node = node

        self.maxlen = maxlen
        self.spike_threshold_mm = spike_threshold_mm
        self.reset_after_sec = reset_after_sec

        self.history_dict = {}
        self.last_seen_ms = {}

    def moving_median_filter(self, key, new_value):

        if key not in self.history_dict:
            self.history_dict[key] = deque(maxlen=self.maxlen)
            self.last_seen_ms[key] = None

        filtered_value = self.spike_filter_with_timeout(
            new_value,
            self.history_dict[key],
            key
        )

        self.history_dict[key].append(filtered_value)

        return float(np.median(self.history_dict[key]))

    def spike_filter_with_timeout(self, new_value, target_history, key):

        now = time.monotonic()

        target_last_seen_ms = self.last_seen_ms[key]

        if target_last_seen_ms is None or len(target_history) == 0:
            self.last_seen_ms[key] = now
            return new_value

        time_since_seen = now - target_last_seen_ms
        self.last_seen_ms[key] = now

        if time_since_seen > self.reset_after_sec:
            return new_value

        last_value = target_history[-1]

        if abs(new_value - last_value) > self.spike_threshold_mm:
            return last_value

        return new_value
