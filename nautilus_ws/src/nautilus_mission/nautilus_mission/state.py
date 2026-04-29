import time

class State:
    def __init__(self, initial_state=None):
        self._state = initial_state
        self._last_updated_time = time.monotonic()

    def set(self, new_state):
        if new_state != self._state:
            self._state = new_state
            self._last_updated_time = time.monotonic()

    @property
    def lifespan(self):
        return time.monotonic() - self._last_updated_time

    def __eq__(self, other):
        return self._state == other

    def __getattr__(self, name):
        return getattr(self._state, name)