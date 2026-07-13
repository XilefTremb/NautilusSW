from enum import IntEnum


class InferenceMode(IntEnum):
    FORWARD_ONLY = 0   # run only the forward camera pipeline
    DOWNWARD_ONLY = 1  # run only the downward camera pipeline
    BOTH = 2           # run both at their distributed frequencies (default)
