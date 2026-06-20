#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from enums.DetectionIndex import DetectionIndex


class DetectionStore:
    """Owns YOLO detection parsing and target lookup."""

    def __init__(self, logger):
        self.logger = logger
        self.detections: list[list[float]] = []

    def update_from_msg(self, msg: Float32MultiArray) -> bool:
        data = msg.data

        if len(data) % 5 != 0:
            self.logger.warn(
                f'Received malformed detection array of length {len(data)}. Expected multiple of 5.'
            )
            return False

        self.detections = [data[i:i + 5] for i in range(0, len(data), 5)]
        return True

    def get_detection(self, ids):
        if ids is None:
            return None

        if isinstance(ids, int):
            ids = [ids]

        ids_as_int = [int(id_) for id_ in ids]

        return next(
            (
                detection for detection in self.detections
                if int(detection[DetectionIndex.ID]) in ids_as_int
            ),
            None,
        )
