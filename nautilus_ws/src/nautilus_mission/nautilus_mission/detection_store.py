#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from enums.DetectionIndex import DetectionIndex
from enums.ObjectID import ObjectID
from typing import Optional

class DetectionStore:
    """Owns YOLO detection parsing and target lookup."""

    def __init__(self, logger):
        self.logger = logger
        self.detections: list[list[float]] = []
        self.role_positions: Optional[list[ObjectID]] = None
        self.save_role = True

    def update_from_msg(self, msg: Float32MultiArray) -> bool:
        data = msg.data

        if len(data) % 5 != 0:
            self.logger.warn(
                f'Received malformed detection array of length {len(data)}. Expected multiple of 5.'
            )
            return False

        if self.save_role:
            self.role_positions = self.save_role_positions()
                
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

    def save_role_positions(self):

        fire = self.get_detection(ObjectID.FIRE)
        blood = self.get_detection(ObjectID.BLOOD)
        middle_post = self.get_detection(ObjectID.GATE_LEG_CENTER)

        # No object detected
        if fire is None and blood is None:
            return None

        # Both objects detected
        if fire is not None and blood is not None:
            if fire[DetectionIndex.CENTER_FOV_RATIO_X] < blood[DetectionIndex.CENTER_FOV_RATIO_X]:
                return [ObjectID.FIRE, ObjectID.BLOOD]

            return [ObjectID.BLOOD, ObjectID.FIRE]
        
        seen_id = None
        missing_id = None
        seen_x = None

        # Only one object detected
        if fire is not None:
            seen_id = ObjectID.FIRE
            missing_id = ObjectID.BLOOD
            seen_x = fire[DetectionIndex.CENTER_FOV_RATIO_X]
        else:
            seen_id = ObjectID.BLOOD
            missing_id = ObjectID.FIRE
            seen_x = blood[DetectionIndex.CENTER_FOV_RATIO_X]

        # Central post in gate is detected 
        if middle_post is not None:
            post_x = middle_post[DetectionIndex.CENTER_FOV_RATIO_X]

            if seen_x < post_x:
                return [seen_id, missing_id]

            return [missing_id, seen_id]

        # Fallback to image center
        if seen_x < 0.0:
            return [seen_id, missing_id]

        return [missing_id, seen_id]