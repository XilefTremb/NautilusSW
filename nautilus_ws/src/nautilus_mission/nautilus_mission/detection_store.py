#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from enums.DetectionIndex import DetectionIndex
from enums.ObjectID import ObjectID

class DetectionStore:
    """Owns YOLO detection parsing and target lookup."""

    def __init__(self, logger):
        self.logger = logger
        self.detections: list[list[float]] = []
        self.role_choice: Optional[list[ObjectID]] = None
        self.save_role = True

    def update_from_msg(self, msg: Float32MultiArray) -> bool:
        data = msg.data

        if len(data) % 4 != 0:
            self.logger.warn(
                f'Received malformed detection array of length {len(data)}. Expected multiple of 4.'
            )
            return False

        self.detections = [data[i:i + 4] for i in range(0, len(data), 4)]

        if self.save_role:
            self.role_choice = self.save_role_choice()
                
        return True
    
    def get_role_choice(self):
        return self.role_choice

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

    def save_role_choice(self):
        requin = self.get_detection(ObjectID.REQUIN)
        poisson = self.get_detection(ObjectID.POISSON)
        middle_post = self.get_detection(ObjectID.POTEAU_MILIEU)

        if requin is None or poisson is None:
            return None

        if middle_post is not None:
            post_x = middle_post[DetectionIndex.CENTER_FOV_RATIO]

            requin_left = requin[DetectionIndex.CENTER_FOV_RATIO] < post_x
            poisson_left = poisson[DetectionIndex.CENTER_FOV_RATIO] < post_x

            if requin_left and not poisson_left:
                return [ObjectID.REQUIN, ObjectID.POISSON]

            if poisson_left and not requin_left:
                return [ObjectID.POISSON, ObjectID.REQUIN]

        
        if requin[DetectionIndex.CENTER_FOV_RATIO] < poisson[DetectionIndex.CENTER_FOV_RATIO]:
            return [ObjectID.REQUIN, ObjectID.POISSON]

        return [ObjectID.POISSON, ObjectID.REQUIN]