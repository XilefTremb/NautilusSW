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
        middle_post = self.get_detection(ObjectID.GATE_LEG_CENTER)

        # Aucun objet détecté
        if requin is None and poisson is None:
            return None

        # Les deux objets sont détectés
        if requin is not None and poisson is not None:
            if requin[DetectionIndex.CENTER_FOV_RATIO] < poisson[DetectionIndex.CENTER_FOV_RATIO]:
                return [ObjectID.REQUIN, ObjectID.POISSON]

            return [ObjectID.POISSON, ObjectID.REQUIN]

        # Un seul objet est détecté
        if requin is not None:
            seen_id = ObjectID.REQUIN
            missing_id = ObjectID.POISSON
            seen_x = requin[DetectionIndex.CENTER_FOV_RATIO]
        else:
            seen_id = ObjectID.POISSON
            missing_id = ObjectID.REQUIN
            seen_x = poisson[DetectionIndex.CENTER_FOV_RATIO]

        # Poteau central détecté 
        if middle_post is not None:
            post_x = middle_post[DetectionIndex.CENTER_FOV_RATIO]

            if seen_x < post_x:
                return [seen_id, missing_id]

            return [missing_id, seen_id]

        # Fallback est centre de l'image
        if seen_x < 0.0:
            return [seen_id, missing_id]

        return [missing_id, seen_id]