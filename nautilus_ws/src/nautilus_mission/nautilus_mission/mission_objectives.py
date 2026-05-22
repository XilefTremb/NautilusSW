#!/usr/bin/env python3

from enum import Enum, auto
from dataclasses import dataclass
from typing import Optional

from enums.ObjectID import ObjectID

class ActionType(Enum):
    NONE = auto()
    FORWARD = auto()
    CIRCLE_MARKER = auto()


@dataclass
class Objective:
    name: str
    target_ids: Optional[list[ObjectID]]

    center_tolerance_px: float = 50.0
    approach_distance: float = 5000.0
    angle_tolerance_deg: float = 15.0

    depth_threshold: Optional[int] = None

    action_type: ActionType = ActionType.NONE
    action_duration: float = 0.0
    action_forward_pwm: int = 1500

    mean_depth_target: Optional[int] = None

mission_list = [
            Objective(
                name='gate',
                target_ids=[ObjectID.GATE_MID_RIGHT],
                center_tolerance_px=40.0,
                angle_tolerance_deg=10.0,
                approach_distance=5000.0,
                depth_threshold=5000,
                action_type=ActionType.FORWARD,
                action_forward_pwm= 1550,
                action_duration=3.0,
            ),
            Objective(
                name='slalom',
                target_ids=[ObjectID.SLALOM_LEFT_MID],
                center_tolerance_px=20.0,
                angle_tolerance_deg=5.0,
                approach_distance=4000.0,
                depth_threshold=9000,
                action_type=ActionType.FORWARD,
                action_forward_pwm= 1550,
                action_duration=8.0,
            ),
        ]