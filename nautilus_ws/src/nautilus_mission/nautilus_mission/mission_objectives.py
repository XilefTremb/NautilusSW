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
    min_action_lifespan: float = 0.0

# mission_list = [
#             Objective(
#                 name='marker',
#                 target_ids=[ObjectID.GATE_LEG_L],
#                 center_tolerance_px=50.0,
#                 approach_distance=5000.0,
#                 depth_threshold=15000,
#                 action_type=ActionType.CIRCLE_MARKER,
#                 mean_depth_target=20000,
#                 min_action_lifespan=8.0,
#             ),
#             Objective(
#                 name='marker_blind',
#                 target_ids=None,
#                 action_type=ActionType.FORWARD,
#                 min_action_lifespan=3.0,
#             ),
#             Objective(
#                 name='return_gate_area',
#                 target_ids=[ObjectID.GATE_LEFT_MID, ObjectID.REQUIN, ObjectID.POISSON],
#                 center_tolerance_px=80.0,
#                 approach_distance=6000.0,
#                 action_type=ActionType.NONE,
#             ),
#             Objective(
#                 name='return_home',
#                 target_ids=[ObjectID.GATE_LEFT_MID],
#                 center_tolerance_px=50.0,
#                 approach_distance=2000.0,
#                 action_type=ActionType.FORWARD,
#                 action_duration=3.0,
#                 action_forward_pwm=1700,
#             ),
#         ]

mission_list = [
    Objective(
                name='center_gate',
                target_ids=[ObjectID.GATE_MID_RIGHT],
                center_tolerance_px=0.0,
                angle_tolerance_deg=0.0,
                approach_distance=5000.0,
                depth_threshold=15000,
                action_type=ActionType.NONE,
            ),
]