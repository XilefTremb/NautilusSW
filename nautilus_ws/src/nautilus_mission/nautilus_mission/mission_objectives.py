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

    spin_pwm: int = 1500

    full_centering: bool = True
    center_tolerance_px: float = 50.0
    approach_distance: float = 5000.0
    angle_tolerance_deg: float = 15.0

    depth_threshold: Optional[int] = None

    action_type: ActionType = ActionType.NONE
    action_duration: float = 0.0 # currently not available as end condition for ActionType.FORWARD

    action_forward_pwm: int = 1500
    action_forward_distance: float = 0.0

    mean_depth_target: Optional[int] = None

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
            # Objective(
            #     name = 'gate',
            #     target_ids = [ObjectID.GATE_MID_RIGHT],
            #     spin_pwm = 1460, # Under 1500 is CCW, over 1500 is CW
            #     center_tolerance_px = 20.0,
            #     angle_tolerance_deg = 5.0,
            #     approach_distance = 1500.0,
            #     depth_threshold = 6000,
            #     action_type = ActionType.FORWARD,
            #     action_forward_pwm = 1550,
            #     action_forward_distance = 2.0,
            # ),
            # Objective(
            #     name = 'slalom',
            #     target_ids = [ObjectID.SLALOM_LEFT_MID],
            #     spin_pwm = 1540,
            #     center_tolerance_px = 20.0,
            #     angle_tolerance_deg = 5.0,
            #     approach_distance = 1000.0,
            #     depth_threshold = 5000,
            #     action_type = ActionType.FORWARD,
            #     action_forward_pwm = 1515,
            #     action_forward_distance = 1.0,
            # ),
            Objective(
                name='slalom2',
                target_ids=[ObjectID.SLALOM_LEFT_MID],
                spin_pwm = 1460,
                full_centering = False,
                center_tolerance_px=20.0,
                approach_distance=1000.0,
                depth_threshold=1500,
                action_type=ActionType.FORWARD,
                action_forward_pwm= 1540,
                action_forward_distance = 1.0,
            ),
            Objective(
                name='slalom3',
                target_ids=[ObjectID.SLALOM_LEFT_MID],
                spin_pwm = 1540,
                full_centering = False,
                center_tolerance_px=20.0,
                approach_distance=1000.0,
                depth_threshold=1500,
                action_type=ActionType.FORWARD,
                action_forward_pwm= 1540,
                action_forward_distance = 1.0,
            ),
        ]
