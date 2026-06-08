#!/usr/bin/env python3

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Union

from enums.ObjectID import ObjectID

class ActionType(Enum):
    NONE = auto()
    FORWARD = auto()
    CIRCLE_MARKER = auto()

@dataclass
class SearchConfig:
    spin_pwm: int = 1500

@dataclass
class CenterConfig:
    full_centering: bool = True
    center_tolerance_fov: float = 0.05 # fraction of the fov. 1 being the full width of the camera 
    angle_tolerance_deg: float = 15.0

@dataclass
class ApproachConfig:
    approach_distance_mm: float = 5000.0
    
@dataclass
class NoAction:
    type: ActionType = ActionType.NONE

@dataclass
class ForwardAction:
    type: ActionType = ActionType.FORWARD
    duration_s: float = 0.0
    forward_pwm: int = 1500

@dataclass
class CircleMarkerAction:
    type: ActionType = ActionType.CIRCLE_MARKER
    camera_mean_depth_target_mm: int = 20000
    min_lifespan_s: float = 8.0

ActionConfig = Union[NoAction, ForwardAction, CircleMarkerAction]

@dataclass
class Objective:
    name: str
    target_ids: Optional[list[ObjectID]]
    detections_depth_filter_mm: Optional[int] = None

    search: SearchConfig = field(default_factory=SearchConfig)
    center: CenterConfig = field(default_factory=CenterConfig)
    approach: ApproachConfig = field(default_factory=ApproachConfig)
    action: ActionConfig = field(default_factory=NoAction)


mission_list = [
    Objective(
        name='gate',
        target_ids=[ObjectID.GATE_MID_RIGHT],
        search=SearchConfig(spin_pwm=1460),
        center=CenterConfig(
            center_tolerance_fov=20.0,
            angle_tolerance_deg=5.0,
        ),
        approach=ApproachConfig(
            approach_distance_mm=1500.0,
        ),
        action=ForwardAction(
            forward_pwm=1550,
            duration_s=3.0,
        ),
    ),

    Objective(
        name='slalom2',
        target_ids=[ObjectID.SLALOM_LEFT_MID],
        search=SearchConfig(spin_pwm=1460),
        center=CenterConfig(
            full_centering=False,
            center_tolerance_fov=20.0,
        ),
        approach=ApproachConfig(
            approach_distance_mm=1000.0
        ),
        action=ForwardAction(
            forward_pwm=1515,
            duration_s=1.0,
        ),
    ),
]
