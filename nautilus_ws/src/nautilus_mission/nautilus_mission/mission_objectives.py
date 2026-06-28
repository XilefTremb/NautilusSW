#!/usr/bin/env python3

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Union

from enums.ObjectID import ObjectID

class ActionType(Enum):
    NONE = auto()
    FORWARD = auto()
    CIRCLE_MARKER = auto()
    SAVE_ROLE = auto()
    CHOOSE_GATE_SIDE = auto()
    LAUNCH_DROPPER = auto()
    FIRE_TORPEDO = auto()

@dataclass
class SearchConfig:
    spin_pwm: int = 1500

@dataclass
class CenterConfig:
    full_centering: bool = False
    center_tolerance_fov: float = 0.05 # fraction of the fov. 1 being the full width of the camera 
    angle_tolerance_deg: float = 15.0
    alignement_tolerance: float = 50.0 #mm for torpedo and degrees for gate or slalom
    center_bottom: bool = False
    target_offset_x: float = 0.2 # fraction of the fov. 1 being the full width of the camera
    target_offset_y: float = 0.1 # fraction of the fov. 

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
    forward_distance_m: float = 0.0

@dataclass
class CircleMarkerAction:
    type: ActionType = ActionType.CIRCLE_MARKER
    camera_mean_depth_target_mm: int = 20000
    min_lifespan_s: float = 8.0

@dataclass
class LaunchDropperAction:
    type: ActionType = ActionType.LAUNCH_DROPPER
    duration_s: float = 0.0
    fired: bool = False

@dataclass
class SaveRoleAction:
    type: ActionType = ActionType.SAVE_ROLE

@dataclass
class ChooseGateSideAction:
    type: ActionType = ActionType.CHOOSE_GATE_SIDE

@dataclass
class FireTorpedoAction:
    type: ActionType = ActionType.FIRE_TORPEDO
    min_lifespan_s: float = 1.0
    fired: bool = False

ActionConfig = Union[NoAction, ForwardAction, CircleMarkerAction, LaunchDropperAction, ChooseGateSideAction, SaveRoleAction,FireTorpedoAction]

@dataclass
class Objective:
    name: str
    target_ids: Optional[list[ObjectID]] = None
    detections_depth_filter_mm: Optional[int] = None
    target_auv_depth_m: Optional[float] = None #positive down

    search: SearchConfig = field(default_factory=SearchConfig)
    center: CenterConfig = field(default_factory=CenterConfig)
    approach: ApproachConfig = field(default_factory=ApproachConfig)
    action: ActionConfig = field(default_factory=NoAction)

approach_gate = Objective(
    name='approachGate',
    target_ids=[ObjectID.GATE_LEG_CENTER],
    target_auv_depth_m = 0.9, #1.45
    search=SearchConfig(spin_pwm=1540),
    center=CenterConfig(
        center_tolerance_fov=0.2,
    ),
    approach=ApproachConfig(
        approach_distance_mm=4000.0,
    ),
    action=SaveRoleAction(
    ),
)

choose_gate_side = Objective(
    name='chooseGateSide',
    search=SearchConfig(spin_pwm=1460),
    approach=ApproachConfig(
        approach_distance_mm=3000.0,
    ),
    center=CenterConfig(
        full_centering=True,
        center_tolerance_fov=0.05,
        angle_tolerance_deg=5.0,
    ),
    action=ChooseGateSideAction(
    ),
)

traverse_gate = Objective(
    name='traverseGate',
    search=SearchConfig(spin_pwm=1460),
    center=CenterConfig(
        full_centering=True,
        center_tolerance_fov=0.05,
        angle_tolerance_deg=5.0,
    ),
    approach=ApproachConfig(
        approach_distance_mm=2000.0,
    ),
    action=ForwardAction(
        forward_distance_m=4.0,
    ),
)

slalom1 = Objective(
    name='slalom1',
    target_ids=[ObjectID.SLALOM_LEFT_MID],
    search=SearchConfig(spin_pwm=1460),
    center=CenterConfig(
        full_centering=True,
        center_tolerance_fov=0.05,
        angle_tolerance_deg=5.0,
    ),
    approach=ApproachConfig(
        approach_distance_mm=2500.0
    ),
    action=ForwardAction(
        forward_distance_m=2.5,
    ),
)

slalom2 = Objective(
    name='slalom2',
    target_ids=[ObjectID.SLALOM_LEFT_MID],
    search=SearchConfig(spin_pwm=1540),
    center=CenterConfig(
        center_tolerance_fov=0.05,
    ),
    approach=ApproachConfig(
        approach_distance_mm=1500.0
    ),
    action=ForwardAction(
        forward_distance_m=1.3,
    ),
)

slalom3 = Objective(
    name='slalom3',
    target_ids=[ObjectID.SLALOM_LEFT_MID],
    search=SearchConfig(spin_pwm=1460),
    center=CenterConfig(
        center_tolerance_fov=0.05,
    ),
    approach=ApproachConfig(
        approach_distance_mm=1500.0
    ),
    action=ForwardAction(
        forward_distance_m=3.0,
    ),
)

approach_dropper = Objective(
    name='approachDropperObjective',
    target_ids=[ObjectID.DROPPER],
    search=SearchConfig(spin_pwm=1460),
    center=CenterConfig(
        full_centering=False,
    ),
    approach=ApproachConfig(
        approach_distance_mm=2000.0
    ),
    action=ForwardAction(
        forward_distance_m=2.5,
    ),
)

launch_dropper = Objective(
    name='launchDropperObjective',
    search=SearchConfig(spin_pwm=1460),
    center=CenterConfig(
        full_centering=False,
        center_tolerance_fov=0.05,
        center_bottom=True,
        target_offset_x=-0.25,
        target_offset_y=0.25,
    ),
    action=LaunchDropperAction(
        duration_s=2.0,
    ),
)

coarse_approach_torpedo = Objective(
         name='coarseApproachTorpedo',
         target_ids=[ObjectID.TORPEDO],
         search=SearchConfig(spin_pwm=1460),
         center=CenterConfig(
             full_centering=False,
             center_tolerance_fov=0.2,
         ),
         approach=ApproachConfig(
             approach_distance_mm=6000.0,
         ),
     )
     
torpedo_depth_change = Objective(
         name='torpedoDepthChange',
         target_ids=None,
         target_auv_depth_m=1.5,
         action=NoAction()
     )

fine_approach_torpedo = Objective(
         name='fineApproachTorpedo',
         target_ids=[ObjectID.TORPEDO],
         center=CenterConfig(
             full_centering=True,
             center_tolerance_fov=0.1,
             alignement_tolerance=150.0,
         ),
         approach=ApproachConfig(
             approach_distance_mm=4500.0,
         ),
     )
    
torpedo_firing_positioning = Objective(
        name='torpedoFiringPositioning',
        target_ids=[ObjectID.TARGET_FIRE],
        # target_ids=None,
        center=CenterConfig(
            full_centering=True,
            center_tolerance_fov=0.02,
            alignement_tolerance=20.0,
        ),
        approach=ApproachConfig(
             approach_distance_mm=1000.0,
        ),
        action=FireTorpedoAction(
            min_lifespan_s=1.0,
        ),
    )

#mission_list = [approach_gate, choose_gate_side, traverse_gate, slalom1, slalom2, slalom3]
mission_list = [approach_gate, choose_gate_side, traverse_gate]
# mission_list = [approach_gate, choose_gate_side, traverse_gate, slalom1, slalom2, slalom3, approach_dropper, launch_dropper]
