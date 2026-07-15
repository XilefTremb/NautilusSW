#!/usr/bin/env python3

from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Union

from enums.ObjectID import ObjectID
from enums.InferenceMode import InferenceMode

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
    spin_pwm: int = 1540

@dataclass
class CenterConfig:
    full_centering: bool = False
    x_center_tolerance_fov: float = 0.05 # fraction of the fov. 1 being the full width of the camera 
    y_center_tolerance_fov: float = 1.0
    angle_tolerance_deg: float = 15.0

    center_bottom: bool = False
    target_offset_x: float = 0.2 # fraction of the fov. 1 being the full width of the camera
    target_offset_y: float = 0.1 # fraction of the fov. 

    align_width: bool = False
    target_width_px: int = 100 #use width of bbox to estiamte perpendicularness with torpedo
    width_tolerance_px: int = 10

@dataclass
class ApproachConfig:
    approach_distance_mm: Optional[float] = None

@dataclass
class NoAction:
    type: ActionType = ActionType.NONE

@dataclass
class ForwardAction:
    type: ActionType = ActionType.FORWARD
    duration_s: float = 0.0
    forward_distance_m: Optional[float] = None
    lateral_distance_m: Optional[float] = None
    dropper_search: bool = False

@dataclass
class CircleMarkerAction:
    type: ActionType = ActionType.CIRCLE_MARKER
    camera_mean_depth_target_mm: int = 20000
    min_lifespan_s: float = 8.0

@dataclass
class LaunchDropperAction:
    type: ActionType = ActionType.LAUNCH_DROPPER
    launch_second_dropper: bool = False
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
    success_frame_treshold: int = 1
    # Inference mode requested while this objective is active (see InferenceMode).
    inference_mode: InferenceMode = InferenceMode.FORWARD_ONLY

approach_gate = Objective(
    name='approachGate',
    target_ids=[ObjectID.GATE_LEG_CENTER],
    inference_mode=InferenceMode.FORWARD_ONLY,
    target_auv_depth_m = 0.7,
    success_frame_treshold = 10,
    search=SearchConfig(spin_pwm=1440),
    detections_depth_filter_mm=6000,
    center=CenterConfig(
        full_centering = False,
        x_center_tolerance_fov=0.3,
        angle_tolerance_deg=5.0,
    ),
    approach=ApproachConfig(
        approach_distance_mm=4000.0,
    ),
    action=SaveRoleAction(
    ),
)

choose_gate_side = Objective(
    name='chooseGateSide',
    success_frame_treshold=10,
    search=SearchConfig(spin_pwm=1540),
    approach=ApproachConfig(
        approach_distance_mm=3000.0,
    ),
    center=CenterConfig(
        full_centering = False,
        x_center_tolerance_fov=0.05,
        angle_tolerance_deg=5.0,
        
    ),
    action=ChooseGateSideAction(
    ),
)

traverse_gate = Objective(
    name='traverseGate',
    success_frame_treshold=10,
    search=SearchConfig(spin_pwm=1460),
    center=CenterConfig(
        full_centering = False,
        x_center_tolerance_fov=0.05,
        angle_tolerance_deg=5.0,
    ),
    approach=ApproachConfig(
        approach_distance_mm=1500.0,
    ),
    action=ForwardAction(
        forward_distance_m=2.7,
    ),
)

slalom1 = Objective(
    name='slalom1',
    # target_ids=[ObjectID.SLALOM_LEFT_MID],
    inference_mode=InferenceMode.FORWARD_ONLY,
    success_frame_treshold=10,
    detections_depth_filter_mm = 4000,
    search=SearchConfig(spin_pwm=1560),
    target_auv_depth_m = 0.9,
    center=CenterConfig(
        full_centering = False,
        x_center_tolerance_fov=0.05,
        y_center_tolerance_fov=1.0,
        angle_tolerance_deg=5.0,
    ),
    approach=ApproachConfig(
        approach_distance_mm=2000.0
    ),
    action=ForwardAction(
        forward_distance_m=3.0,
    ),
)

slalom2 = Objective(
    name='slalo2',
    detections_depth_filter_mm = 30000,
    action=ForwardAction(
        forward_distance_m=2.0,
        lateral_distance_m = -1.0
    ),
)

slalom3 = Objective(
    name='slalo3',
    detections_depth_filter_mm = 30000,
    action=ForwardAction(
        forward_distance_m=2.0,
        lateral_distance_m=1.0,
    ),
)

lateral_after_gate = Objective(
    name='lateral_after_gate',
    detections_depth_filter_mm = 30000,
    action=ForwardAction(
        lateral_distance_m = -1.5
    ),
)

approach_dropper = Objective(
    name='approachDropperObjective',
    target_ids=[ObjectID.DROPPER],
    inference_mode=InferenceMode.BOTH,
    detections_depth_filter_mm=30000,
    target_auv_depth_m = 1.3,
    success_frame_treshold= 10,
    search=SearchConfig(spin_pwm=1440),
    center=CenterConfig(
        full_centering=False,
        x_center_tolerance_fov=0.05
    ),
    approach=ApproachConfig(
        approach_distance_mm=2000.0
    ),
    action=NoAction()
)

set_first_depth_for_dropper = Objective(
    name='setDepthForDropper',
    target_ids=None,
    inference_mode=InferenceMode.DOWNWARD_ONLY,
    target_auv_depth_m = 0.7,
    action=NoAction(),
)

set_second_depth_for_dropper = Objective(
    name='setDepthForDropper',
    target_ids=None,
    inference_mode=InferenceMode.DOWNWARD_ONLY,
    target_auv_depth_m = 0.5,
    action=ForwardAction(
        forward_distance_m=4.0,
        dropper_search=True,
    )
)

launch_dropper = Objective(
    name='launchDropperObjective',
    success_frame_treshold=10,
    inference_mode=InferenceMode.DOWNWARD_ONLY,
    search=SearchConfig(spin_pwm=1400),
    center=CenterConfig(
        full_centering=False,
        x_center_tolerance_fov=0.08,
        y_center_tolerance_fov=0.08,
        center_bottom=True,
        target_offset_x=-0.25,
        target_offset_y=0.25,
    ),
    action=LaunchDropperAction(
        duration_s=2.0,
    ),
)

# launch_second_dropper = Objective(
#     name='launchSecondDropperObjective',
#     success_frame_treshold=10,
#     inference_mode=InferenceMode.DOWNWARD_ONLY,
#     search=SearchConfig(spin_pwm=1460),
#     center=CenterConfig(
#         full_centering=False,
#         x_center_tolerance_fov=0.05,
#         center_bottom=True,
#         target_offset_x=-0.17,
#         target_offset_y=0.15,
#     ),
#     action=LaunchDropperAction(
#         duration_s=2.0,
#         launch_second_dropper=True,
#     ),
# )

coarse_approach_torpedo = Objective(
         name='coarseApproachTorpedo',
         success_frame_treshold=10,
         target_auv_depth_m=1.2,
         inference_mode=InferenceMode.FORWARD_ONLY,
         target_ids=[ObjectID.TORPEDO, ObjectID.FIRE, ObjectID.BLOOD, ObjectID.FIRE_TRUCK, ObjectID.AMBULANCE],
         detections_depth_filter_mm = 30000,
         search=SearchConfig(spin_pwm=1560),
         center=CenterConfig(
             full_centering=False,
             x_center_tolerance_fov=0.2,
         ),
         approach=ApproachConfig(
             approach_distance_mm=1500.0,
         ),
     )
     
# torpedo_depth_change = Objective(
#          name='torpedoDepthChange',
#          target_ids=None,
#          target_auv_depth_m=1.2,
#          action=NoAction()
#      )

# fine_approach_torpedo = Objective(
#          name='fineApproachTorpedo',
#          success_frame_treshold=10,
#          target_ids=[ObjectID.TORPEDO],
#          center=CenterConfig(
#              full_centering=True,
#              x_center_tolerance_fov=0.1,
             
             
#             align_width=True,
#             target_width_px=180,
#             width_tolerance_px=10,
#          ),
#          approach=ApproachConfig(
#              approach_distance_mm=1000.0,
#          ),
#      )
    
torpedo_firing_positioning_1 = Objective(
    name='torpedoFiringPositioning1',
    success_frame_treshold=10,
    inference_mode=InferenceMode.FORWARD_ONLY,
    target_ids=None,
    center=CenterConfig(
        full_centering=False,
        x_center_tolerance_fov=0.02,
    ),
    action=FireTorpedoAction(
        min_lifespan_s=1.0,
    ),
)

torpedo_firing_positioning_2 = Objective(
    name='torpedoFiringPositioning2',
    success_frame_treshold=10,
    inference_mode=InferenceMode.FORWARD_ONLY,
    target_ids=None,
    center=CenterConfig(
        full_centering=False,
        x_center_tolerance_fov=0.02,
    ),
    action=FireTorpedoAction(
        min_lifespan_s=1.0,
    ),
)

dropper_octogon_transition = Objective(
    name='dropper_octogon_transition',
    inference_mode=InferenceMode.FORWARD_ONLY,
    detections_depth_filter_mm=30000,
    action=ForwardAction(
        forward_distance_m = -1.5,
    )
)

approach_table = Objective(
    name='approach_table',
    inference_mode=InferenceMode.FORWARD_ONLY,
    target_ids=[ObjectID.TABLE],
    detections_depth_filter_mm=30000,
    target_auv_depth_m = 1.3,
    success_frame_treshold= 10,
    search=SearchConfig(spin_pwm=1440),
    center=CenterConfig(
        full_centering=False,
        x_center_tolerance_fov=0.05
    ),
    approach=ApproachConfig(
        approach_distance_mm=3000.0
    ),
    action=NoAction()
)

center_over_table = Objective(
    name='center_over_table',
    target_ids=None,
    target_auv_depth_m = 0.6,
    action=ForwardAction(
        forward_distance_m=3.0,
    )
)

surface_octagon = Objective(
    name='surface_octagon',
    target_ids=None,
    target_auv_depth_m = 0.1,
    action=NoAction()
)

test_objective = Objective(
    name='testObjective',
    inference_mode=InferenceMode.FORWARD_ONLY,
    success_frame_treshold=10,
    detections_depth_filter_mm = 500,
    target_ids=[ObjectID.TORPEDO],
    center=CenterConfig(
        full_centering=False,
        x_center_tolerance_fov=0.1,
    ),
    approach=ApproachConfig(
        approach_distance_mm=1000.0,
    ),
    action=NoAction()
)

skip_slalom_objective = Objective(
    name='HM_slalo1',
    action=ForwardAction(
        forward_distance_m=3.0,
    )
)

# role_choice = ObjectID.SOS_SAFETY
role_choice = ObjectID.COMPASS_HAMMER


gate_list = [approach_gate, choose_gate_side, traverse_gate, lateral_after_gate]
slalom_list = [slalom1, slalom2, slalom3]
dropper_list = [approach_dropper, set_first_depth_for_dropper, set_second_depth_for_dropper, launch_dropper, dropper_octogon_transition]
torpedo_list = [coarse_approach_torpedo, torpedo_firing_positioning_1, torpedo_firing_positioning_2]
octogon_list = [approach_table, center_over_table, surface_octagon]
hail_mary = [skip_slalom_objective, slalom2, slalom3]

# mission_list = gate_list + slalom_list + torpedo_list + dropper_list + octogon_list
mission_list = dropper_list + octogon_list