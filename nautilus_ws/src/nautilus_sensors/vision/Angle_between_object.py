import numpy as np
from nautilus_bringup.ObjectID import ObjectID
from vision.Pixel_and_depth import params_cams

"""
def find_angle_plane(boxes):
    profondeurs = []
    angles = []
    C = 1442

    for box in boxes:
        profondeurs.append(box["depth"])
        angles.append(box["angle"])

    h_3 = np.abs(profondeurs[1] - profondeurs[0])
    angle = np.arccos(h_3/C)
    angle = np.degrees(angle)

    result = 90 - angle

    return result
    
"""

def find_angle_plane_V2(profondeurs, C, mode):

    if any(p is None for p in profondeurs):
        return float(-1000) #erreur

    h_3 = np.abs(profondeurs[1] - profondeurs[0])
    ratio = h_3 / C

    if ratio < -1.0 or ratio > 1.0:
        return float(-1000)  # erreur physique

    angle = np.arccos(ratio)
    angle = np.degrees(angle)

    return 90 - angle

def find_dist_center_gate(cx, left_x, right_x):
    return min(left_x, right_x) + abs(int((left_x - right_x)/2)) - cx 


def switch_case_sub_angle(objects, mode):
    results_angle = []

    gate_left = ObjectID.GATE_LEG_L in objects
    gate_right = ObjectID.GATE_LEG_R in objects
    gate_middle = ObjectID.GATE_LEG_CENTER in objects
    slalom_side = ObjectID.SLALOM_SIDE in objects
    slalom_middle = ObjectID.SLALOM_CENTER in objects

    # Exemple : angle entre gate_left et gate_middle 
    if gate_left and gate_middle:
        left_obj = objects[ObjectID.GATE_LEG_L]
        right_obj = objects[ObjectID.GATE_LEG_CENTER]

        profondeurs = [left_obj["depth"], right_obj["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1524, mode)
        cx, _, _, _= params_cams(mode)
        dist_center_gate = find_dist_center_gate(cx, left_obj["bbox_cx"], right_obj["bbox_cx"])

        results_angle.extend([float(1), angle_calc, float(dist_center_gate)])

    # # Exemple : angle entre gate_right et gate_middle
    if gate_right and gate_middle:
        left_obj = objects[ObjectID.GATE_LEG_CENTER]
        right_obj = objects[ObjectID.GATE_LEG_R]

        profondeurs = [left_obj["depth"], right_obj["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1524, mode)
        cx, _, _, _= params_cams(mode)
        dist_center_gate = find_dist_center_gate(cx, left_obj["bbox_cx"], right_obj["bbox_cx"])

        results_angle.extend([float(2), angle_calc, float(dist_center_gate)])
 
    # # Exemple : angle entre slalom_cote et slalom_middle 
    if slalom_side and slalom_middle:
        # TODO: may have to treat case if side slalom is on the left or right of center slalom
        left_obj = objects[ObjectID.SLALOM_SIDE]
        right_obj = objects[ObjectID.SLALOM_CENTER]

        profondeurs = [left_obj["depth"], right_obj["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1500, mode)
        cx, _, _, _= params_cams(mode)
        dist_center_gate = find_dist_center_gate(cx, left_obj["bbox_cx"], right_obj["bbox_cx"])

        results_angle.extend([float(3), angle_calc, float(dist_center_gate)])

    return results_angle