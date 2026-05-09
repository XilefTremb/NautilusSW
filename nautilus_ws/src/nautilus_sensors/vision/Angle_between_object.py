import numpy as np
from nautilus_bringup.ObjectID import ObjectID
from vision.Pixel_and_depth import params_cams

def find_angle_plane_V2(middle_depth, side_depth, C, middle_x, side_x, mode):

    h_3 = middle_depth - side_depth
    ratio = np.abs(h_3 / C)

    if ratio < -1.0 or ratio > 1.0:
        return float(-1000)  # erreur physique

    angle = np.arccos(ratio)
    angle = np.degrees(angle)

    if h_3 > 0 and middle_x-side_x > 0 or middle_x-side_x < 0 and h_3 < 0:
        return -(90-angle)

    else:
        return 90 - angle

def find_dist_center_gate(cx, left_x, right_x):
    return min(left_x, right_x) + abs(int((left_x - right_x)/2)) - cx 


def switch_case_sub_angle(objects, mode):
    results_angle = []
    """
    gate_left = ObjectID.GATE_LEG_L in objects
    gate_right = ObjectID.GATE_LEG_R in objects
    gate_middle = ObjectID.GATE_LEG_CENTER in objects
    slalom_side = ObjectID.SLALOM_SIDE in objects
    slalom_middle = ObjectID.SLALOM_CENTER in objects
    
    if gate_left and gate_middle:
        left_obj = objects[ObjectID.GATE_LEG_L]
        right_obj = objects[ObjectID.GATE_LEG_CENTER]

        angle_calc = find_angle_plane_V2(right_obj["depth"],left_obj["depth"],1524, right_obj["bbox_cx"],left_obj["bbox_cx"], mode)
        cx, _, _, _= params_cams(mode)
        dist_center_gate = find_dist_center_gate(cx, left_obj["bbox_cx"], right_obj["bbox_cx"])

        results_angle.extend([float(1), angle_calc, float(dist_center_gate)])

    if gate_right and gate_middle:
        left_obj = objects[ObjectID.GATE_LEG_CENTER]
        right_obj = objects[ObjectID.GATE_LEG_R]

        angle_calc = find_angle_plane_V2(left_obj["depth"],right_obj["depth"],1524, left_obj["bbox_cx"],right_obj["bbox_cx"], mode)
        cx, _, _, _= params_cams(mode)
        dist_center_gate = find_dist_center_gate(cx, left_obj["bbox_cx"], right_obj["bbox_cx"])

        results_angle.extend([float(2), angle_calc, float(dist_center_gate)])

    if slalom_side and slalom_middle:
        # TODO: may have to treat case if side slalom is on the left or right of center slalom
        left_obj = objects[ObjectID.SLALOM_SIDE]
        right_obj = objects[ObjectID.SLALOM_CENTER]

        angle_calc = find_angle_plane_V2(right_obj["depth"],left_obj["depth"],1524, right_obj["bbox_cx"],left_obj["bbox_cx"], mode)
        cx, _, _, _= params_cams(mode)
        dist_center_gate = find_dist_center_gate(cx, left_obj["bbox_cx"], right_obj["bbox_cx"])

        results_angle.extend([float(3), angle_calc, float(dist_center_gate)])
    """

    gate_prequalif = (0 in objects and 1 in objects)

    if gate_prequalif:
        cx, _, _, _ = params_cams(mode)

        gate_L = objects.get(0)
        gate_R = objects.get(1)

        if gate_L is not None and gate_R is not None:
            angle_calc = find_angle_plane_V2(gate_L["depth"],gate_R["depth"],2100, gate_L["bbox_cx"],gate_R["bbox_cx"], mode)
            dist_center_gate = find_dist_center_gate(cx, gate_L["bbox_cx"], gate_R["bbox_cx"])
            results_angle.extend([float(3), angle_calc, float(dist_center_gate)])

    return results_angle