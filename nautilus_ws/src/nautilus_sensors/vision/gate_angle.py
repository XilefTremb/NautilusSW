import numpy as np
from enums.ObjectID import ObjectID
from vision.object_depth import params_cams


def find_angle_plane(left_depth, right_depth, C, left_x, right_x):

    h_3 = left_depth - right_depth
    ratio = np.abs(h_3 / C)

    if ratio < -1.0 or ratio > 1.0:
        return None # erreur physique

    angle = np.arccos(ratio)
    angle = np.degrees(angle)

    if h_3 > 0 and left_x-right_x > 0 or left_x-right_x < 0 and h_3 < 0:
        return -(90-angle)

    else:
        return 90 - angle

def find_dist_center_gate(cx, left_x, right_x):
    return (min(left_x, right_x) + abs(int((left_x - right_x)/2)) - cx) / cx

def build_payload(GATE_ID, left_obj, right_obj, results_angle, mode):
    angle_calc = find_angle_plane(right_obj["depth"],left_obj["depth"],1524, right_obj["box_cx"],left_obj["box_cx"])
    cx, _, _, _= params_cams(mode)
    dist_center_gate = find_dist_center_gate(cx, left_obj["box_cx"], right_obj["box_cx"])
    mean_distance = (right_obj["depth"] + left_obj["depth"]) / 2
    if angle_calc is not None and abs(angle_calc) < 50 and dist_center_gate is not None:
        results_angle.extend([float(GATE_ID), float(dist_center_gate), float(mean_distance), float(angle_calc)])
    
def find_gate_angle(objects, mode):
    results_angle = []

    #print(f"{objects}")

    gate_left = ObjectID.GATE_LEG_L in objects
    gate_right = ObjectID.GATE_LEG_R in objects
    gate_middle = ObjectID.GATE_LEG_CENTER in objects
    slalom_left = ObjectID.SLALOM_LEFT in objects
    slalom_right = ObjectID.SLALOM_RIGHT in objects
    slalom_middle = ObjectID.SLALOM_CENTER in objects

    if gate_left and gate_middle:
        left_obj = objects[ObjectID.GATE_LEG_L]
        right_obj = objects[ObjectID.GATE_LEG_CENTER]
        build_payload(ObjectID.GATE_LEFT_MID, left_obj, right_obj,results_angle, mode)
    

    if gate_right and gate_middle:
        left_obj = objects[ObjectID.GATE_LEG_CENTER]
        right_obj = objects[ObjectID.GATE_LEG_R]
        build_payload(ObjectID.GATE_MID_RIGHT, left_obj, right_obj,results_angle, mode)

    if slalom_left and slalom_middle:
        left_obj = objects[ObjectID.SLALOM_LEFT]
        right_obj = objects[ObjectID.SLALOM_CENTER]
        build_payload(ObjectID.SLALOM_LEFT_MID, left_obj, right_obj, results_angle, mode)

    if slalom_right and slalom_middle:
        left_obj = objects[ObjectID.SLALOM_CENTER]
        right_obj = objects[ObjectID.SLALOM_RIGHT]
        build_payload(ObjectID.SLALOM_MID_RIGHT, left_obj, right_obj, results_angle, mode)

    return results_angle