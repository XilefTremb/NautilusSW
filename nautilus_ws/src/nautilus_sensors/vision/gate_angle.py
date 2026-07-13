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

def find_dist_center_gate_in_x(cx, left_x, right_x):
    return (min(left_x, right_x) + abs(int((left_x - right_x)/2)) - cx) / cx

def find_dist_center_gate_in_y(cy, left_y, right_y):
    return (min(left_y, right_y) + abs(int((left_y - right_y)/2)) - cy) / cy

def build_payload(GATE_ID, left_obj, right_obj, results_angle, mode):
    angle_calc = find_angle_plane(right_obj["depth"],left_obj["depth"],1524, right_obj["box_cx"],left_obj["box_cx"])
    cx, _, _, cy= params_cams(mode, "forward")
    dist_center_gate_in_x = find_dist_center_gate_in_x(cx, left_obj["box_cx"], right_obj["box_cx"])
    dist_center_gate_in_y = find_dist_center_gate_in_y(cy, left_obj["box_cy"], right_obj["box_cy"])
    mean_distance = (right_obj["depth"] + left_obj["depth"]) / 2
    if angle_calc is not None and abs(angle_calc) < 35 and dist_center_gate_in_x is not None:
        results_angle.extend([float(GATE_ID), float(dist_center_gate_in_x), float(mean_distance), float(angle_calc), float(dist_center_gate_in_y), 0.0, 0.0])
    
def find_gate_angle(objects, mode):
    results_angle = []

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

def find_angle_torpedo(objects):
    angles = []

    if ObjectID.FIRE_TRUCK in objects and ObjectID.AMBULANCE in objects:
        angle = find_angle_plane(
            objects[ObjectID.FIRE_TRUCK]["depth"],
            objects[ObjectID.AMBULANCE]["depth"],
            1524,
            objects[ObjectID.FIRE_TRUCK]["box_cx"],
            objects[ObjectID.AMBULANCE]["box_cx"])
        if angle is not None:
            angles.append(angle)

    if ObjectID.BLOOD in objects and ObjectID.FIRE in objects:
        angle = find_angle_plane(
            objects[ObjectID.BLOOD]["depth"],
            objects[ObjectID.FIRE]["depth"],
            1524,
            objects[ObjectID.BLOOD]["box_cx"],
            objects[ObjectID.FIRE]["box_cx"])
        if angle is not None:
            angles.append(angle)

    if not angles:
        return 0.0

    return float(np.mean(angles))