import numpy as np


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

def find_angle_plane_V2(profondeurs, C):

    if any(p is None for p in profondeurs):
        return float(-1000) #erreur

    h_3 = np.abs(profondeurs[1] - profondeurs[0])*1000 #juste pour sim
    ratio = h_3 / C

    if ratio < -1.0 or ratio > 1.0:
        return float(-1000)  # erreur physique

    angle = np.arccos(ratio)
    angle = np.degrees(angle)

    return 90 - angle


def switch_case_sub_angle(objets):
    results_angle = []

    # Présence des objets

    gate_left_id =1
    gate_right_id = 2
    gate_middle_id = 3
    slalim_cote_id = 5
    slalom_middle_id = 4

    gate_left = gate_left_id in objets
    gate_right = gate_right_id in objets
    gate_middle = gate_middle_id in objets
    slalom_cote = slalim_cote_id in objets
    slalom_middle = slalom_middle_id in objets

    # Exemple : angle entre gate_left et gate_middle 
    if gate_left and gate_middle:
        profondeurs = [objets[gate_left_id]["depth"], objets[gate_middle_id]["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1524)

        results_angle.extend([float(1), angle_calc])

    # Exemple : angle entre gate_right et gate_middle
    if gate_right and gate_middle:
        profondeurs = [objets[gate_right_id]["depth"], objets[gate_middle_id]["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1524)

        results_angle.extend([float(2), angle_calc])

    # Exemple : angle entre slalom_cote et slalom_middle 
    if slalom_cote and slalom_middle:
        profondeurs = [objets[slalim_cote_id]["depth"], objets[slalom_middle_id]["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1500)

        results_angle.extend([float(3), angle_calc])

    return results_angle