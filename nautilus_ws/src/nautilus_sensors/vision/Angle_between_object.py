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
        return -1000 #erreur

    h_3 = np.abs(profondeurs[1] - profondeurs[0])
    ratio = h_3 / C

    if ratio < -1.0 or ratio > 1.0:
        return -1000  # erreur physique

    angle = np.arccos(ratio)
    angle = np.degrees(angle)

    return 90 - angle


def switch_case_sub_angle(objets):
    results_angle = []

    # Présence des objets
    gate_left = 1 in objets
    gate_right = 2 in objets
    gate_middle = 3 in objets
    slalom_cote = 5 in objets
    slalom_middle = 4 in objets

    # Exemple : angle entre gate_left (0) et gate_middle (2)
    if gate_left and gate_middle:
        profondeurs = [objets[0]["depth"], objets[2]["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1524)

        results_angle.extend([1, angle_calc])

    # Exemple : angle entre gate_right (1) et gate_middle (2)
    if gate_right and gate_middle:
        profondeurs = [objets[1]["depth"], objets[2]["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1524)

        results_angle.extend([2, angle_calc])

    # Exemple : angle entre slalom_cote (3) et slalom_middle (4)
    if slalom_cote and slalom_middle:
        profondeurs = [objets[3]["depth"], objets[4]["depth"]]
        angle_calc = find_angle_plane_V2(profondeurs, 1500)

        results_angle.extend([3, angle_calc])

    return results_angle