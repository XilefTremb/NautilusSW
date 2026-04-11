import numpy as np
import cv2


#PARAMETRES DE LA CAMERA
cx = 623
fx = 728
fy = 726
cy = 370

def find_depth(depth_frame, half, cy_boundingbox, cx_boundingbox):
    frame_h, frame_w = depth_frame.shape

    y1 = max(0, cy_boundingbox - half)
    y2 = min(frame_h, cy_boundingbox + half + 1)
    x1 = max(0, cx_boundingbox - half)
    x2 = min(frame_w, cx_boundingbox + half + 1)

    center_patch = depth_frame[y1:y2, x1:x2]

    valid_center = center_patch[(center_patch > 200) & (center_patch < 5000)]

    if valid_center.size > 0:
        center_depth = float(np.median(valid_center))
    else:
        print("Center depth: invalid")
        center_depth = None

    """
    # Calcul vraie distance
    X = (cx_boundingbox - cx) * center_depth / fx
    Y = (cy_boundingbox - cy) * center_depth / fy

    true_distance = np.sqrt(X**2 + Y**2 + center_depth**2)
    """

    return center_depth


def find_angle(x_center, depth_mean):
    if depth_mean is None:
        return None

    # Coordonnée horizontale dans le repère caméra
    X = (x_center - cx) * depth_mean / fx

    # Yaw pour aligner l'objet avec le centre
    yaw = np.arctan2(X, depth_mean)
    yaw_deg = np.degrees(yaw)

    return yaw_deg