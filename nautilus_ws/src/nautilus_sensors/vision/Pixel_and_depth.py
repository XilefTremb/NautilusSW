import numpy as np
import cv2

def find_depth(depth_frame, half, bbox_y, bbox_x, mode):
    frame_h, frame_w = depth_frame.shape

    y1 = max(0, bbox_y- half)
    y2 = min(frame_h, bbox_y+ half + 1)
    x1 = max(0, bbox_x- half)
    x2 = min(frame_w, bbox_x+ half + 1)

    center_patch = depth_frame[y1:y2, x1:x2]
    #print("patch:", center_patch)

    
    center_depth = float(np.median(center_patch))
    #print(center_depth)
    """
    valid_center = center_patch[(center_patch > 200) & (center_patch < 5000)]

    if valid_center.size > 0:
        center_depth = float(np.median(valid_center))
    else:
        #print("Center depth: invalid")
        center_depth = None
    """
    """
    # Calcul vraie distance
    X = (bbox_x- cx) * center_depth / fx
    Y = (bbox_y- cy) * center_depth / fy

    true_distance = np.sqrt(X**2 + Y**2 + center_depth**2)
    """

    return center_depth


def find_angle(x_center, depth_mean, mode):

    cx, fx, fy, cy = params_cams(mode)

    if depth_mean is None or int(depth_mean) == 0:
        return None
    # Coordonnée horizontale dans le repère caméra
    X = (x_center - cx) * depth_mean / fx

    # Yaw pour aligner l'objet avec le centre
    yaw = np.arctan2(X, depth_mean)
    yaw_deg = np.degrees(yaw)

    return yaw_deg

def find_dist_from_center(x_center, mode):
    cx, fx, fy, cy = params_cams(mode)

    return float(x_center - cx)

def params_cams(mode):

    if mode == "sim":
        # SIMULATION
        cx = 320
        fx = 293
        fy = 293
        cy = 240

        return cx, fx, fy, cy

    if mode == "real":
        # OAK-D S1
        cx = 640
        fx = 728
        fy = 726
        cy = 370

        return cx, fx, fy, cy

    return None