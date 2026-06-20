import numpy as np
from scipy.ndimage import median_filter
from scipy import ndimage
from vision.edge_detector import *

from enums.ObjectID import ObjectID

# ---------------- FILL ZEROS ----------------
def fill_zeros_with_nearest_fast(depth_patch):
    depth = depth_patch.copy()

    zero_mask = (depth == 0)

    if not np.any(zero_mask):
        return depth

    if np.all(zero_mask):
        return depth

    _, indices = ndimage.distance_transform_edt(
        zero_mask,
        return_indices=True
    )

    filled = depth.copy()
    filled[zero_mask] = depth[indices[0][zero_mask], indices[1][zero_mask]]

    return filled


# ---------------- FILTER PATCH ----------------
def filter_depth_bbox(depth_frame, x1, y1, x2, y2, kernel_size, margin):
    frame_h, frame_w = depth_frame.shape

    #Bigger zone
    x1_big = max(0, int(x1 - margin))
    y1_big = max(0, int(y1 - margin))
    x2_big = min(frame_w, int(x2 + margin))
    y2_big = min(frame_h, int(y2 + margin))

    bbox_patch = depth_frame[y1_big:y2_big, x1_big:x2_big]

    if bbox_patch.size == 0:
        return None

    # fill holes
    bbox_patch_filled = fill_zeros_with_nearest_fast(bbox_patch)

    #Median filter
    bbox_filtered = median_filter(
        bbox_patch_filled,
        size=kernel_size,
        mode='nearest'
    )

    #Back to original size
    local_x1 = x1 - x1_big
    local_y1 = y1 - y1_big
    local_x2 = local_x1 + (x2 - x1)
    local_y2 = local_y1 + (y2 - y1)

    center_patch_filtered = bbox_filtered[
        local_y1:local_y2,
        local_x1:local_x2
    ]

    return center_patch_filtered

def find_depth(depth_frame, half, bbox_cy, bbox_cx, mode):
    frame_h, frame_w = depth_frame.shape

    y1 = max(0, bbox_cy- half)
    y2 = min(frame_h, bbox_cy+ half + 1)
    x1 = max(0, bbox_cx- half)
    x2 = min(frame_w, bbox_cx+ half + 1)

    if mode == "real":
        center_patch = filter_depth_bbox(depth_frame, x1, y1, x2, y2, 5, half)
        if center_patch is None or center_patch.size == 0:
            return -1000.0

    else:
        center_patch = depth_frame[y1:y2, x1:x2]

    center_depth = float(np.median(center_patch))

    if mode == "sim":
        center_depth = center_depth * 1000

    if np.isnan(center_depth) or center_depth == 0 or center_depth is None:
        return -1000.0

    return center_depth

def find_dist_from_center_in_x(x_center, mode, cam):
    cx, fx, fy, cy = params_cams(mode, cam)
    return float((x_center - cx)/cx)

def find_dist_from_center_in_y(y_center, mode, cam):
    cx, fx, fy, cy = params_cams(mode, cam)
    return float((y_center - cy)/cy)

def global_median_forward_cam(depth_frame, mode, cam):
    cx, fx, fy, cy = params_cams(mode, cam)

    h, w = depth_frame.shape

    #Zone
    x1 = max(0, int(cx - cx/2))
    x2 = min(w, int(cx + cx/2))
    y1 = max(0, int(cy - cy/2))
    y2 = min(h, int(cy + cy/2))

    patch = depth_frame[y1:y2, x1:x2]

    #Remove zeros median
    valid_values = patch[patch != 0]

    if valid_values.size == 0:
        return 0
    
    global_depth = np.median(valid_values)

    if mode == "sim":
        global_depth = global_depth*1000
        if not np.isfinite(global_depth) or global_depth > 20000:
            global_depth = 20000

    return global_depth

def find_depth_from_edge_detector(depth_frame, x1, y1, x2, y2, annotated_frame, id, edge_params):

    roi = annotated_frame[y1:y2, x1:x2]

    if roi.size == 0:
        return None, None

    if id == ObjectID.SLALOM_SIDE:
        filled_mask = detect_light_object_in_roi(roi, edge_params)

    else:
        filled_mask = detect_dark_object_in_roi(roi, edge_params)

    depth_roi = depth_frame[y1:y2, x1:x2]

    if depth_roi.size == 0 or filled_mask.size == 0:
        return None, None

    valid_pixels = depth_roi[
        (filled_mask > 0) &
        np.isfinite(depth_roi) &
        (depth_roi > 0)]

    if valid_pixels.size < 20:
        return None, None

    return float(np.median(valid_pixels)), filled_mask

def params_cams(mode, cam):

    if mode == "sim":
        # SIMULATION
        cx = 320
        fx = 293
        fy = 293
        cy = 240

        return cx, fx, fy, cy

    if mode == "real":
        if cam == "forward":
            # OAK-D S1
            cx = 640
            fx = 728
            fy = 726
            cy = 480

            return cx, fx, fy, cy

        if cam == "downward":
            # OAK-D S1
            cx = 640
            fx = 728
            fy = 726
            cy = 360

            return cx, fx, fy, cy


    return None
#1280X720 DOWN
#1280X960 FORWARD