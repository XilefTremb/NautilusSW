import numpy as np
from scipy.ndimage import median_filter
from scipy import ndimage

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
            return -1000

    else:
        center_patch = depth_frame[y1:y2, x1:x2]

    center_depth = float(np.median(center_patch))

    if mode == "sim":
        center_depth = center_depth*1000

    if np.isnan(center_depth) or center_depth == 0 or center_depth is None:
        return -1000.0

    return center_depth


def find_angle(x_center, depth_mean, mode):
    cx, fx, fy, cy = params_cams(mode)

    if depth_mean is None or int(depth_mean) == 0:
        return None
    #X pos
    X = (x_center - cx) * depth_mean / fx

    #Yaw compare with middle cam
    yaw = np.arctan2(X, depth_mean)
    yaw_deg = np.degrees(yaw)

    return yaw_deg

def find_dist_from_center(x_center, mode):
    cx, fx, fy, cy = params_cams(mode)

    return float(x_center - cx)

def global_median_forward_cam(depth_frame, mode):
    cx, fx, fy, cy = params_cams(mode)

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

def params_cams(mode):

    if mode == "sim":
        # SIMULATION
        cx = 160
        fx = 293
        fy = 293
        cy = 120

        return cx, fx, fy, cy

    if mode == "real":
        # OAK-D S1
        cx = 640
        fx = 728
        fy = 726
        cy = 370

        return cx, fx, fy, cy

    return None