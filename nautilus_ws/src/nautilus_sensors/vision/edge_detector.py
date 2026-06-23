import cv2
import numpy as np

# =====================================================
# LIGHT OBJECT DETECTOR
# =====================================================
#LIGHT_BRIGHT_PERCENTILE = 85
#LIGHT_MIN_BRIGHTNESS = 200

LIGHT_CLOSE_KERNEL_SIZE = (5, 5)  # Reduced from (7, 7) for speed
LIGHT_OPEN_KERNEL_SIZE = (3, 3)
LIGHT_CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, LIGHT_CLOSE_KERNEL_SIZE)  # Pre-computed
LIGHT_OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, LIGHT_OPEN_KERNEL_SIZE)    # Pre-computed

# =====================================================
# DARK OBJECT DETECTOR
# =====================================================
#DARK_THRESHOLD = 150
DARK_CLOSE_KERNEL_SIZE = (5, 15)
DARK_OPEN_KERNEL_SIZE = (3, 3)
DARK_CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, DARK_CLOSE_KERNEL_SIZE)  # Pre-computed
DARK_OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, DARK_OPEN_KERNEL_SIZE)    # Pre-computed


#MIN_PIXEL_COUNT = 30

def detect_light_object_in_roi(roi, edge_params):

    LIGHT_MIN_BRIGHTNESS, LIGHT_BRIGHT_PERCENTILE, MIN_PIXEL_COUNT, _ = get_edge_params(edge_params)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    bright_threshold = max(LIGHT_MIN_BRIGHTNESS, np.percentile(v, LIGHT_BRIGHT_PERCENTILE))
    bright_mask = cv2.inRange(v, int(bright_threshold),255)

    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_CLOSE, LIGHT_CLOSE_KERNEL)  # Use pre-computed kernel

    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN, LIGHT_OPEN_KERNEL)   # Use pre-computed kernel
    contours, _ = cv2.findContours(bright_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filled = np.zeros_like(bright_mask)

    if not contours:
        return filled

    biggest = max(contours, key=cv2.contourArea)

    if cv2.contourArea(biggest) < MIN_PIXEL_COUNT:
        return filled

    cv2.drawContours(filled, [biggest], -1, 255, thickness=cv2.FILLED)

    return filled

def detect_dark_object_in_roi(roi, edge_params):

    _,_,MIN_PIXEL_COUNT, DARK_THRESHOLD = get_edge_params(edge_params)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_dark = np.array([0, 0, 0])
    upper_dark = np.array([180, 255, DARK_THRESHOLD])

    mask = cv2.inRange(hsv, lower_dark, upper_dark)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, DARK_CLOSE_KERNEL)  # Use pre-computed kernel

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, DARK_OPEN_KERNEL)    # Use pre-computed kernel

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    filled = np.zeros_like(mask)

    if not contours:
        return filled

    biggest = max(contours, key=cv2.contourArea)

    if cv2.contourArea(biggest) < MIN_PIXEL_COUNT:
        return filled

    cv2.drawContours(
        filled,
        [biggest],
        -1,
        255,
        thickness=cv2.FILLED
    )

    return filled

def get_edge_params(edge_params):
    LIGHT_MIN_BRIGHTNESS = edge_params["light_min_brightness"]
    LIGHT_BRIGHT_PERCENTILE = edge_params["light_bright_percentile"]
    MIN_PIXEL_COUNT = edge_params["min_pixel_count"]
    DARK_THRESHOLD = edge_params["dark_threshold"]

    # print("LIGHT_MIN_BRIGHTNESS", LIGHT_MIN_BRIGHTNESS)
    # print("LIGHT_BRIGHT_PERCENTILE", LIGHT_BRIGHT_PERCENTILE)
    # print("MIN_PIXEL_COUNT", MIN_PIXEL_COUNT)
    # print("DARK_THRESHOLD", DARK_THRESHOLD)


    return LIGHT_MIN_BRIGHTNESS, LIGHT_BRIGHT_PERCENTILE, MIN_PIXEL_COUNT, DARK_THRESHOLD

