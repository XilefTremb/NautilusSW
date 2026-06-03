import cv2
import numpy as np

DARK_THRESHOLD = 180
CLOSE_KERNEL_SIZE = (5, 15)
OPEN_KERNEL_SIZE = (3, 3)

def detect_object_in_roi(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # Black
    dark_threshold = min(DARK_THRESHOLD, np.percentile(v, 35))
    dark_mask = cv2.inRange(v, 0, int(dark_threshold))

    # White
    bright_threshold = max(200, np.percentile(v, 85))
    bright_mask = cv2.inRange(v, int(bright_threshold), 255)

    mask = cv2.bitwise_or(dark_mask, bright_mask)

    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filled = np.zeros_like(mask)

    if not contours:
        return filled

    biggest = max(contours, key=cv2.contourArea)

    cv2.drawContours(filled, [biggest], -1, 255, thickness=cv2.FILLED)

    return filled

def detect_dark_object_in_roi(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_dark = np.array([0, 0, 0])
    upper_dark = np.array([180, 255, DARK_THRESHOLD])

    mask = cv2.inRange(hsv, lower_dark, upper_dark)

    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, CLOSE_KERNEL_SIZE)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, OPEN_KERNEL_SIZE)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    filled = np.zeros_like(mask)

    if not contours:
        return filled

    biggest = max(contours, key=cv2.contourArea)

    cv2.drawContours(
        filled,
        [biggest],
        -1,
        255,
        thickness=cv2.FILLED
    )

    return filled


def find_depth_from_mask(depth_frame, x1, y1, x2, y2, mask):
    depth_roi = depth_frame[y1:y2, x1:x2]

    if depth_roi.size == 0 or mask.size == 0:
        return None

    valid_pixels = depth_roi[
        (mask > 0) &
        np.isfinite(depth_roi) &
        (depth_roi > 0)]

    if valid_pixels.size < 20:
        return None

    return float(np.median(valid_pixels))