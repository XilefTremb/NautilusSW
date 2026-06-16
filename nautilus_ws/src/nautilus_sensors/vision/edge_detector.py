import cv2
import numpy as np

# =====================================================
# LIGHT OBJECT DETECTOR
# =====================================================
LIGHT_BRIGHT_PERCENTILE = 60
LIGHT_MIN_BRIGHTNESS = 100

LIGHT_CLOSE_KERNEL_SIZE = (7, 7)
LIGHT_OPEN_KERNEL_SIZE = (3, 3)

# =====================================================
# DARK OBJECT DETECTOR
# =====================================================
DARK_THRESHOLD = 200
DARK_CLOSE_KERNEL_SIZE = (5, 15)
DARK_OPEN_KERNEL_SIZE = (3, 3)


MIN_PIXEL_COUNT = 30

def detect_light_object_in_roi(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    bright_threshold = max(LIGHT_MIN_BRIGHTNESS, np.percentile(v, LIGHT_BRIGHT_PERCENTILE))
    bright_mask = cv2.inRange(v, int(bright_threshold),255)

    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,LIGHT_CLOSE_KERNEL_SIZE)
    bright_mask = cv2.morphologyEx(bright_mask,cv2.MORPH_CLOSE,close_kernel)

    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,LIGHT_OPEN_KERNEL_SIZE)
    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN,open_kernel)
    contours, _ = cv2.findContours(bright_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filled = np.zeros_like(bright_mask)

    if not contours:
        return filled

    biggest = max(contours, key=cv2.contourArea)

    if cv2.contourArea(biggest) < MIN_PIXEL_COUNT:
        return filled

    cv2.drawContours(filled, [biggest], -1, 255, thickness=cv2.FILLED)

    return filled

def detect_dark_object_in_roi(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_dark = np.array([0, 0, 0])
    upper_dark = np.array([180, 255, DARK_THRESHOLD])

    mask = cv2.inRange(hsv, lower_dark, upper_dark)

    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, DARK_CLOSE_KERNEL_SIZE)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, DARK_OPEN_KERNEL_SIZE)
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
