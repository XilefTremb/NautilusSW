import numpy as np
import cv2
import math

THRESHOLD_RATIO = 2000
MIN_AVG_RED = 60
MAX_HUE_SHIFT = 120
BLUE_MAGIC_VALUE = 1.2


def hue_shift_red(mat, h):
    U = math.cos(h * math.pi / 180)
    W = math.sin(h * math.pi / 180)

    r = (0.299 + 0.701 * U + 0.168 * W) * mat[..., 0]
    g = (0.587 - 0.587 * U + 0.330 * W) * mat[..., 1]
    b = (0.114 - 0.114 * U - 0.497 * W) * mat[..., 2]

    return np.dstack([r, g, b])


def normalizing_interval(array):
    high = 255
    low = 0
    max_dist = 0

    for i in range(1, len(array)):
        dist = array[i] - array[i - 1]
        if dist > max_dist:
            max_dist = dist
            high = array[i]
            low = array[i - 1]

    return low, high


def get_filter_matrix(mat):
    # Plus petit = plus rapide
    #mat = cv2.resize(mat, (96, 96))

    avg_mat = np.array(cv2.mean(mat)[:3], dtype=np.uint8)

    new_avg_r = avg_mat[0]
    hue_shift = 0

    while new_avg_r < MIN_AVG_RED:
        shifted = hue_shift_red(avg_mat, hue_shift)
        new_avg_r = np.sum(shifted)
        hue_shift += 1

        if hue_shift > MAX_HUE_SHIFT:
            new_avg_r = MIN_AVG_RED

    shifted_mat = hue_shift_red(mat, hue_shift)
    new_r_channel = np.sum(shifted_mat, axis=2)
    new_r_channel = np.clip(new_r_channel, 0, 255)
    mat[..., 0] = new_r_channel

    hist_r = cv2.calcHist([mat], [0], None, [256], [0, 256])
    hist_g = cv2.calcHist([mat], [1], None, [256], [0, 256])
    hist_b = cv2.calcHist([mat], [2], None, [256], [0, 256])

    normalize_mat = np.zeros((256, 3))
    threshold_level = (mat.shape[0] * mat.shape[1]) / THRESHOLD_RATIO

    for x in range(256):
        if hist_r[x] < threshold_level:
            normalize_mat[x][0] = x

        if hist_g[x] < threshold_level:
            normalize_mat[x][1] = x

        if hist_b[x] < threshold_level:
            normalize_mat[x][2] = x

    normalize_mat[255] = [255, 255, 255]

    adjust_r_low, adjust_r_high = normalizing_interval(normalize_mat[..., 0])
    adjust_g_low, adjust_g_high = normalizing_interval(normalize_mat[..., 1])
    adjust_b_low, adjust_b_high = normalizing_interval(normalize_mat[..., 2])

    # Protection division par zéro
    adjust_r_high = max(adjust_r_high, adjust_r_low + 1)
    adjust_g_high = max(adjust_g_high, adjust_g_low + 1)
    adjust_b_high = max(adjust_b_high, adjust_b_low + 1)

    shifted = hue_shift_red(np.array([1, 1, 1]), hue_shift)
    shifted_r, shifted_g, shifted_b = shifted[0][0]

    red_gain = 256 / (adjust_r_high - adjust_r_low)
    green_gain = 256 / (adjust_g_high - adjust_g_low)
    blue_gain = 256 / (adjust_b_high - adjust_b_low)

    red_offset = (-adjust_r_low / 256) * red_gain
    green_offset = (-adjust_g_low / 256) * green_gain
    blue_offset = (-adjust_b_low / 256) * blue_gain

    adjust_red = shifted_r * red_gain
    adjust_red_green = shifted_g * red_gain
    adjust_red_blue = shifted_b * red_gain * BLUE_MAGIC_VALUE

    return np.array([
        adjust_red, adjust_red_green, adjust_red_blue, 0, red_offset,
        0, green_gain, 0, 0, green_offset,
        0, 0, blue_gain, 0, blue_offset,
        0, 0, 0, 1, 0,
    ], dtype=np.float32)


def apply_filter_fast(mat, filt):
    mat_f = mat.astype(np.float32)

    M = np.array([
        [filt[0], filt[1], filt[2]],
        [0, filt[6], 0],
        [0, 0, filt[12]]
    ], dtype=np.float32)

    bias = np.array([
        filt[4] * 255,
        filt[9] * 255,
        filt[14] * 255
    ], dtype=np.float32)

    out = cv2.transform(mat_f, M)
    out += bias

    return np.clip(out, 0, 255).astype(np.uint8)


def blue_filter(frame):
    """
    Input : frame BGR OpenCV
    """

    rgb_mat = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    filter_matrix = get_filter_matrix(rgb_mat)

    corrected = apply_filter_fast(rgb_mat, filter_matrix)

    corrected = cv2.cvtColor(corrected, cv2.COLOR_RGB2BGR)

    return corrected