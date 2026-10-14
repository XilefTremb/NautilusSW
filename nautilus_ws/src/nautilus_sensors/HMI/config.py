# config.py

TOPICS = {
    "imu": "/imu",
    "dvl": "/dvl/twist",
    "depth": "/ap/pose/filtered",
    "battery": "/ap/battery",
    "motor_pwm": "/ap/rc",
    "motor_current": "/motors/current",
    "profiling": "/yolo/profiling",
    "camera_depth": "/yolo/mean_depth_forward_cam",
    "inference_mode": "/yolo/inference_mode",
}

MOTOR_COUNT = 8

GRAPH_HISTORY_SECONDS = 30
GRAPH_UPDATE_MS = 50