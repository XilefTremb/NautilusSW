# config.py

TOPICS = {
    "imu": "/imu/data",
    "dvl": "/dvl/velocity",
    "depth": "/depth",
    "battery_voltage": "/battery/voltage",
    "battery_current": "/battery/current",
    "motor_pwm": "/motors/pwm",
    "motor_current": "/motors/current",
}

MOTOR_COUNT = 8

GRAPH_HISTORY_SECONDS = 30
GRAPH_UPDATE_MS = 50