from enum import IntEnum

class ObjectID(IntEnum):
# model ids
    GATE_LEG_L = 0
    GATE_LEG_CENTER = 1
    GATE_LEG_R = 2
    SOS_SAFETY = 3 # Poisson SIM
    COMPASS_HAMMER = 4 # Requin SIM
    SLALOM_SIDE = 5
    SLALOM_CENTER = 6
    FIRE = 7 # Requin dropper SIM
    BLOOD = 8 # Poisson dropper SIM
    SOS = 9
    SAFETY = 10
    COMPASS = 11
    HAMMER = 12
    TORPEDO = 13
    GATE = 14
    DROPPER = 15
    TABLE = 16
    FIRE_TRUCK = 17
    AMBULANCE = 18
    TARGET = 19
    GATE_LEFT_MID = 20
    GATE_MID_RIGHT = 21
    SLALOM_LEFT_MID = 22
    SLALOM_MID_RIGHT = 23
    SLALOM_LEFT = 30
    SLALOM_RIGHT = 31
    TARGET_TRUCK = 32
    TARGET_AMBULANCE = 33
    TARGET_BLOOD = 34
    TARGET_FIRE = 35
    