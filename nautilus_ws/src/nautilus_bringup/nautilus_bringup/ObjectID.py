from enum import IntEnum

class ObjectID(IntEnum):
#official list to be used on next model training
    # GATE_LEFT = 0
    # GATE_MID = 1
    # GATE_RIGHT = 2
    # SOS_SAFETY = 3
    # COMPASS_HAMMER = 4
    # SLALOM_SIDE = 5
    # SLALOM_CENTER = 6
    # FIRE = 7
    # BLOOD = 8
    # SOS = 9
    # SAFETY = 10
    # COMPASS = 11
    # HAMMER = 12
    # TORPEDO = 13
    # GATE = 14

#SIM model ids
    GATE = 0
    GATE_LEG_L = 1
    GATE_LEG_R = 2
    GATE_LEG_CENTER = 3
    SLALOM_CENTER = 4
    SLALOM_SIDE = 5
    REQUIN = 6
    POISSON = 7

    
    