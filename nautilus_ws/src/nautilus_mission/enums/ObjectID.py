from enum import IntEnum

class ObjectID(IntEnum):
# official list to be used on next model training
    GATE_LEG_L = 0
    GATE_LEG_CENTER = 1
    GATE_LEG_R = 2
    SOS_SAFETY = 3
    COMPASS_HAMMER = 4
    SLALOM_SIDE = 5
    SLALOM_CENTER = 6
    FIRE = 7
    BLOOD = 8
    SOS = 9
    SAFETY = 10
    COMPASS = 11
    HAMMER = 12
    TORPEDO = 13
    GATE = 14
    SLALOM_LEFT = 15
    SLALOM_RIGHT = 16
    GATE_LEFT_MID = 20
    GATE_MID_RIGHT = 21
    SLALOM_LEFT_MID = 22
    SLALOM_MID_RIGHT = 23

# #SIM model ids
#     GATE = 0
#     GATE_LEG_L = 1
#     GATE_LEG_R = 2
#     GATE_LEG_CENTER = 3
#     SLALOM_CENTER = 4
#     SLALOM_SIDE = 5
#     REQUIN = 6
#     POISSON = 7
#     SLALOM_LEFT = 8
#     SLALOM_RIGHT = 9
#     GATE_LEFT_MID = 20
#     GATE_MID_RIGHT = 21
#     SLALOM_LEFT_MID = 22
#     SLALOM_MID_RIGHT = 23

#Prequal model ids
    # GATE_LEG = 2
    # GATE_TOTAL = 3
    # MARQUEUR = 4
    # LUMIERE = 5
    