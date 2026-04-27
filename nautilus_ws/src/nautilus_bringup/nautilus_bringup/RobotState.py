from enum import IntEnum

class RobotState(IntEnum):
    IDLE = 0
    SEARCH = 1
    APPROACH = 2
    CENTER_GATE = 3
    APPROACH_GATE = 4
    TRAVERSE_GATE = 5


    
    