from enum import IntEnum

class RobotState(IntEnum):
    IDLE = 0
    LOAD_OBJECTIVE = 1
    SEARCH_TARGET = 2
    CENTER_TARGET = 3
    APPROACH_TARGET = 4
    EXECUTE_ACTION = 5
    MISSION_COMPLETE = 6


    
    