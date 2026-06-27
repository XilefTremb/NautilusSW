from enum import IntEnum

class ServoEnum(IntEnum):
    DROPPER_ID = 12
    TORPEDO_ID = 11
    DROPPER_INIT_PWM = 1500
    DROPPER_1_PWM = 1100
    DROPPER_2_PWM = 1900
    TORPEDO_INIT_PWM = 1250
    TORPEDO_L_PWM = 800 
    TORPEDO_R_PWM = 2000