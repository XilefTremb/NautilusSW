from pymavlink import mavutil
import time

pos_mask = int(0b010111111000)
vel_mask = int(0b110111000111)
vel_pos_mask = int(0b110111000000)
ingore_all = int(0b111111111111)


ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
CHGMODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE


the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

the_connection.wait_heartbeat()

print(f"Heartbeat from system: system {the_connection.target_system}  and component {the_connection.target_component}")

while True:
    msg = the_connection.recv_match(type='MISSION_ITEM_INT',blocking=True)
    print(msg)

