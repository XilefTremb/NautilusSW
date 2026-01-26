from pymavlink import mavutil
import time

pos_mask = int(0b100111111000)
vel_mask = int(0b110111000111)
vel_pos_mask = int(0b110111000000)
ingore_all = int(0b111111111111)


ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
CHGMODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE


the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

the_connection.wait_heartbeat()

print(f"Heartbeat from system: system {the_connection.target_system}  and component {the_connection.target_component}")

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, ARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, CHGMODE, 0, 1, 4, 0, 0, 0, 0, 0)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True) 
print(msg)

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                                                                                   pos_mask, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0))

# msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
# print(msg)


# msg = the_connection.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=True)



while True:
#     msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=False)
     msg2 = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
#     msg3 = the_connection.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=False)
#     print(msg)
     print(msg2)
#     print(msg3)