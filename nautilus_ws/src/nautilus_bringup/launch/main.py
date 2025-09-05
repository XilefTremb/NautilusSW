from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

the_connection.wait_heartbeat()

print(f"Heartbeat from system: system {the_connection.target_system}  and component {the_connection.target_component}")

# while True:
#     msg = the_connection.recv_match(type='RC_CHANNELS',blocking=True)
#     print(msg)


ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF


the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, ARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


the_connection.mav.rc_channels_override_send(the_connection.target_system, the_connection.target_component, 0, 0, 1300, 0, 1600, 0, 0, 0)
#msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
#print(msg)