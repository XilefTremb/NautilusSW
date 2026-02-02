from auv_pymavlink import AuvPymavlink

AUV = AuvPymavlink()

AUV.Connect()

AUV.Arm()

AUV.ModeGuided()

AUV.SendPos(-1, -1, 1)

# msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
# print(msg)


# msg = the_connection.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=True)


# while True:
    #      msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=False)
    #      msg2 = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    #      msg3 = the_connection.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=False)
#     print("Hello!")
#      print(msg2)
#      print(msg3)
