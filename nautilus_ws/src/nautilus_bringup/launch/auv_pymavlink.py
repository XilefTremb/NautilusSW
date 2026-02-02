from pymavlink import mavutil


class AuvPymavlink:
    def __init__(self):
        self.pos_mask = int(0b100111111000)
        self.vel_mask = int(0b110111000111)
        self.vel_pos_mask = int(0b110111000000)
        self.ingore_all = int(0b111111111111)
        self.the_connection = None

    def Connect(self):
        self.the_connection = mavutil.mavlink_connection("udpin:localhost:14551")
        self.the_connection.wait_heartbeat()
        print(
            f"Heartbeat from system: system {self.the_connection.target_system}  and component {self.the_connection.target_component}"
        )

    def Arm(self):
        ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            ARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    def ModeGuided(self):
        CHGMODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system,
            self.the_connection.target_component,
            CHGMODE,
            0,
            1,
            4,
            0,
            0,
            0,
            0,
            0,
        )
        msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)

    def SendPos(self, north, east, down):
        self.the_connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0,
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.pos_mask,
                north,
                east,
                down,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        )
