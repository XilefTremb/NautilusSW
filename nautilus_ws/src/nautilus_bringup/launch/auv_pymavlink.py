from pymavlink import mavutil
from math import pi
import math
import time
import sys


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
        print("Waiting for motors to be armed")
        self.the_connection.motors_armed_wait()
        print("Armed!")

    def ChangeMode(self, mode):

        # Check if mode is available
        if mode not in self.the_connection.mode_mapping():
            print("Unknown mode : {}".format(mode))
            print("Try:", list(self.the_connection.mode_mapping().keys()))
            sys.exit(1)

        # Get mode ID
        mode_id = self.the_connection.mode_mapping()[mode]

        # Set new mode
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
        print(f"Mode {mode} was sent to controller!")

        while True:
            # Wait for ACK command
            # Would be good to add mechanism to avoid endlessly blocking
            # if the autopilot sends a NACK or never receives the message
            ack_msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True)
            ack_msg = ack_msg.to_dict()

            # Continue waiting if the acknowledged command is not `set_mode`
            if ack_msg["command"] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                print("Acknowledged command is not 'set_mode'")
                continue

            # Print the ACK result !
            print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
            break

    def SendPosLocal(self, north, east, down, yaw):
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
                yaw,
                pi/2,
            )
        )

    def SendPosOffset(self, north, east, down, yaw):
        self.the_connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0,
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
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
                yaw,
                pi/2,
            )
        )

    def GetLocalPosNed(self):
        msg = self.the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)
        print(msg)
        return msg  # has x,y,z,vx,vy,vz
    
    def ArrivedLogic(self, currentPos, target):
        dx = target[0] - currentPos.x
        dy = target[1] - currentPos.y
        dz = target[2] - currentPos.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        speed = math.sqrt(currentPos.vx**2 + currentPos.vy**2 + currentPos.vz**2)
        ok = (dist < 0.1) and (speed < 0.2)
        return ok, dist, speed

    def GoToWaypointLocal(self, north, east, down, yaw):

        target = [north, east, down]
        settle_time=0.7
        
        rate_hz = 10
        dt = 1.0 / rate_hz
        t0 = time.time()
        stable_since = None

        self.SendPosLocal(north, east, down, yaw)

        while True:
            
            pos = self.GetLocalPosNed()
            if pos:
                ok, dist, speed = self.ArrivedLogic(pos, target)
                print(f"Dist={dist:.2f}m Speed={speed:.2f}m/s")

                if ok:
                    if stable_since is None:
                        stable_since = time.time()
                    elif (time.time() - stable_since) >= settle_time:
                        print("Arrived at destination!")
                        return  # arrived
                else:
                    stable_since = None

            time.sleep(dt)