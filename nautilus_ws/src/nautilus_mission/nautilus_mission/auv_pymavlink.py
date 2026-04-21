from pymavlink import mavutil
from math import pi
import math
import time
import sys
import os
import threading
from collections import deque


class AuvPymavlink:
    """
    One MAVLink connection, multi-threaded use.

    Rules:
      - ONLY ONE thread may call recv_match() on this connection at a time.
      - Multiple threads may send if protected by a lock.

    This implementation supports a clean startup sequence:
      1) Connect(start_receiver=False)
      2) Set params (uses recv_match() to confirm PARAM_VALUE)
      3) StartReceiver() (from now on, RX thread is the only recv_match() consumer)
      4) Run your control threads (SendPosLocal, SendDVLAsGps, etc.)
    """

    def __init__(self, node):

        self.node = node

        self.pos_mask = int(0b100111111000)
        self.pos_mask_no_yaw = int(0b110111111000)
        self.vel_mask = int(0b100111000111)
        self.ingore_all = int(0b111111111111)

        self.reset_counter = 0


        # Parameter profiles (minimal, extend as needed)
        self.SITL_PROFILE = {
            "VISO_TYPE": 0,
            "EK3_SRC1_POSXY": 3,   # GPS
            "EK3_SRC1_VELXY": 3,   # GPS
            "EK3_SRC1_POSZ": 3,    # GPS
        }

        self.AUV_PROFILE = {
            "VISO_TYPE": 1,        # MAVLink vision/odometry (DVL integration)
            "EK3_SRC1_POSXY": 6,   # ExternalNav
            "EK3_SRC1_VELXY": 6,   # ExternalNav
            "EK3_SRC1_POSZ": 1,    # Baro
            "EK3_SRC1_VELZ": 0,    # None
            "EK3_SRC1_YAW": 1,     # Compass
            "EK3_ENABLE" : 1,
            "AHRS_EKF_TYPE" : 3,
            "EK2_ENABLE" : 0,
                # ExternalNav (to be changed back to 1 when ran on the real vehicle)
        }


        self.the_connection = None
        self.last_t = time.time()

        os.environ["MAVLINK20"] = "1"
        os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

        # TX protection
        self._send_lock = threading.Lock()

        # RX thread + caches
        self._rx_thread = None
        self._rx_stop = threading.Event()
        self._state_lock = threading.Lock()

        self._latest_local_pos_ned = None   # LOCAL_POSITION_NED
        self._latest_attitude = None        # ATTITUDE
        self._latest_heartbeat = None       # HEARTBEAT

        # ACK support (e.g., ChangeMode)
        self._ack_cv = threading.Condition()
        self._last_ack_by_command = {}  # command_id -> ack_dict

        # debug ring buffer
        self._last_msgs = deque(maxlen=200)

    # -------------------- Connection --------------------

    def Connect(self, endpoint: str = "udpin:localhost:14550", start_receiver: bool = True):
        self.the_connection = mavutil.mavlink_connection(endpoint)
        self.the_connection.wait_heartbeat()
        self.node.get_logger().info(
            f"Heartbeat from system: system {self.the_connection.target_system}  and component {self.the_connection.target_component}"
        )
        if start_receiver:
            self.StartReceiver()

    # -------------------- Receiver thread --------------------

    def StartReceiver(self):
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="mavlink-rx", daemon=True)
        self.node.get_logger().info("Started receiver thread...")
        self._rx_thread.start()

    def StopReceiver(self, join_timeout: float = 1.0):
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=join_timeout)
        self.node.get_logger().info("Stopped receiver thread...")

    def _rx_loop(self):
        while not self._rx_stop.is_set():
            try:
                msg = self.the_connection.recv_match(blocking=True, timeout=0.5)
                
                if msg is None:
                    continue

                mtype = msg.get_type()

                with self._state_lock:
                    self._last_msgs.append(msg)
                    if mtype == "LOCAL_POSITION_NED":
                        self._latest_local_pos_ned = msg
                    elif mtype == "ATTITUDE":
                        self._latest_attitude = msg
                    elif mtype == "HEARTBEAT":
                        self._latest_heartbeat = msg

                if mtype == "COMMAND_ACK":
                    ack = msg.to_dict()
                    cmd = ack.get("command")
                    if cmd is not None:
                        with self._ack_cv:
                            self._last_ack_by_command[cmd] = ack
                            self._ack_cv.notify_all()

            except Exception as e:
                print(f"[RX] Exception: {e}")

    def GetLocalPosNedCached(self):
        """Non-blocking: returns latest cached LOCAL_POSITION_NED (or None)."""
        with self._state_lock:
            return self._latest_local_pos_ned

    def GetAttitudeCached(self):
        """Non-blocking: returns latest cached ATTITUDE (or None)."""
        with self._state_lock:
            return self._latest_attitude
        
    def GetLocalPosNed(self):
        """
        If RX thread is running: returns cached LOCAL_POSITION_NED (non-blocking).
        If RX thread is not running: blocks on recv_match() like your original.
        """
        if self._rx_thread and self._rx_thread.is_alive():
            msg = self.GetLocalPosNedCached()
            if msg is not None:
                return msg
            return None

        msg = self.the_connection.recv_match(type="LOCAL_POSITION_NED", blocking=True)
        return msg  # has x,y,z,vx,vy,vz

    def GetAttitude(self):
        """
        If RX thread is running: returns cached ATTITUDE (non-blocking).
        If RX thread is not running: blocks on recv_match() like your original.
        """
        if self._rx_thread and self._rx_thread.is_alive():
            msg = self.GetAttitudeCached()
            if msg is not None:
                return msg
            return None

        msg = self.the_connection.recv_match(type="ATTITUDE", blocking=True)
        return msg 

    def WaitForCommandAck(self, command_id: int, timeout_s: float = 3.0):
        deadline = time.time() + timeout_s
        with self._ack_cv:
            while time.time() < deadline:
                ack = self._last_ack_by_command.get(command_id)
                if ack is not None:
                    return ack
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._ack_cv.wait(timeout=min(remaining, 0.5))
        return None

    # -------------------- Params (startup only) --------------------
    # IMPORTANT: call these BEFORE StartReceiver(), because they use recv_match().

    def SetParamAndConfirm(self, name: str, value: float, timeout_s: float = 5.0) -> bool:
        if self.the_connection is None:
            raise RuntimeError("Not connected")

        param_id = name.encode("ascii")

        with self._send_lock:
            self.the_connection.mav.param_set_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                param_id,
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.the_connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
            if msg is None:
                continue
            d = msg.to_dict()
            pid = d.get("param_id", "").strip("\x00")
            if pid == name:
                got = d.get("param_value")
                self.node.get_logger().info(f"[PARAM] {name} -> {got}")
                return True

        self.node.get_logger().info(f"[PARAM] Timeout waiting confirm for {name}")
        return False

    def ApplyParamProfile(self, profile: dict, timeout_s_each: float = 5.0) -> bool:
        ok_all = True
        for k, v in profile.items():
            ok = self.SetParamAndConfirm(k, v, timeout_s=timeout_s_each)
            ok_all = ok_all and ok
        return ok_all

    # -------------------- Original high-level commands (kept) --------------------

    def Arm(self):
        ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        with self._send_lock:
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
        self.node.get_logger().info("Waiting for motors to be armed")
        self.the_connection.motors_armed_wait()
        self.node.get_logger().info("Armed!")

    def Disarm(self):
        ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        with self._send_lock:
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                ARM,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        self.node.get_logger().info("Waiting for motors to be disarmed")
        self.the_connection.motors_disarmed_wait()
        self.node.get_logger().info("Disarmed!")

    def ChangeMode(self, mode: str, timeout_s: float = 3.0):
        mode_map = self.the_connection.mode_mapping()

        # Check if mode is available
        if mode not in mode_map:
            self.node.get_logger().info(f"Unknown mode: {mode}")
            self.node.get_logger().info(f"Try: {list(mode_map.keys())}")
            return False

        mode_id = mode_map[mode]

        # Clear any stale ACK for this command before sending
        CHGMODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        with self._ack_cv:
            self._last_ack_by_command.pop(CHGMODE, None)

        # Send requested mode
        with self._send_lock:
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                CHGMODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
                0,
                0,
                0,
                0,
                0,
            )

        self.node.get_logger().info(f"Mode change requested: {mode} (custom_mode={mode_id})")

        # If RX thread is running, wait using ACK cache
        if self._rx_thread and self._rx_thread.is_alive():
            ack = self.WaitForCommandAck(CHGMODE, timeout_s=timeout_s)
            if ack is None:
                self.node.get_logger().info("[MODE] Timeout waiting for COMMAND_ACK")
                return False

            result_desc = mavutil.mavlink.enums["MAV_RESULT"][ack["result"]].description
            self.node.get_logger().info(f"[MODE] ACK: {result_desc}")

            return ack["result"] == mavutil.mavlink.MAV_RESULT_ACCEPTED

        # Startup / no RX thread case
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            ack_msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack_msg is None:
                continue

            ack_msg = ack_msg.to_dict()
            if ack_msg.get("command") != CHGMODE:
                continue

            result_desc = mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description
            self.node.get_logger().info(f"[MODE] ACK: {result_desc}")

            return ack_msg["result"] == mavutil.mavlink.MAV_RESULT_ACCEPTED

        self.node.get_logger().info("[MODE] Timeout waiting for COMMAND_ACK")
        return False

    def SendPosLocal(self, north, east, down, yaw):
        with self._send_lock:
            self.the_connection.mav.set_position_target_local_ned_send(
                0,
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.pos_mask,
                north, east, down,
                0, 0, 0,
                0, 0, 0,
                yaw,
                pi/2
            )

    def SendPosLocalReset(self):
        self.reset_counter += 1
        if self.reset_counter > 255:
            self.reset_counter = 0
        self.node.get_logger().info(f'reset_counter : {self.reset_counter}')
        with self._send_lock:
            self.the_connection.mav.vision_position_estimate_send(
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    [math.nan,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                    self.reset_counter
                )
            
    def ResetPosEstimate(self):
        while not self.ValidateLocalNedReset():
            self.node.get_logger().info("Reset position estimate was asked...")
            self.SendPosLocalReset()
            time.sleep(0.1)
        self.node.get_logger().info("Reset position succeeded!")

    def ValidateLocalNedReset(self):
        msg = self.GetLocalPosNed()
        self.node.get_logger().info(f"Validating local position ned was reset: {msg}")

        if msg is None:
            return False

        return math.sqrt(msg.x**2 + msg.y**2) < 0.05

    def SendPosOffset(self, north, east, down, yaw):
        with self._send_lock:
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
                    pi / 2,
                )
            )
        
    def ArrivedLogic(self, currentPos, target):
        dx = target[0] - currentPos.x
        dy = target[1] - currentPos.y
        dz = target[2] - currentPos.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        speed = math.sqrt(currentPos.vx ** 2 + currentPos.vy ** 2 + currentPos.vz ** 2)
        ok = (dist < 0.1) and (speed < 0.2)
        return ok, dist, speed

    def GoToWaypointLocal(self, north, east, down, yaw):
        target = [north, east, down]
        settle_time = 0.7

        rate_hz = 10
        dt = 1.0 / rate_hz
        stable_since = None

        self.SendPosLocal(north, east, down, yaw)

        while True:
            pos = self.GetLocalPosNed()
            if pos:
                ok, dist, speed = self.ArrivedLogic(pos, target)
                self.node.get_logger().info(f"Dist={dist:.2f}m Speed={speed:.2f}m/s")

                if ok:
                    if stable_since is None:
                        stable_since = time.time()
                    elif (time.time() - stable_since) >= settle_time:
                        self.node.get_logger().info("Arrived at destination!")
                        return
                else:
                    stable_since = None

            time.sleep(dt)

    def GoToWaypointLocalFRD(self, x, y, down, yaw, yaw_offset):
        north = x * math.cos(yaw_offset) - y * math.sin(yaw_offset)
        east = x * math.sin(yaw_offset) + y * math.cos(yaw_offset)

        self.GoToWaypointLocal(north, east, down, yaw)

    def SendVecCommand(self, speed, yaw, vz = 0.0):
        vx = speed * math.cos(yaw)
        vy = speed * math.sin(yaw)
        with self._send_lock:
            self.the_connection.mav.set_position_target_local_ned_send(
                0,
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.vel_mask,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                yaw,
                0.05
            )
        self.node.get_logger().info(f"Sent vector cmd vx: {vx} vy: {vy} vz: {vz} yaw: {yaw}")

    def CheckDialectAndMethodAvailability(self, method):
        self.node.get_logger().info("dialect:", mavutil.mavlink.WIRE_PROTOCOL_VERSION if hasattr(mavutil.mavlink, 'WIRE_PROTOCOL_VERSION') else "unknown")
        self.node.get_logger().info("has vision_position_delta_send:", hasattr(self.the_connection.mav, method))

    def SendRCOverride(self, forward=None, lateral=None, throttle=None, yaw=None, pitch=None, roll=None):
   
        UINT16_MAX = 65535

        def encode_ch_1_to_8(value):
            if value is None:
                return 0      
            if not isinstance(value, int):
                raise TypeError(
                    f"Expected int or None, got {type(value).__name__}"
                )
            if not (0 <= value <= UINT16_MAX):
                raise ValueError(f"RC override value {value} out of uint16 range")
            return value

        ch1_pitch    = encode_ch_1_to_8(pitch)
        ch2_roll     = encode_ch_1_to_8(roll)
        ch3_throttle = encode_ch_1_to_8(throttle)
        ch4_yaw      = encode_ch_1_to_8(yaw)
        ch5_forward  = encode_ch_1_to_8(forward)
        ch6_lateral  = encode_ch_1_to_8(lateral)

        # Unused CH7..CH18 = ignore
        ch7 = ch8 = UINT16_MAX
        ch9 = ch10 = ch11 = ch12 = ch13 = ch14 = ch15 = ch16 = ch17 = ch18 = 0

        with self._send_lock:
            self.the_connection.mav.rc_channels_override_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                ch1_pitch,
                ch2_roll,
                ch3_throttle,
                ch4_yaw,
                ch5_forward,
                ch6_lateral,
                ch7,
                ch8,
                ch9,
                ch10,
                ch11,
                ch12,
                ch13,
                ch14,
                ch15,
                ch16,
                ch17,
                ch18,
            )
