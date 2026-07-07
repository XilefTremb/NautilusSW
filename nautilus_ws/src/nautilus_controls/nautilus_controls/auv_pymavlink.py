from collections import deque
from math import pi
import math
import os
import threading
import time

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "ardupilotmega"

from pymavlink import mavutil


class AuvPymavlink:
    def __init__(self, node):
        self.node = node

        self.pos_mask = int(0b100111111000)
        self.pos_mask_no_yaw = int(0b110111111000)
        self.vel_mask = int(0b100111000111)
        self.ignore_all = int(0b111111111111)
        self.depth_mask = int(0b111111111011)

        self.reset_counter = 0

        self.sitl_profile = {
            "VISO_TYPE": 0,
            "EK3_SRC1_POSXY": 3,
            "EK3_SRC1_VELXY": 3,
            "EK3_SRC1_POSZ": 3,
        }

        self.auv_profile = {
            "VISO_TYPE": 1,
            "EK3_SRC1_POSXY": 6,
            "EK3_SRC1_VELXY": 6,
            "EK3_SRC1_POSZ": 1,
            "EK3_SRC1_VELZ": 0,
            "EK3_SRC1_YAW": 1,
            "EK3_ENABLE": 1,
            "AHRS_EKF_TYPE": 3,
            "EK2_ENABLE": 0,
        }

        self.the_connection = None
        self.last_t = time.time()

        self._send_lock = threading.Lock()

        self._rx_thread = None
        self._rx_stop = threading.Event()
        self._state_lock = threading.Lock()

        self._latest_ahrs2 = None
        self._latest_attitude = None
        self._latest_heartbeat = None

        self._ack_cv = threading.Condition()
        self._last_ack_by_command = {}

        self._last_msgs = deque(maxlen=200)

    def connect(self, endpoint: str = "udpin:localhost:14550", start_receiver: bool = True):
        self.the_connection = mavutil.mavlink_connection(endpoint)
        self.the_connection.wait_heartbeat()

        self.node.get_logger().info(
            f"Heartbeat from system: system {self.the_connection.target_system} "
            f"and component {self.the_connection.target_component}"
        )

        if start_receiver:
            self.start_receiver()

    def start_receiver(self):
        if self._rx_thread and self._rx_thread.is_alive():
            return

        self._rx_stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name="mavlink-rx",
            daemon=True,
        )

        self.node.get_logger().info("Started receiver thread...")
        self._rx_thread.start()

    def stop_receiver(self, join_timeout: float = 1.0):
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

                msg_type = msg.get_type()

                with self._state_lock:
                    self._last_msgs.append(msg)

                    if msg_type == "AHRS2":
                        self._latest_ahrs2 = msg
                    elif msg_type == "ATTITUDE":
                        self._latest_attitude = msg
                    elif msg_type == "HEARTBEAT":
                        self._latest_heartbeat = msg

                if msg_type == "COMMAND_ACK":
                    ack = msg.to_dict()
                    command = ack.get("command")

                    if command is not None:
                        with self._ack_cv:
                            self._last_ack_by_command[command] = ack
                            self._ack_cv.notify_all()

            except Exception as exc:
                self.node.get_logger().error(f"[RX] Exception: {exc}")

    def get_ahrs2_cached(self):
        with self._state_lock:
            return self._latest_ahrs2

    def get_attitude_cached(self):
        with self._state_lock:
            return self._latest_attitude

    def get_ahrs2(self):
        if self._rx_thread and self._rx_thread.is_alive():
            return self.get_ahrs2_cached()

        return self.the_connection.recv_match(
            type="AHRS2",
            blocking=True,
        )

    def get_attitude(self):
        if self._rx_thread and self._rx_thread.is_alive():
            return self.get_attitude_cached()

        return self.the_connection.recv_match(
            type="ATTITUDE",
            blocking=True,
        )

    def wait_for_command_ack(self, command_id: int, timeout_s: float = 3.0):
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

    def set_param_and_confirm(self, name: str, value: float, timeout_s: float = 5.0) -> bool:
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
            msg = self.the_connection.recv_match(
                type="PARAM_VALUE",
                blocking=True,
                timeout=2.0,
            )

            if msg is None:
                continue

            data = msg.to_dict()
            received_param_id = data.get("param_id", "").strip("\x00")

            if received_param_id == name:
                received_value = data.get("param_value")
                self.node.get_logger().info(f"[PARAM] {name} -> {received_value}")
                return True

        self.node.get_logger().info(f"[PARAM] Timeout waiting confirm for {name}")
        return False

    def apply_param_profile(self, profile: dict, timeout_s_each: float = 5.0) -> bool:
        ok_all = True

        for name, value in profile.items():
            ok = self.set_param_and_confirm(name, value, timeout_s=timeout_s_each)
            ok_all = ok_all and ok

        return ok_all

    def arm(self):
        arm_command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

        with self._send_lock:
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                arm_command,
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

    def disarm(self):
        arm_command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM

        with self._send_lock:
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                arm_command,
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

    def change_mode(self, mode: str, timeout_s: float = 3.0):
        mode_map = self.the_connection.mode_mapping()

        if mode not in mode_map:
            self.node.get_logger().info(f"Unknown mode: {mode}")
            self.node.get_logger().info(f"Try: {list(mode_map.keys())}")
            return False

        mode_id = mode_map[mode]
        change_mode_command = mavutil.mavlink.MAV_CMD_DO_SET_MODE

        with self._ack_cv:
            self._last_ack_by_command.pop(change_mode_command, None)

        with self._send_lock:
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                change_mode_command,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
                0,
                0,
                0,
                0,
                0,
            )

        self.node.get_logger().info(
            f"Mode change requested: {mode} (custom_mode={mode_id})"
        )

        if self._rx_thread and self._rx_thread.is_alive():
            ack = self.wait_for_command_ack(change_mode_command, timeout_s=timeout_s)

            if ack is None:
                self.node.get_logger().info("[MODE] Timeout waiting for COMMAND_ACK")
                return False

            result_desc = mavutil.mavlink.enums["MAV_RESULT"][ack["result"]].description
            self.node.get_logger().info(f"[MODE] ACK: {result_desc}")

            return ack["result"] == mavutil.mavlink.MAV_RESULT_ACCEPTED

        deadline = time.time() + timeout_s

        while time.time() < deadline:
            ack_msg = self.the_connection.recv_match(
                type="COMMAND_ACK",
                blocking=True,
                timeout=0.5,
            )

            if ack_msg is None:
                continue

            ack_data = ack_msg.to_dict()

            if ack_data.get("command") != change_mode_command:
                continue

            result_desc = mavutil.mavlink.enums["MAV_RESULT"][ack_data["result"]].description
            self.node.get_logger().info(f"[MODE] ACK: {result_desc}")

            return ack_data["result"] == mavutil.mavlink.MAV_RESULT_ACCEPTED

        self.node.get_logger().info("[MODE] Timeout waiting for COMMAND_ACK")
        return False

    def send_pos_local(self, north, east, down, yaw):
        with self._send_lock:
            self.the_connection.mav.set_position_target_local_ned_send(
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
                pi / 2,
            )

    def send_pos_local_reset(self):
        self.reset_counter += 1

        if self.reset_counter > 255:
            self.reset_counter = 0

        self.node.get_logger().info(f"reset_counter : {self.reset_counter}")

        with self._send_lock:
            self.the_connection.mav.vision_position_estimate_send(
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                [math.nan, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                self.reset_counter,
            )

    def reset_pos_estimate(self):
        while not self.validate_local_ned_reset():
            self.node.get_logger().info("Reset position estimate was asked...")
            self.send_pos_local_reset()
            time.sleep(0.1)

        self.node.get_logger().info("Reset position succeeded!")

    def validate_local_ned_reset(self):
        msg = self.get_ahrs2()
        self.node.get_logger().info(f"Validating local position ned was reset: {msg}")

        if msg is None:
            return False

        return math.sqrt(msg.x**2 + msg.y**2) < 0.05

    def send_pos_offset(self, north, east, down, yaw):
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

    def arrived_logic(self, current_pos, target):
        dx = target[0] - current_pos.x
        dy = target[1] - current_pos.y
        dz = target[2] - current_pos.z

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = math.sqrt(
            current_pos.vx**2 + current_pos.vy**2 + current_pos.vz**2
        )

        ok = (dist < 0.1) and (speed < 0.2)

        return ok, dist, speed

    def go_to_waypoint_local(self, north, east, down, yaw):
        target = [north, east, down]
        settle_time = 0.7

        rate_hz = 10
        dt = 1.0 / rate_hz
        stable_since = None

        self.send_pos_local(north, east, down, yaw)

        while True:
            pos = self.get_ahrs2()

            if pos:
                ok, dist, speed = self.arrived_logic(pos, target)
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

    def go_to_waypoint_local_frd(self, x, y, down, yaw, yaw_offset):
        north = x * math.cos(yaw_offset) - y * math.sin(yaw_offset)
        east = x * math.sin(yaw_offset) + y * math.cos(yaw_offset)

        self.go_to_waypoint_local(north, east, down, yaw)

    def send_vec_command(self, speed, yaw, vz=0.0):
        vx = speed * math.cos(yaw)
        vy = speed * math.sin(yaw)

        with self._send_lock:
            self.the_connection.mav.set_position_target_local_ned_send(
                0,
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                self.vel_mask,
                0,
                0,
                0,
                vx,
                vy,
                vz,
                0,
                0,
                0,
                yaw,
                0.05,
            )

        self.node.get_logger().info(
            f"Sent vector cmd vx: {vx} vy: {vy} vz: {vz} yaw: {yaw}"
        )

    def check_dialect_and_method_availability(self, method):
        version = (
            mavutil.mavlink.WIRE_PROTOCOL_VERSION
            if hasattr(mavutil.mavlink, "WIRE_PROTOCOL_VERSION")
            else "unknown"
        )

        self.node.get_logger().info(f"dialect: {version}")
        self.node.get_logger().info(
            f"has {method}: {hasattr(self.the_connection.mav, method)}"
        )

    def send_rc_override(self, forward=None, lateral=None, throttle=None, yaw=None, pitch=None, roll=None):

        uint16_max = 65535

        def encode(value):
            if value is None:
                return uint16_max

            if not isinstance(value, int):
                raise TypeError(f"Expected int or None, got {type(value).__name__}")

            if not (0 <= value <= uint16_max):
                raise ValueError(f"RC override value {value} out of uint16 range")

            return value

        channels = [
        encode(pitch),     # ch1
        encode(roll),      # ch2
        encode(throttle),  # ch3
        encode(yaw),       # ch4
        encode(forward),   # ch5
        encode(lateral),   # ch6
        uint16_max,        # ch7
        uint16_max,        # ch8
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        last_error = None

        retries = 2

        for attempt in range(retries):
            try:
                with self._send_lock:
                    self._rc_override_send_compat(channels)
                return True

            except Exception as e:
                last_error = e
                self.node.get_logger().warn(
                    f"RC override send failed ({attempt + 1}/{retries}): {e}"
                )
                time.sleep(0.02)

        self.node.get_logger().error(f"RC override failed after {retries} retries: {last_error}")
        return False
        

    def _rc_override_send_compat(self, channels):
        try:
            self.the_connection.mav.rc_channels_override_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                *channels[:18],
            )
        except TypeError as e:
            if "arguments" not in str(e):
                raise

            # Older pymavlink / MAVLink1 dialect: only channels 1-8 supported
            self.the_connection.mav.rc_channels_override_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                *channels[:8],
            )

    def set_target_depth(self, depth):
        self.the_connection.mav.set_position_target_global_int_send(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            self.depth_mask,
            0, 0, depth, 
            0, 0, 0, #vx vy vz
            0, 0, 0, #ax ay az
            0, 0) #yaw yawrate
        
    def set_servo(self, servo_num, pwm):
        with self._send_lock:
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                servo_num,  # servo number (11 = AUX3)
                pwm,        # PWM value (1100, 1500, 1900, ...)
                0,
                0,
                0,
                0,
                0,
            )
    
        
    def go_to_depth(self, depth):
        depth *= -1
        z = self.get_ahrs2().altitude
        while(abs(z-depth)>0.1):
            z = self.get_ahrs2().altitude
            if abs(z-depth)>0.1:
                self.set_target_depth(depth)
            time.sleep(0.1)
            self.node.get_logger().info(f"{abs(z-depth)}")

    def set_servo(self, servo_num, pwm):
        with self._send_lock:
            self.the_connection.mav.command_long_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                servo_num,  # servo number (11 = AUX3)
                pwm,        # PWM value (1100, 1500, 1900, ...)
                0,
                0,
                0,
                0,
                0,
            )

