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

    def __init__(self):
        self.pos_mask = int(0b100111111000)
        self.vel_mask = int(0b110111000111)
        self.vel_pos_mask = int(0b110111000000)
        self.ingore_all = int(0b111111111111)

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

    def Connect(self, endpoint: str = "udpin:localhost:14551", start_receiver: bool = True):
        self.the_connection = mavutil.mavlink_connection(endpoint)
        self.the_connection.wait_heartbeat()
        print(
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
        self._rx_thread.start()

    def StopReceiver(self, join_timeout: float = 1.0):
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=join_timeout)

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

    def SetParamAndConfirm(self, name: str, value: float, timeout_s: float = 2.0) -> bool:
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
            msg = self.the_connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            d = msg.to_dict()
            pid = d.get("param_id", "").strip("\x00")
            if pid == name:
                got = d.get("param_value")
                print(f"[PARAM] {name} -> {got}")
                return True

        print(f"[PARAM] Timeout waiting confirm for {name}")
        return False

    def ApplyParamProfile(self, profile: dict, timeout_s_each: float = 2.0) -> bool:
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
        print("Waiting for motors to be armed")
        self.the_connection.motors_armed_wait()
        print("Armed!")

    def ChangeMode(self, mode: str, timeout_s: float = 3.0):
        # Check if mode is available
        if mode not in self.the_connection.mode_mapping():
            print("Unknown mode : {}".format(mode))
            print("Try:", list(self.the_connection.mode_mapping().keys()))
            sys.exit(1)

        # Set new mode via MAV_CMD_DO_SET_MODE (same as your original)
        CHGMODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        with self._send_lock:
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

        # If RX thread is running, wait using the ACK cache.
        # If RX thread is NOT running (startup), we can safely recv_match() here.
        if self._rx_thread and self._rx_thread.is_alive():
            ack = self.WaitForCommandAck(CHGMODE, timeout_s=timeout_s)
            if ack is None:
                print("[MODE] Timeout waiting for COMMAND_ACK")
                return False
            print(mavutil.mavlink.enums["MAV_RESULT"][ack["result"]].description)
            return True

        # Startup/no RX thread case:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            ack_msg = self.the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack_msg is None:
                continue
            ack_msg = ack_msg.to_dict()
            if ack_msg["command"] != CHGMODE:
                continue
            print(mavutil.mavlink.enums["MAV_RESULT"][ack_msg["result"]].description)
            return True

        print("[MODE] Timeout waiting for COMMAND_ACK")
        return False

    def SendPosLocal(self, north, east, down, yaw):
        with self._send_lock:
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
                    pi / 2,
                )
            )

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

    # Kept for compatibility, but now uses cache if receiver is running.
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
                print(f"Dist={dist:.2f}m Speed={speed:.2f}m/s")

                if ok:
                    if stable_since is None:
                        stable_since = time.time()
                    elif (time.time() - stable_since) >= settle_time:
                        print("Arrived at destination!")
                        return
                else:
                    stable_since = None

            time.sleep(dt)

    # -------------------- DVL / VISION_POSITION_DELTA (kept) --------------------

    def SendDVLAsGps(self, dx, dy, dz, confidence=100.0):
        """
        dx, dy, dz: position increments (meters) for VISION_POSITION_DELTA
        confidence: 0..100
        """
        now = time.time()
        dt = now - self.last_t
        self.last_t = now

        time_usec = int(now * 1e6)
        time_delta_usec = int(dt * 1e6)

        angle_delta = [0.0, 0.0, 0.0]  # rad
        position_delta = [dx, dy, dz]  # m

        with self._send_lock:
            self.the_connection.mav.vision_position_delta_send(
                time_usec,
                time_delta_usec,
                angle_delta,
                position_delta,
                float(confidence),
            )

        print(f"Sending DVL estimated pos [{dx}, {dy}, {dz}] to VISION_POSITION_DELTA")

    def StartDvlThread(self, dvl_delta_fn, rate_hz: float = 10.0, confidence: float = 100.0, name="dvl-tx"):
        """
        Starts a daemon thread that calls dvl_delta_fn() -> (dx,dy,dz) and sends VISION_POSITION_DELTA.
        Returns: (stop_event, thread)
        """
        period = 1.0 / float(rate_hz)
        stop_evt = threading.Event()

        def _loop():
            next_t = time.time()
            while not stop_evt.is_set():
                dx, dy, dz = dvl_delta_fn()
                self.SendDVLAsGps(dx, dy, dz, confidence=confidence)

                next_t += period
                sleep = next_t - time.time()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.time()

        th = threading.Thread(target=_loop, name=name, daemon=True)
        th.start()
        return stop_evt, th

    def CheckDialectAndMethodAvailability(self, method):
        print("dialect:", mavutil.mavlink.WIRE_PROTOCOL_VERSION if hasattr(mavutil.mavlink, 'WIRE_PROTOCOL_VERSION') else "unknown")
        print("has vision_position_delta_send:", hasattr(self.the_connection.mav, method))
