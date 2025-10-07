# !/usr/bin/env python3

import select, threading, time
from typing import Optional
from rclpy.node import Node
import rclpy

from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from pymavlink import mavutil

def quat_from_euler(roll, pitch, yaw):
    # simple euler->quat (R->XYZ convention as ROS expects)
    import math
    cy, sy = math.cos(yaw*0.5), math.sin(yaw*0.5)
    cp, sp = math.cos(pitch*0.5), math.sin(pitch*0.5)
    cr, sr = math.cos(roll*0.5), math.sin(roll*0.5)
    return Quaternion(
        x = sr*cp*cy - cr*sp*sy,
        y = cr*sp*cy + sr*cp*sy,
        z = cr*cp*sy - sr*sp*cy,
        w = cr*cp*cy + sr*sp*sy,
    )

class MavlinkTelemetryBridge(Node):
    def __init__(self):
        super().__init__('mavlink_telemetry_bridge')

        # Parameters
        self.declare_parameter('connection', 'udp:127.0.0.1:14550')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('thruster_count', 8)

        conn_str = self.get_parameter('connection').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.thruster_count = self.get_parameter('thruster_count').get_parameter_value().integer_value

        # Publishers
        self.pub_pwm   = self.create_publisher(Int32MultiArray, 'auv/thrusters/pwm', 10)
        self.pub_depth = self.create_publisher(Float32,        'auv/depth',          10)
        self.pub_odom  = self.create_publisher(Odometry,       'odom',               10)

        # Connect MAVLink
        self.get_logger().info(f'Connecting to MAVLink at {conn_str} …')
        self.mav = mavutil.mavlink_connection(conn_str)
        self.mav.wait_heartbeat()
        self.get_logger().info(f'Heartbeat from system {self.mav.target_system} component {self.mav.target_component}')

        # Reader thread
        self._stop = False
        self._t = threading.Thread(target=self._reader_loop, daemon=True)
        self._t.start()

        # Optional: health timer (warn if no data)
        self.last_msg_time = time.time()
        self.create_timer(2.0, self._watchdog)

    def _watchdog(self):
        if time.time() - self.last_msg_time > 5.0:
            self.get_logger().warn('No MAVLink messages in >5s (check SITL connection)')

    def destroy_node(self):
        self._stop = True
        try:
            if self.mav and self.mav.fd:
                self.mav.close()
        except Exception:
            pass
        super().destroy_node()

    def _reader_loop(self):
        # Request streams (helps some SITL builds)
        try:
            self.mav.mav.request_data_stream_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        except Exception:
            pass

        while not self._stop:
            r, _, _ = select.select([self.mav.fd], [], [], 0.1)
            if not r:
                continue
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                continue
            self.last_msg_time = time.time()
            mtype = msg.get_type()

            if mtype == 'SERVO_OUTPUT_RAW':
                self._handle_servo_output(msg)
            elif mtype == 'VFR_HUD':
                # ArduSub uses VFR_HUD.alt to report DEPTH (m, +down)
                self._handle_depth(msg)
            elif mtype == 'LOCAL_POSITION_NED':
                self._handle_local_position(msg)
            elif mtype == 'ATTITUDE':
                # Save attitude for odom orientation if available
                self._last_att = msg

    def _handle_servo_output(self, m):
        # m.servo1_raw .. servo8_raw (µs), more exist if >8 channels
        vals = []
        for i in range(1, self.thruster_count + 1):
            field = f'servo{i}_raw'
            vals.append(getattr(m, field, 1500) or 1500)

        out = Int32MultiArray()
        out.data = vals
        self.pub_pwm.publish(out)

    def _handle_depth(self, m):
        depth = float(m.alt)  # meters, positive down in ArduSub
        self.pub_depth.publish(Float32(data=depth))

    def _handle_local_position(self, m):
        # NED: x North, y East, z Down (m)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(m.x)     # N
        odom.pose.pose.position.y = float(m.y)     # E
        odom.pose.pose.position.z = -float(m.z)    # up in ROS (+up), so invert

        # Orientation from last ATTITUDE if present
        att = getattr(self, '_last_att', None)
        if att:
            q = quat_from_euler(att.roll, att.pitch, att.yaw)
            odom.pose.pose.orientation = q

        # Optionally fill linear velocities (NED → ROS)
        odom.twist.twist.linear.x = float(m.vx)
        odom.twist.twist.linear.y = float(m.vy)
        odom.twist.twist.linear.z = -float(m.vz)

        self.pub_odom.publish(odom)

def main():
    rclpy.init()
    node = MavlinkTelemetryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
