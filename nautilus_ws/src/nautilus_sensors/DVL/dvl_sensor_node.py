#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistWithCovarianceStamped
import socket
import math
from nautilus_controls.auv_pymavlink import AuvPymavlink
import time

# ===== CONFIGURATION =====
DVL_IP = "192.168.1.3"      # DVL IP
DVL_PORT = 50000             # DVL port to send commands to
VM_IP = "192.168.2.10"     # VM IP on DVL subnet
LOCAL_PORT = 27000           # Port to listen for UDP packets
PUBLISH_HZ = 20              # Publishing frequency (Hz)


class DVLSensor(Node):
    def __init__(self):
        super().__init__("dvl_sensor_node")

        # Publisher
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, "/dvl/twist", 10)
        # self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.timer_callback)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        actual_buf = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        self.get_logger().info(f"UDP SO_RCVBUF actual value: {actual_buf}")
        self.sock.settimeout(2)

        self.dvl = AuvPymavlink(self)
        self.dvl.connect("udpin:localhost:14552",False)
        self.get_logger().info('Real DVL started')

        self.last_dvl_time = None

        try:
            # Bind to all interfaces (0.0.0.0) on LOCAL_PORT
            self.sock.bind(("0.0.0.0", LOCAL_PORT))
            self.get_logger().info(f"UDP socket bound to 0.0.0.0:{LOCAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP socket: {e}")
            raise e
        
        try:
            self.sock.sendto("SEND-DVPDL ON\r".encode(), (DVL_IP, DVL_PORT))
            self.get_logger().info(f"Sent DVPDL ON command to DVL {DVL_IP}:{DVL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send DVPDL ON command: {e}")

        try:
            self.sock.sendto("SEND-DVEXT OFF\r".encode(), (DVL_IP, DVL_PORT))
            self.get_logger().info(f"Sent DVEXT OFF command to DVL {DVL_IP}:{DVL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send DVEXT OFF command: {e}")

        try:
            self.sock.sendto("SEND-FREEFORM OFF\r".encode(), (DVL_IP, DVL_PORT))
            self.get_logger().info(f"Sent FREEFORM ON command to DVL {DVL_IP}:{DVL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send FREEFORM ON command: {e}")

        try:
            self.sock.sendto("MANUAL-MODE 0.001,5.0,0.5,56,0.1,50,20.6,-0.671,100,100\r".encode(), (DVL_IP, DVL_PORT))
            self.get_logger().info(f"Sent MANUAL MODE command to DVL {DVL_IP}:{DVL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send MANUAL MODE command: {e}")

        self.loop()

    def loop(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
                msg_str = data.decode("utf-8").strip()
                # self.get_logger().info("Running loop...")
                if msg_str.startswith("$DVPDL"):
                    # self.get_logger().info("Received DVPDL...")
                    self.parse_dvpdl(msg_str)
                elif msg_str.startswith("$DVTXT"):
                    self.parse_dvtxt(msg_str)
                elif msg_str.startswith("$DVEXT"):
                    self.parse_dvext(msg_str)
        
            except socket.timeout:
                self.get_logger().warn("No UDP data received")
            except Exception as e:
                self.get_logger().error(f"Error receiving UDP: {e}")

    def parse_dvext(self, msg: str) -> str:
        """parsing of $DVEXT message"""

        try:
            fields = msg.split(",")
            lock = fields[1] == "T"
            roll = float(fields[4])
            pitch = float(fields[5])
            heading = float(fields[6])
            theta = math.radians(360 - heading)
            vx = float(fields[10]) * math.cos(theta) + float(fields[11]) * math.sin(theta)
            vy = -float(fields[10]) * math.sin(theta) + float(fields[11]) * math.cos(theta)
            
            self.SendYAW(heading)

        except Exception as e:
            return f"Parse error: {e}"
        
    def parse_dvpdl(self, msg: str) -> str:
        """Parsing of $DVPDL message and publishing instantaneous DVL speed."""

        try:
            fields = msg.split(",")


            t_usec = float(fields[1])
            dt_usec = float(fields[2])
            dt_s = dt_usec / 1e6
            droll = float(fields[3])
            dpitch = float(fields[4])
            dyaw = float(fields[5])
            dx = float(fields[6])
            dy = float(fields[7])
            dz = float(fields[8])
            confidence = float(fields[9].split('*')[0])

            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = "base_link"

            twist_msg.twist.twist.linear.x = dx / dt_s
            twist_msg.twist.twist.linear.y = dy / dt_s
            twist_msg.twist.twist.linear.z = dz / dt_s

            twist_msg.twist.twist.angular.x = droll / dt_s
            twist_msg.twist.twist.angular.y = dpitch / dt_s
            twist_msg.twist.twist.angular.z = dyaw / dt_s

            twist_msg.twist.covariance[0] = 0.05
            twist_msg.twist.covariance[7] = 0.05

            self.dvl_pub.publish(twist_msg)

            self.SendDVLAsGps(t_usec, dt_usec, droll, dpitch, dyaw, dx, dy, dz, confidence) #TODO : verify send dvl as gps takes usecs

            self.get_logger().info(f"Sent DVL data t:{t_usec}, dt:{dt_usec}, dx:{dx}, dy:{dy}, dz:{dz}, confidence:{confidence}")

        except Exception as e:
            return f"Parse error: {e}"
        
        
    def parse_dvtxt(self, msg: str) -> str:
        """parsing of $DVTXT message"""

        try:
            self.get_logger().info(msg)
        except Exception as e:
            return f"Parse error: {e}"
        
    def SendDVLAsGps(self, t, dt, droll, dpitch, dyaw, dx, dy, dz, confidence=80.0):
        t = float(t)
        dt = float(dt)
        droll = float(droll)
        dpitch = float(dpitch)
        dyaw = float(dyaw)
        dx = float(dx)
        dy = float(dy)
        dz = float(dz)
        confidence = float(confidence)

        time_usec = int(t)
        time_delta_usec = int(dt)

        angle_delta = [droll, dpitch, dyaw]
        position_delta = [dx, dy, dz]

        self.dvl.the_connection.mav.vision_position_delta_send(
            time_usec,
            time_delta_usec,
            angle_delta,
            position_delta,
            confidence,
        )
    
        # self.get_logger().info(f"Sent DVL data t:{time_usec}, dt:{time_delta_usec}, dx:{dx}, dy:{dy}, dz:{dz}")

    def SendYAW(self, heading):
        self.dvl.the_connection.mav.gps_input_send(
            time.time(),
            0,
            0b11111111,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            int(heading * 100)
        )


def main(args=None):
    rclpy.init(args=args)
    node = DVLSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

