#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import math

# ===== CONFIGURATION =====
DVL_IP = "192.168.2.3"      # DVL IP
DVL_PORT = 50000             # DVL port to send commands to
VM_IP = "192.168.2.100"     # VM IP on DVL subnet
LOCAL_PORT = 27000           # Port to listen for UDP packets
PUBLISH_HZ = 20              # Publishing frequency (Hz)
STREAM_CMD = "SET OUTPUT UDP {} {} ON\r".format(VM_IP, LOCAL_PORT)


class DVLSensor(Node):
    def __init__(self):
        super().__init__("dvl_sensor_node")

        # Publisher
        self.publisher = self.create_publisher(String, "dvl_pub", 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.timer_callback)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)

        try:
            # Bind to all interfaces (0.0.0.0) on LOCAL_PORT
            self.sock.bind(("0.0.0.0", LOCAL_PORT))
            self.get_logger().info(f"UDP socket bound to 0.0.0.0:{LOCAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP socket: {e}")
            raise e

        # Send initial streaming command
        try:
            self.sock.sendto(STREAM_CMD.encode(), (DVL_IP, DVL_PORT))
            self.get_logger().info(f"Sent streaming command to DVL {DVL_IP}:{DVL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send streaming command: {e}")

    def timer_callback(self):
        try:
            data, addr = self.sock.recvfrom(2048)
            msg_str = data.decode("utf-8").strip()
            parsed = self.parse_dvext(msg_str)
            self.publisher.publish(String(data=parsed))
            self.get_logger().info(f"Published: {parsed}")
        except socket.timeout:
            self.get_logger().warn("No UDP data received")
        except Exception as e:
            self.get_logger().error(f"Error receiving UDP: {e}")

    def parse_dvext(self, msg: str) -> str:
        """Minimal parsing of $DVEXT message"""
        if not msg.startswith("$DVEXT"):
            return "Invalid message"

        try:
            fields = msg.split(",")
            lock = fields[1] == "T"
            roll = float(fields[4])
            pitch = float(fields[5])
            heading = float(fields[6])
            theta = math.radians(360 - heading)
            vx = float(fields[10]) * math.cos(theta) + float(fields[11]) * math.sin(theta)
            vy = -float(fields[10]) * math.sin(theta) + float(fields[11]) * math.cos(theta)
            return f"Lock:{lock} Roll:{roll:.2f} Pitch:{pitch:.2f} Heading:{heading:.2f} Vx:{vx:.2f} Vy:{vy:.2f}"
        except Exception as e:
            return f"Parse error: {e}"


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

