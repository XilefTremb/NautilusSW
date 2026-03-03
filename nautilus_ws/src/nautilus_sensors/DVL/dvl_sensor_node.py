#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import math
from nautilus_mission.auv_pymavlink import AuvPymavlink

# ===== CONFIGURATION =====
DVL_IP = "192.168.2.3"      # DVL IP
DVL_PORT = 50000             # DVL port to send commands to
VM_IP = "192.168.2.10"     # VM IP on DVL subnet
LOCAL_PORT = 27000           # Port to listen for UDP packets
PUBLISH_HZ = 20              # Publishing frequency (Hz)
STREAM_CMD = "SET OUTPUT UDP {} {} ON\r".format(VM_IP, LOCAL_PORT)


class DVLSensor(Node):
    def __init__(self):
        super().__init__("dvl_sensor_node")

        # Publisher
        # self.publisher = self.create_publisher(String, "dvl_pub", 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.timer_callback)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)

        self.dvl = AuvPymavlink(self)
        self.dvl.Connect("udpin:localhost:14552",False)
        self.get_logger().info('Real DVL started')

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

        try:
            self.sock.sendto("SEND-DVPDL ON\r".encode(), (DVL_IP, DVL_PORT))
            self.get_logger().info(f"Sent DVPDL ON command to DVL {DVL_IP}:{DVL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send streaming command: {e}")

        try:
            self.sock.sendto("SEND-DVEXT OFF\r".encode(), (DVL_IP, DVL_PORT))
            self.get_logger().info(f"Sent DVEXT OFF command to DVL {DVL_IP}:{DVL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send streaming command: {e}")

    def timer_callback(self):
        try:
            data, addr = self.sock.recvfrom(2048)
            msg_str = data.decode("utf-8").strip()
            # self.get_logger().info(msg_str)
            parsed = self.parse_dvpdl(msg_str)
            # self.publisher.publish(String(data=parsed))
            # self.get_logger().info(f"Published: {parsed}")
        except socket.timeout:
            self.get_logger().warn("No UDP data received")
        except Exception as e:
            self.get_logger().error(f"Error receiving UDP: {e}")

    def parse_dvext(self, msg: str) -> str:
        """Minimal parsing of $DVEXT message"""
        if not msg.startswith("$DVEXT"):
            self.get_logger().info("Invalid message")

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
        
    def parse_dvpdl(self, msg: str) -> str:
        """Minimal parsing of $DVPDL message"""
        if not msg.startswith("$DVPDL"):
            self.get_logger().info("Invalid message")

        try:
            fields = msg.split(",")
            t = fields[1]
            dt = fields[2]
            droll = fields[3]
            dpitch = fields[4]
            dyaw = fields[5]
            dx = fields[6]
            dy = fields[7]
            dz = fields[8]
            confidence = fields[9].split('*')[0]
            
            self.SendDVLAsGps(t, dt, dx, dy, dz, confidence)
            #self.get_logger().info(f"{dt} {dx} {dy} {dz} {droll} {dpitch} {dyaw}")
            

        except Exception as e:
            return f"Parse error: {e}"
        
    def SendDVLAsGps(self, t, dt, dx, dy, dz, confidence=80.0):
        t = float(t)
        dt = float(dt)
        dx = float(dx)
        dy = float(dy)
        dz = float(dz)
        confidence = float(confidence)

        time_usec = int(t * 1e6)
        time_delta_usec = int(dt * 1e6)

        angle_delta = [0.0, 0.0, 0.0]
        position_delta = [dx, dy, dz]

        self.dvl.the_connection.mav.vision_position_delta_send(
            time_usec,
            time_delta_usec,
            angle_delta,
            position_delta,
            confidence,
        )
    
        self.get_logger().info(f"Sent DVL data t:{time_usec}, dt:{time_delta_usec}, dx:{dx}, dy:{dy}, dz:{dz}")


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

