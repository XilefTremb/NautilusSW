# ros_node.py

import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Float32MultiArray

from config import TOPICS


class NautilusROSNode(Node):

    def __init__(self, data_manager):
        super().__init__("nautilus_hmi")

        self.data = data_manager

        # SENSORS
        self.create_subscription(Imu, TOPICS["imu"], self.imu_callback, 10)
        self.create_subscription(Vector3Stamped, TOPICS["dvl"], self.dvl_callback, 10)

        self.create_subscription(
            Float32, TOPICS["depth"],
            lambda msg: self.data.update("Depth", msg.data), 10
        )

        self.create_subscription(
            Float32, TOPICS["battery_voltage"],
            lambda msg: self.data.update("Battery Voltage", msg.data), 10
        )

        self.create_subscription(
            Float32, TOPICS["battery_current"],
            lambda msg: self.data.update("Battery Current", msg.data), 10
        )

        # MOTORS
        self.create_subscription(Float32MultiArray, TOPICS["motor_pwm"], self.pwm_callback, 10)
        self.create_subscription(Float32MultiArray, TOPICS["motor_current"], self.current_callback, 10)

        self.get_logger().info("Nautilus HMI ROS node started.")

    def imu_callback(self, msg):
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))

        sinp = 2 * (w*y - z*x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))

        self.data.update("IMU Roll", math.degrees(roll))
        self.data.update("IMU Pitch", math.degrees(pitch))
        self.data.update("IMU Yaw", math.degrees(yaw))

        self.data.update("IMU Accel X", msg.linear_acceleration.x)
        self.data.update("IMU Accel Y", msg.linear_acceleration.y)
        self.data.update("IMU Accel Z", msg.linear_acceleration.z)

    def dvl_callback(self, msg):
        self.data.update("DVL X", msg.vector.x)
        self.data.update("DVL Y", msg.vector.y)
        self.data.update("DVL Z", msg.vector.z)

    def pwm_callback(self, msg):
        for i, value in enumerate(msg.data[:8]):
            self.data.update(f"Motor {i+1} PWM", value)

    def current_callback(self, msg):
        for i, value in enumerate(msg.data[:8]):
            self.data.update(f"Motor {i+1} Current", value)