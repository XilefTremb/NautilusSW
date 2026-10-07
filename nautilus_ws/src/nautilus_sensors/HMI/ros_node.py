# ros_node.py

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

from config import TOPICS


class NautilusROSNode(Node):

    def __init__(self, data_manager):
        super().__init__("nautilus_hmi")

        self.data = data_manager

        # =========================
        # IMU
        # =========================

        self.create_subscription(
            Imu,
            TOPICS["imu"],
            self.imu_callback,
            10
        )

        # =========================
        # DVL
        # =========================

        self.create_subscription(
            Vector3Stamped,
            TOPICS["dvl"],
            self.dvl_callback,
            10
        )

        # =========================
        # DEPTH
        # =========================

        self.create_subscription(
            Float32,
            TOPICS["depth"],
            lambda msg: self.data.update(
                "Depth",
                msg.data
            ),
            10
        )

        # =========================
        # BATTERY
        # =========================

        self.create_subscription(
            Float32,
            TOPICS["battery_voltage"],
            lambda msg: self.data.update(
                "Battery Voltage",
                msg.data
            ),
            10
        )

        self.create_subscription(
            Float32,
            TOPICS["battery_current"],
            lambda msg: self.data.update(
                "Battery Current",
                msg.data
            ),
            10
        )

        # =========================
        # MOTORS
        # =========================

        self.create_subscription(
            Float32MultiArray,
            TOPICS["motor_pwm"],
            self.pwm_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            TOPICS["motor_current"],
            self.current_callback,
            10
        )

        self.get_logger().info(
            "Nautilus HMI ROS node started."
        )

    # ==========================================================
    # IMU
    # ==========================================================

    def imu_callback(self, msg):

        # Quaternion -> Euler

        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        import math

        # Roll
        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2 * (x*x + y*y)

        roll = math.atan2(
            sinr_cosp,
            cosr_cosp
        )

        # Pitch
        sinp = 2 * (w*y - z*x)

        if abs(sinp) >= 1:
            pitch = math.copysign(
                math.pi / 2,
                sinp
            )
        else:
            pitch = math.asin(sinp)

        # Yaw
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.data.update(
            "IMU Roll",
            math.degrees(roll)
        )

        self.data.update(
            "IMU Pitch",
            math.degrees(pitch)
        )

        self.data.update(
            "IMU Yaw",
            math.degrees(yaw)
        )

        self.data.update(
            "IMU Accel X",
            msg.linear_acceleration.x
        )

        self.data.update(
            "IMU Accel Y",
            msg.linear_acceleration.y
        )

        self.data.update(
            "IMU Accel Z",
            msg.linear_acceleration.z
        )

    # ==========================================================
    # DVL
    # ==========================================================

    def dvl_callback(self, msg):

        self.data.update(
            "DVL X",
            msg.vector.x
        )

        self.data.update(
            "DVL Y",
            msg.vector.y
        )

        self.data.update(
            "DVL Z",
            msg.vector.z
        )

    # ==========================================================
    # MOTORS
    # ==========================================================

    def pwm_callback(self, msg):

        for i, value in enumerate(msg.data[:8]):

            self.data.update(
                f"Motor {i+1} PWM",
                value
            )

    def current_callback(self, msg):

        for i, value in enumerate(msg.data[:8]):

            self.data.update(
                f"Motor {i+1} Current",
                value
            )