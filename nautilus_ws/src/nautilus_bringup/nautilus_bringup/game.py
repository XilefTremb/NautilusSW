#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from geometry_msgs.msg import PoseStamped
from evdev import InputDevice, ecodes

import threading
import math
import numpy as np


TORPEDO_BUTTON = 311   # BTN_TR
DROPPER_BUTTON = 310   # BTN_TL


# Fixed Gazebo target positions
TORPEDO_TARGET = np.array([6.93, 0.80, -1.35])
DROPPER_TARGET = np.array([6.85, 5.07, -1.85])

TORPEDO_MAX_DISTANCE = 2.0     # m
POSITION_TOLERANCE = 0.2       # m
YAW_TOLERANCE = math.radians(10.0)

def euler_from_quat(x, y, z, w):
    roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x))))
    yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))

    return roll, pitch, yaw


class GameControllerInput(Node):

    def __init__(self):
        super().__init__('game_controller_input')

        # --------------------------------------------------
        # Controller
        # --------------------------------------------------

        self.controller = InputDevice("/dev/input/event18")

        self.get_logger().info(f"Controller detected: {self.controller.name}")

        self.controller_thread = threading.Thread(
            target=self.read_controller,
            daemon=True
        )

        self.controller_thread.start()


        # --------------------------------------------------
        # Gazebo submarine pose
        # --------------------------------------------------

        self.sub_position = None
        self.sub_roll = None
        self.sub_pitch = None
        self.sub_yaw = None

        self.pose_sub = self.create_subscription(PoseStamped, '/gz/tf', self.pose_callback, 1)

        # --------------------------------------------------
        # Target positions
        # --------------------------------------------------

        self.torpedo_target = TORPEDO_TARGET
        self.dropper_target = DROPPER_TARGET

        self.get_logger().info(
            f"Torpedo target: "
            f"x={self.torpedo_target[0]}, "
            f"y={self.torpedo_target[1]}, "
            f"z={self.torpedo_target[2]}"
        )

        self.get_logger().info(
            f"Dropper target: "
            f"x={self.dropper_target[0]}, "
            f"y={self.dropper_target[1]}, "
            f"z={self.dropper_target[2]}"
        )

        # --------------------------------------------------
        # Results publisher
        # --------------------------------------------------
        self.torpedo_result_pub = self.create_publisher(
            Bool,
            '/game/torpedo_hit',
            10
        )

        self.dropper_result_pub = self.create_publisher(
            Bool,
            '/game/dropper_hit',
            10
        )


    # ------------------------------------------------------
    # Gazebo
    # ------------------------------------------------------

    def pose_callback(self, msg):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        roll, pitch, yaw = euler_from_quat(qx, qy, qz, qw)

        self.sub_position = np.array([x,y,z])

        self.sub_roll = roll
        self.sub_pitch = pitch
        self.sub_yaw = yaw


    # ------------------------------------------------------
    # Controller
    # ------------------------------------------------------

    def read_controller(self):

        for event in self.controller.read_loop():

            if event.type != ecodes.EV_KEY:
                continue

            # Only react when button is initially pressed
            if event.value != 1:
                continue

            if event.code == TORPEDO_BUTTON:
                self.torpedo_fired()

            elif event.code == DROPPER_BUTTON:
                self.dropper_released()


    # ------------------------------------------------------
    # Game events
    # ------------------------------------------------------

    def torpedo_fired(self):
        self.get_logger().info("TORPEDO FIRED")

        if self.sub_position is None:
            self.get_logger().warning("No Gazebo submarine pose received yet")
            return

        hit = self.check_torpedo_hit()

        msg = Bool()
        msg.data = hit
        self.torpedo_result_pub.publish(msg)

        if hit:
            self.get_logger().info("TORPEDO HIT!")
        else:
            self.get_logger().info("TORPEDO MISS")


    def dropper_released(self):
        self.get_logger().info("DROPPER RELEASED")

        if self.sub_position is None:
            self.get_logger().warning("No Gazebo submarine pose received yet")
            return

        hit = self.check_dropper_hit()

        msg = Bool()
        msg.data = hit
        self.dropper_result_pub.publish(msg)

        if hit:
            self.get_logger().info("DROPPER HIT!")
        else:
            self.get_logger().info("DROPPER MISS")

    def check_dropper_hit(self):
        dx = self.sub_position[0] - self.dropper_target[0]
        dy = self.sub_position[1] - self.dropper_target[1]

        horizontal_distance = math.sqrt(dx**2 + dy**2)

        hit = horizontal_distance <= POSITION_TOLERANCE

        self.get_logger().info(
            f"Dropper horizontal error: {horizontal_distance:.3f} m"
        )

        return hit
        
    def check_torpedo_hit(self):
        dx = self.torpedo_target[0] - self.sub_position[0]
        dy = self.torpedo_target[1] - self.sub_position[1]
        dz = self.torpedo_target[2] - self.sub_position[2]

        # Horizontal distance to target
        horizontal_distance = math.sqrt(dx**2 + dy**2)

        # Direction submarine SHOULD be facing
        desired_yaw = math.atan2(dy, dx)

        # Smallest angle difference
        yaw_error = math.atan2(
            math.sin(desired_yaw - self.sub_yaw),
            math.cos(desired_yaw - self.sub_yaw)
        )

        distance_ok = horizontal_distance <= TORPEDO_MAX_DISTANCE
        z_ok = abs(dz) <= POSITION_TOLERANCE
        yaw_ok = abs(yaw_error) <= YAW_TOLERANCE

        self.get_logger().info(
            f"Torpedo distance: {horizontal_distance:.2f} m | "
            f"Z error: {dz:.2f} m | "
            f"Yaw error: {math.degrees(yaw_error):.1f} deg"
        )

        return distance_ok and z_ok and yaw_ok


def main(args=None):

    rclpy.init(args=args)

    node = GameControllerInput()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()