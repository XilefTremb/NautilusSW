#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from geometry_msgs.msg import PoseStamped
from evdev import InputDevice, ecodes

import threading
import math
import numpy as np
import subprocess


TORPEDO_BUTTON = 311   # BTN_TR
DROPPER_BUTTON = 310   # BTN_TL


# Fixed Gazebo target positions
TORPEDO_TARGET = np.array([6.93, 0.80, -1.35])
DROPPER_TARGET = np.array([6.85, 5.07, -1.85])

TORPEDO_MAX_DISTANCE = 2.0     # m
POSITION_TOLERANCE = 0.2       # m
YAW_TOLERANCE = math.radians(10.0)

# Torpedo model important constants
TORPEDO_MODEL_NAME = "test_torpedo_world"
TORPEDO_LAUNCH_OFFSET = 0.1   # m in front of AUV
TORPEDO_FORCE = 10.0          # N
TORPEDO_THRUST_TIME = 0.45     # seconds

TORPEDO_HIDE_X = -100.0
TORPEDO_HIDE_Y = -100.0
TORPEDO_HIDE_Z = -0.5
TORPEDO_RESET_TIME = 3.0  # seconds after firing

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

        # Torpedo state
        self.torpedo_active = False


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

        if self.torpedo_active:
            self.get_logger().warning("Torpedo already active\n")
            return

        self.torpedo_active = True

        self.get_logger().info("TORPEDO FIRED")

        if self.sub_position is None:
            self.get_logger().warning("No Gazebo submarine pose received yet")
            return

        self.launch_torpedo_visual()

        hit = self.check_torpedo_hit()

        # Show visual tracer
        # self.show_torpedo_tracer(hit)

        msg = Bool()
        msg.data = hit
        self.torpedo_result_pub.publish(msg)

        if hit:
            self.get_logger().info("TORPEDO HIT YAYYYYYY!")
        else:
            self.get_logger().info("TORPEDO MISS... :(")


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
            self.get_logger().info("DROPPER HIT YAYYYYYY!")
        else:
            self.get_logger().info("DROPPER MISS... :(")

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

        return bool(distance_ok and z_ok and yaw_ok)

    def show_torpedo_tracer(self, hit):
        if self.sub_position is None:
            return

        # --------------------------------------------------
        # Tracer configuration
        # --------------------------------------------------

        TRACER_DIAMETER = 0.032
        MAX_DISTANCE = 2.0
        LIFETIME_NS = 500_000_000  # 0.5 s

        # --------------------------------------------------
        # Start point
        # --------------------------------------------------

        start_x = self.sub_position[0]
        start_y = self.sub_position[1]
        start_z = self.sub_position[2]

        # --------------------------------------------------
        # End point
        # --------------------------------------------------

        if hit:
            end_x = self.torpedo_target[0]
            end_y = self.torpedo_target[1]
            end_z = self.torpedo_target[2]

            # Green
            r = 0.0
            g = 1.0
            b = 0.0

        else:
            end_x = start_x + MAX_DISTANCE * math.cos(self.sub_yaw)
            end_y = start_y + MAX_DISTANCE * math.sin(self.sub_yaw)
            end_z = start_z

            # Red
            r = 1.0
            g = 0.0
            b = 0.0

        # --------------------------------------------------
        # Cylinder position = midpoint of tracer
        # --------------------------------------------------

        mid_x = (start_x + end_x) / 2.0
        mid_y = (start_y + end_y) / 2.0
        mid_z = (start_z + end_z) / 2.0

        # Length of tracer
        dx = end_x - start_x
        dy = end_y - start_y
        dz = end_z - start_z

        tracer_length = math.sqrt(
            dx**2 +
            dy**2 +
            dz**2
        )

        # --------------------------------------------------
        # Cylinder orientation
        #
        # Gazebo cylinders point along their local Z axis.
        # We need to rotate local Z onto the shot direction.
        # --------------------------------------------------

        horizontal_distance = math.sqrt(dx**2 + dy**2)

        yaw = math.atan2(dy, dx)

        pitch = math.atan2(
            horizontal_distance,
            dz
        )

        # Quaternion from roll=0, pitch, yaw
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        qx = -sp * sy
        qy = sp * cy
        qz = cp * sy
        qw = cp * cy

        # --------------------------------------------------
        # Gazebo marker request
        # --------------------------------------------------

        marker_request = f"""
            action: ADD_MODIFY,
            ns: "torpedo_tracer",
            id: 1,
            type: CYLINDER,
            pose: {{
                position: {{
                    x: {mid_x},
                    y: {mid_y},
                    z: {mid_z}
                }},
                orientation: {{
                    x: {qx},
                    y: {qy},
                    z: {qz},
                    w: {qw}
                }}
            }},
            scale: {{
                x: {TRACER_DIAMETER},
                y: {TRACER_DIAMETER},
                z: {tracer_length}
            }},
            material: {{
                ambient: {{
                    r: {r},
                    g: {g},
                    b: {b},
                    a: 1.0
                }},
                diffuse: {{
                    r: {r},
                    g: {g},
                    b: {b},
                    a: 1.0
                }}
            }},
            lifetime: {{
                sec: 0,
                nsec: {LIFETIME_NS}
            }},
            visibility: ALL
            """

        subprocess.run([
            "gz",
            "service",
            "-s", "/marker",
            "--reqtype", "gz.msgs.Marker",
            "--reptype", "gz.msgs.Empty",
            "--timeout", "1000",
            "--req", marker_request
        ])

    def launch_torpedo_visual(self):
        x = self.sub_position[0]
        y = self.sub_position[1]
        z = self.sub_position[2]
        yaw = self.sub_yaw

        # Spawn position slightly in front of the submarine
        torpedo_x = x + TORPEDO_LAUNCH_OFFSET * math.cos(yaw)
        torpedo_y = y + TORPEDO_LAUNCH_OFFSET * math.sin(yaw)
        torpedo_z = z

        # Torpedo model was originally rotated +90° around Y
        # so its local +Z axis becomes its forward direction.
        half_yaw = yaw / 2.0
        half_pitch = math.pi / 4.0  # pitch = +90 deg

        qx = -math.sin(half_yaw) * math.sin(half_pitch)
        qy =  math.cos(half_yaw) * math.sin(half_pitch)
        qz =  math.sin(half_yaw) * math.cos(half_pitch)
        qw =  math.cos(half_yaw) * math.cos(half_pitch)

        # Move torpedo from its hidden location to the launcher
        pose_request = (
            f'name: "{TORPEDO_MODEL_NAME}", '
            f'position: {{x: {torpedo_x}, y: {torpedo_y}, z: {torpedo_z}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )

        # Force direction follows submarine yaw
        force_x = TORPEDO_FORCE * math.cos(yaw)
        force_y = TORPEDO_FORCE * math.sin(yaw)

        wrench_request = (
            f'entity: {{name: "{TORPEDO_MODEL_NAME}::link", type: LINK}}, '
            f'wrench: {{force: {{x: {force_x}, y: {force_y}, z: 0.0}}}}'
        )

        # Move torpedo
        subprocess.Popen([
            "gz", "service",
            "-s", "/world/default/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", pose_request,
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Apply thrust immediately without waiting for set_pose to return
        subprocess.Popen([
            "gz", "topic",
            "-t", "/world/default/wrench/persistent",
            "-m", "gz.msgs.EntityWrench",
            "-p", wrench_request,
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        threading.Timer(
            TORPEDO_THRUST_TIME,
            self.stop_torpedo_thrust
        ).start()

        # Automatically stop thrust shortly afterward
        threading.Timer(
            TORPEDO_THRUST_TIME,
            self.stop_torpedo_thrust
        ).start()

        threading.Timer(
            TORPEDO_RESET_TIME,
            self.reset_torpedo
        ).start()

    def stop_torpedo_thrust(self):
        subprocess.run([
            "gz", "topic",
            "-t", "/world/default/wrench/clear",
            "-m", "gz.msgs.Entity",
            "-p",
            f'name: "{TORPEDO_MODEL_NAME}::link", type: LINK',
        ])

    def reset_torpedo(self):
        # Make sure no thrust is still being applied
        self.stop_torpedo_thrust()

        pose_request = (
            f'name: "{TORPEDO_MODEL_NAME}", '
            f'position: {{x: {TORPEDO_HIDE_X}, y: {TORPEDO_HIDE_Y}, z: {TORPEDO_HIDE_Z}}}, '
            f'orientation: {{x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}'
        )

        subprocess.Popen([
            "gz", "service",
            "-s", "/world/default/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "1000",
            "--req", pose_request,
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        self.torpedo_active = False


def main(args=None):

    rclpy.init(args=args)

    node = GameControllerInput()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()