# ros_node.py

import math
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, PoseStamped, TwistWithCovarianceStamped
from std_msgs.msg import Float32, Float32MultiArray, Int32, UInt8
from sensor_msgs.msg import Imu, BatteryState
from rclpy.qos import qos_profile_sensor_data
from ardupilot_msgs.msg import Rc

from config import TOPICS

PROFILING_STAGES = [
    "camera",
    "image_cam_interval",
    "yolo_inference",
    "detection_processing",
    "angle_computation",
    "process_total",
    "publish",
    "detections_forward_interval",
    "detections_downward_interval",
    "image_annotated_fwd_cam_interval",
    "image_annotated_dwd_cam_interval",
    "end_2_end"
]


class NautilusROSNode(Node):

    def __init__(self, data_manager):
        super().__init__("nautilus_hmi")

        self.data = data_manager

        # SENSORS
        self.create_subscription(Imu, TOPICS["imu"], self.imu_callback, 10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            TOPICS["dvl"],
            self.dvl_callback,
            10)
        

        self.create_subscription(
            PoseStamped,
            TOPICS["depth"],
            lambda msg: self.data.update("Depth", abs(msg.pose.position.z)),
            qos_profile_sensor_data)

        self.create_subscription(
            BatteryState,
            TOPICS["battery"],
            self.battery_callback,
            qos_profile_sensor_data)
        
        # CAMERA
        self.create_subscription(Int32, TOPICS["camera_depth"], self.camera_depth_callback, 10)
        self.create_subscription(UInt8, TOPICS["inference_mode"], self.inference_mode_callback, 10)
        
        # MOTORS
        self.create_subscription(Rc, TOPICS["motor_pwm"], self.pwm_callback, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, TOPICS["motor_current"], self.current_callback, 10)

         # PROFILING
        self.create_subscription(
            Float32MultiArray,
            TOPICS["profiling"],
            self.profiling_callback,
            10
        )

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
        velocity = msg.twist.twist.linear

        self.data.update("DVL X", velocity.x)
        self.data.update("DVL Y", velocity.y)
        self.data.update("DVL Z", velocity.z)

    def pwm_callback(self, msg):
        for i, value in enumerate(msg.channels[:8]):
            self.data.update(f"Motor {i+1} PWM", value)
    
    def current_callback(self, msg):
        for i, value in enumerate(msg.data[:8]):
            self.data.update(f"Motor {i+1} Current", value)

    def battery_callback(self, msg):
        self.data.update("Battery Voltage", msg.voltage)
        self.data.update("Battery Current", msg.current)
        
    def camera_depth_callback(self, msg):
        self.data.update("Camera Depth", msg.data)

    def inference_mode_callback(self, msg):
        self.data.update("Inference Mode", msg.data)
    
    def profiling_callback(self, msg):
        if len(msg.data) != len(PROFILING_STAGES):
            return

        values = dict(zip(PROFILING_STAGES, msg.data))
        camera = int(values.pop("camera"))

        if camera in (0, 1):
            values = {
                name: value
                for name, value in values.items()
                if not math.isnan(value)
            }

            self.data.update_profiling(camera, values)