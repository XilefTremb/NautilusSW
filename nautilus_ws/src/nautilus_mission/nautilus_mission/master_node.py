#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from std_msgs.msg import Float32MultiArray, Int8, Float32, Int16, Int16MultiArray
from nav_msgs.msg import Odometry

from nautilus_mission.detection_store import DetectionStore
from nautilus_mission.state_machine import StateMachine
from nautilus_mission.vision_controller import VisionController
from nautilus_interfaces.srv import SetTargetDepth
from nautilus_mission.rosbag_recorder import RosbagRecorder
from robot_localization.srv import SetPose
from std_srvs.srv import Trigger


class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        fast_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.detection_store = DetectionStore(self.get_logger())
        self.fsm = StateMachine(self, self.detection_store)
        self.vision_controller = VisionController(self)

        # Subscribers
        self.odometry_filtered_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odometry_filtered_callback, fast_qos)
        self.fwd_detection_sub = self.create_subscription(Float32MultiArray,'/yolo/detections_forward',self.fwd_detection_callback,fast_qos)
        self.dwd_detection_sub = self.create_subscription(Float32MultiArray, 'yolo/detections_downward', self.dwd_detection_callback, fast_qos)
        self.mean_depth_sub = self.create_subscription(Int16,'/yolo/mean_depth_forward_cam',self.mean_depth_callback,fast_qos)

        # Publishers
        self.yaw_error_pub = self.create_publisher(Float32, '/control/vision_errors/yaw', 10)
        self.forward_error_pub = self.create_publisher(Float32, '/control/vision_errors/forward', 10)
        self.forward_ekf_error_pub = self.create_publisher(Float32, '/control/vision_errors/forward_ekf', 10)
        self.lateral_error_pub = self.create_publisher(Float32, '/control/vision_errors/lateral', 10)
        self.bottom_cam_forward_error_pub = self.create_publisher(Float32, '/control/vision_errors/bottom_cam/forward', 10)
        self.bottom_cam_lateral_error_pub = self.create_publisher(Float32, '/control/vision_errors/bottom_cam/lateral', 10)
        self.yaw_cmd_pub = self.create_publisher(Int16, '/control/cmd/yaw', 10)
        self.servo_cmd_pub = self.create_publisher(Int16MultiArray, '/control/cmd/servo', 10)
        self.forward_cmd_pub = self.create_publisher(Int16, '/control/cmd/forward', 10)
        self.lateral_cmd_pub = self.create_publisher(Int16, '/control/cmd/lateral', 10)
        self.detections_depth_filter_mm_pub = self.create_publisher(Int16, '/yolo/detections_depth_filter_mm', 10)
        self.servo_cmd_pub = self.create_publisher(Int16MultiArray, '/control/cmd/servo', 10)
   
        # Services
        self.depth_client = self.create_client(SetTargetDepth,'/mission/set_target_depth')
        self.set_pose_client = self.create_client(SetPose,'/set_pose')
        self.yaw_reset_client = self.create_client(Trigger, '/pid_yaw/reset_pid')
        self.forward_reset_client = self.create_client(Trigger, '/pid_forward/reset_pid')
        self.lateral_reset_client = self.create_client(Trigger, '/pid_lateral/reset_pid')
        self.forward_ekf_reset_client = self.create_client(Trigger, '/pid_forward_ekf/reset_pid')

        # Timer
        self.timer = self.create_timer(1 / 20, self.pipeline_tick)

        self.get_logger().info('Master mission + vision node started.')

        self.fsm.start_mission()

    def fwd_detection_callback(self, msg: Float32MultiArray):
        if not self.detection_store.update_from_msg(msg):
            return

        # Immediate callback-driven processing to reduce detection-to-error delay.
        self.vision_tick()

    def dwd_detection_callback(self, msg: Float32MultiArray):
        if not self.detection_store.update_from_msg(msg):
            return
        
        self.vision_tick()

    def mean_depth_callback(self, msg: Int16):
        self.fsm.mean_depth_forward_cam = msg.data

    def odometry_filtered_callback(self, msg: Odometry):
        self.fsm.forward_position = msg.pose.pose.position.x
        self.fsm.lateral_position = msg.pose.pose.position.y

    def pipeline_tick(self):
        self.fsm.tick()
    
    def vision_tick(self):
        target_detection = self.detection_store.get_detection(self.fsm.target_ids)
        self.vision_controller.process(self.fsm.vision_action, target_detection)

    def publish_detections_depth_filter_mm(self, threshold: int):
        msg = Int16()
        msg.data = int(threshold)
        self.detections_depth_filter_mm_pub.publish(msg)

    def publish_yaw_error(self, error: float):
        msg = Float32()
        msg.data = float(error)
        self.yaw_error_pub.publish(msg)

    def publish_forward_error(self, error: float):
        msg = Float32()
        msg.data = float(error)
        self.forward_error_pub.publish(msg)

    def publish_forward_ekf_error(self, error: float):
        msg = Float32()
        msg.data = float(error)
        self.forward_ekf_error_pub.publish(msg)

    def publish_lateral_error(self, error: float):
        msg = Float32()
        msg.data = float(error)
        self.lateral_error_pub.publish(msg)

    def publish_servo_cmd(self, servo: int, pwm: int):
        msg = Int16MultiArray()
        msg.data = [servo, pwm]
        self.servo_cmd_pub.publish(msg)

    def publish_forward_cmd(self, pwm: int):
        msg = Int16()
        msg.data = int(pwm)
        self.forward_cmd_pub.publish(msg)

    def publish_lateral_cmd(self, pwm: int):
        msg = Int16()
        msg.data = int(pwm)
        self.lateral_cmd_pub.publish(msg)

    def publish_yaw_cmd(self, pwm: int):
        msg = Int16()
        msg.data = int(pwm)
        self.yaw_cmd_pub.publish(msg)

    def publish_servo_cmd(self, servo: int, pwm: int):
        msg = Int16MultiArray()
        msg.data = [servo, pwm]
        self.servo_cmd_pub.publish(msg)

    def publish_bottom_cam_lateral_error(self, error: float):
        msg = Float32()
        msg.data = float(error)
        self.bottom_cam_lateral_error_pub.publish(msg)

    def publish_bottom_cam_forward_error(self, error: float):
        msg = Float32()
        msg.data = float(error)
        self.bottom_cam_forward_error_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = MasterNode()

    recorder = RosbagRecorder(node)
    recorder.start()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        recorder.stop()
        recorder.ask_keep_or_delete()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
