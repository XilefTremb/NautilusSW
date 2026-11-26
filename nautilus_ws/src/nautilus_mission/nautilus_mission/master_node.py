#!/usr/bin/env python3

import argparse
import json
from dataclasses import asdict, is_dataclass
from enum import Enum
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from std_msgs.msg import Float32MultiArray, Int8, Float32, Int16, Int16MultiArray, UInt8, String, Empty
from nav_msgs.msg import Odometry

from nautilus_mission.detection_store import DetectionStore
from nautilus_mission.state_machine import StateMachine
from nautilus_mission.vision_controller import VisionController
from nautilus_interfaces.srv import SetTargetDepth
from nautilus_mission.rosbag_recorder import RosbagRecorder
from robot_localization.srv import SetPose
from std_srvs.srv import Trigger

from enums.InferenceMode import InferenceMode

class MasterNode(Node):
    def __init__(self, args):
        super().__init__('master_node')

        fast_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.detection_store = DetectionStore(self.get_logger())
        self.fsm = StateMachine(self, self.detection_store, use_sim=args.sim)
        self.vision_controller = VisionController(self)

        # Subscribers
        self.odometry_filtered_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odometry_filtered_callback, fast_qos)
        self.fwd_detection_sub = self.create_subscription(Float32MultiArray,'/yolo/detections_forward',self.fwd_detection_callback,fast_qos)
        self.dwd_detection_sub = self.create_subscription(Float32MultiArray, 'yolo/detections_downward', self.dwd_detection_callback, fast_qos)
        self.mean_depth_sub = self.create_subscription(Int16,'/yolo/mean_depth_forward_cam',self.mean_depth_callback,fast_qos)

        # Publishers
        self.yaw_error_pub = self.create_publisher(Float32, '/control/vision_errors/yaw', 10)
        self.forward_error_pub = self.create_publisher(Float32, '/control/vision_errors/forward', 10)
        self.lateral_error_pub = self.create_publisher(Float32, '/control/vision_errors/lateral', 10)
        self.lateral_width_error_pub = self.create_publisher(Float32, '/control/vision_errors/lateral_width', 10)
        self.throttle_error_pub = self.create_publisher(Float32, '/control/vision_errors/throttle', 10)
        self.forward_ekf_error_pub = self.create_publisher(Float32, '/control/vision_errors/forward_ekf', 10)
        self.lateral_ekf_error_pub = self.create_publisher(Float32, '/control/vision_errors/lateral_ekf', 10)

        self.bottom_cam_forward_error_pub = self.create_publisher(Float32, '/control/vision_errors/bottom_cam/forward', 10)
        self.bottom_cam_lateral_error_pub = self.create_publisher(Float32, '/control/vision_errors/bottom_cam/lateral', 10)

        self.yaw_cmd_pub = self.create_publisher(Int16, '/control/cmd/yaw', 10)
        self.forward_cmd_pub = self.create_publisher(Int16, '/control/cmd/forward', 10)
        self.lateral_cmd_pub = self.create_publisher(Int16, '/control/cmd/lateral', 10)
        self.throttle_cmd_pub = self.create_publisher(Int16, '/control/cmd/throttle', 10)

        self.detections_depth_filter_mm_pub = self.create_publisher(Int16, '/yolo/detections_depth_filter_mm', 10)
        self.sim_torpedo_pub = self.create_publisher(Empty, '/game/fire_torpedo', 10)
        self.servo_cmd_pub = self.create_publisher(Int16MultiArray, '/control/cmd/servo', 10)

        # Inference mode is republished at a fixed rate (see inference_mode_timer)
        # so a late-joining / restarted YOLO node keeps getting the current value.
        self.inference_mode_pub = self.create_publisher(UInt8, '/yolo/inference_mode', 10)
   
        # Services
        self.depth_client = self.create_client(SetTargetDepth,'/mission/set_target_depth')
        self.set_pose_client = self.create_client(SetPose,'/set_pose')

        self.pid_reset_clients = {
            "yaw": self.create_client(Trigger, "/pid_forward_cam_yaw/reset_pid"),
            "forward": self.create_client(Trigger, "/pid_forward_cam_forward/reset_pid"),
            "lateral": self.create_client(Trigger, "/pid_forward_cam_lateral/reset_pid"),
            "forward_ekf": self.create_client(Trigger, "/pid_forward_ekf/reset_pid"),
            "lateral_ekf": self.create_client(Trigger, "/pid_lateral_ekf/reset_pid"),
            # "lateral_width": self.create_client(Trigger, "/pid_width_lateral/reset_pid"),
        }

        self.mission_status_pub = self.create_publisher(String, '/mission/status', 10)
        self.mission_status_timer = self.create_timer(0.2, self.publish_mission_status)

        # Timer
        self.timer = self.create_timer(1 / 20, self.pipeline_tick)

        # Current inference mode, republished at a fixed rate for robustness.
        self.current_inference_mode = int(InferenceMode.BOTH)
        self.inference_mode_timer = self.create_timer(0.5, self._republish_inference_mode)

        # Depth threshold timer
        self.depth_threshold = self.create_timer(0.5, self.depth_threshold_timer)

        self.get_logger().info(f'Mission will start in {args.timer} seconds.')
        time.sleep(args.timer) 
        self.fsm.start_mission()

        self.get_logger().info('Master mission + vision node started.')

    def publish_mission_status(self):
        fsm = self.fsm
        now = time.monotonic()
        active = fsm.state not in ('IDLE', 'LOAD_OBJECTIVE', 'MISSION_COMPLETE')
        objective = fsm.current_objective if active else None

        def encode(value):
            if isinstance(value, Enum):
                return value.name
            if is_dataclass(value):
                return asdict(value)
            raise TypeError(type(value).__name__)

        status = {
            'version': 1,
            'state': fsm.state,
            'current_index': fsm.objective_index if active else None,
            'objectives': [
                {'name': item.name, 'status': fsm.monitor_outcomes.get(i,
                    'ACTIVE' if active and i == fsm.objective_index else 'PENDING')}
                for i, item in enumerate(fsm.objectives)
            ],
            'mission_elapsed_s': (fsm.monitor_finished or now) - fsm.monitor_started if fsm.monitor_started is not None else 0,
            'state_elapsed_s': (fsm.monitor_finished or now) - fsm.state_start_time,
            'objective_elapsed_s': now - fsm.current_objective_start_time_s if active else None,
            'parameters': asdict(objective) if objective is not None else {},
            'runtime': {
                'target_ids': [item.name for item in fsm.target_ids or []],
                'vision_action': fsm.vision_action.name,
                'role_choice': fsm.role_choice.name,
                'success_frames': fsm.current_success_frame_count,
                'missing_frames': fsm.target_missing_count,
                'forward_position_m': fsm.forward_position,
                'lateral_position_m': fsm.lateral_position,
                'camera_mean_depth_mm': fsm.mean_depth_forward_cam,
            } if active else {},
            'events': fsm.monitor_events,
        }
        msg = String()
        msg.data = json.dumps(status, default=encode)
        self.mission_status_pub.publish(msg)

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

    def _publish_float32(self, pub, value: float):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)
    
    def _publish_int16(self, pub, value: int):
        msg = Int16()
        msg.data = int(value)
        pub.publish(msg)

    def publish_error(self, name: str, error: float):
        pubs = {
            "yaw": self.yaw_error_pub,
            "throttle": self.throttle_error_pub,
            "forward": self.forward_error_pub,
            "forward_ekf": self.forward_ekf_error_pub,
            "lateral_ekf": self.lateral_ekf_error_pub,
            "lateral": self.lateral_error_pub,
            "lateral_width": self.lateral_width_error_pub,
            "bottom_lateral": self.bottom_cam_lateral_error_pub,
            "bottom_forward": self.bottom_cam_forward_error_pub,
        }

        self._publish_float32(pubs[name], error)

    def publish_cmd(self, name: str, pwm: int):
        pubs = {
            "forward": self.forward_cmd_pub,
            "lateral": self.lateral_cmd_pub,
            "throttle": self.throttle_cmd_pub,
            "yaw": self.yaw_cmd_pub,
        }

        self._publish_int16(pubs[name], pwm)

    def publish_detections_depth_filter_mm(self, threshold: int):
        msg = Int16()
        msg.data = int(threshold)
        self.detections_depth_filter_mm_pub.publish(msg)

    def depth_threshold_timer(self):
        depth = self.fsm.current_objective.detections_depth_filter_mm
        if depth is not None:
            self.publish_detections_depth_filter_mm(depth)

    def publish_inference_mode(self, mode: int):
        # Store the requested mode; it is (re)published at a fixed rate.
        self.current_inference_mode = int(mode)
        self._republish_inference_mode()

    def _republish_inference_mode(self):
        msg = UInt8()
        msg.data = int(self.current_inference_mode)
        self.inference_mode_pub.publish(msg)

    def publish_sim_torpedo_trigger(self):
        self.sim_torpedo_pub.publish(Empty())

    def publish_servo_cmd(self, servo: int, pwm: int):
        msg = Int16MultiArray()
        msg.data = [servo, pwm]
        self.servo_cmd_pub.publish(msg)

def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--timer", type=float, default=0.0)
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = MasterNode(parsed_args)

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
