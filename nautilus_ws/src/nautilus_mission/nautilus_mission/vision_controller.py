#!/usr/bin/env python3

import math
from typing import Optional

from rclpy.node import Node

from enums.VisionAction import VisionAction
from enums.DetectionIndex import DetectionIndex

class VisionController:
    """Vision action to error/cmd calculation only. No ROS subscriptions."""

    def __init__(self, node: Node):
        self.node = node
        self.previous_vision_action: Optional[VisionAction] = None

    def process(self, vision_action: VisionAction, target_detection):
        if self.previous_vision_action != vision_action:
            if self.previous_vision_action is not None:
                self.node.get_logger().info(
                    f'Set vision_action from {self.previous_vision_action.name} to: {vision_action.name}'
                )
            else:
                self.node.get_logger().info(
                    f'Set vision_action from {self.previous_vision_action} to: {vision_action.name}'
                )
            self.previous_vision_action = vision_action

        if vision_action == VisionAction.CENTER_TARGET:
            self.center_target(target_detection)
        elif vision_action == VisionAction.APPROACH_TARGET:
            self.approach_target(target_detection)
        elif vision_action == VisionAction.CIRCLE_MARKER:
            self.circle_marker(target_detection)
        elif vision_action == VisionAction.CENTER_FOV:
            self.center_fov(target_detection)
        elif vision_action == VisionAction.CENTER_BOTTOM:
            self.center_bottom(target_detection)

    def center_target(self, target_detection):
        if target_detection is None:
            return

        px = target_detection[DetectionIndex.CENTER_FOV_RATIO_X]
        angle = target_detection[DetectionIndex.ANGLE_DEG]

        forward_error, lateral_error = self.split_angle(angle)
        self.node.publish_forward_error(forward_error)
        self.node.publish_lateral_error(lateral_error)
        self.node.publish_yaw_error(float(px))

    def center_bottom(self, target_detection):
        if target_detection is None:
            return

        px_error = target_detection[DetectionIndex.CENTER_FOV_RATIO_X] - self.node.fsm.current_objective.center.target_offset_x
        py_error = target_detection[DetectionIndex.CENTER_FOV_RATIO_Y] - self.node.fsm.current_objective.center.target_offset_y

        self.node.publish_bottom_cam_lateral_error(float(px_error))
        self.node.publish_bottom_cam_forward_error(float(py_error))

    def approach_target(self, target_detection):
        if target_detection is not None:
            px = target_detection[DetectionIndex.CENTER_FOV_RATIO_X]
            self.node.publish_yaw_error(float(px))

        self.node.publish_forward_cmd(1540)

    def circle_marker(self, target_detection):
        if target_detection is None:
            return

        px = target_detection[DetectionIndex.CENTER_FOV_RATIO_X]
        self.node.publish_yaw_error(float(px) - self.circle_marker_pixel_offset)
        self.node.publish_forward_cmd(1540)
        self.node.publish_lateral_cmd(1375)

        if self.circle_marker_pixel_offset < 280.0:
            self.circle_marker_pixel_offset += 1.0

    def center_fov(self, target_detection):
        if target_detection is None:
            return
        
        px = target_detection[DetectionIndex.CENTER_FOV_RATIO_X]
        self.node.publish_yaw_error(float(px))

    def center_dropper(self, target_detection):
        if target_detection is None:
            return
        
        px_error = target_detection[DetectionIndex.CENTER_FOV_X_RATIO]
        py_error = target_detection[DetectionIndex.CENTER_HEIGHT_RATIO]


    def split_angle(self, angle_deg):
        angle = math.radians(angle_deg)
        forward_error = -angle_deg * math.sin(angle)
        lateral_error = angle_deg * math.cos(angle)
        return forward_error, lateral_error
