#!/usr/bin/env python3

from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped

def reset_ekf_pose(node):

    if not node.set_pose_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn('/set_pose unavailable')
        return

    req = SetPose.Request()

    pose = PoseWithCovarianceStamped()

    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = 'odom'

    pose.pose.pose.position.x = 0.0
    pose.pose.pose.position.y = 0.0
    pose.pose.pose.position.z = 0.0

    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y = 0.0
    pose.pose.pose.orientation.z = 0.0
    pose.pose.pose.orientation.w = 1.0

    pose.pose.covariance = [
        0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
    ]

    req.pose = pose

    future = node.set_pose_client.call_async(req)

    future.add_done_callback(
        lambda _: node.get_logger().info('EKF reset complete')
    )