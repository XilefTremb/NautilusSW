from nautilus_interfaces.srv import SetTargetDepth
from std_srvs.srv import Trigger
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

    try:
        future.add_done_callback(
            lambda _: node.get_logger().info('EKF reset complete')
        )
        return True
    except: 
        return False

def request_depth_change(node, depth):

    if not node.depth_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn('Depth service unavailable')
        return

    req = SetTargetDepth.Request()
    req.depth_m = float(depth)

    node.get_logger().info(f"requesting depth change to {req.depth_m}")

    future = node.depth_client.call_async(req)

    # future.add_done_callback(
    #     node.depth_change_response
    # )

def reset_pids(node, timeout_sec = 5.0):

    futures = []

    for name, client in node.pid_reset_clients.items():
        if not client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().warn(f"{name} PID reset service unavailable")
            continue

        req = Trigger.Request()
        future = client.call_async(req)
        futures.append((name, future))

    node.get_logger().info(
        f"Requested PID reset for {len(futures)}/{len(node.pid_reset_clients)} services"
    )

    return futures