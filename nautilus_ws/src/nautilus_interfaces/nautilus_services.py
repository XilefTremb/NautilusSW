from nautilus_interfaces.srv import SetTargetDepth
from std_srvs.srv import Trigger

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

def reset_pids(node):

    if not node.yaw_reset_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn('Yaw pid reset service unavailable')
        return

    if not node.forward_reset_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn('Forward pid reset service unavailable')
        return
    
    if not node.lateral_reset_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn('Lateral pid reset service unavailable')
        return
    
    if not node.forward_ekf_reset_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn('Forward EKF pid reset service unavailable')
        return
    
    req = Trigger.Request()

    future = node.yaw_reset_client.call_async(req)
    future = node.forward_reset_client.call_async(req)
    future = node.lateral_reset_client.call_async(req)
    future = node.forward_ekf_reset_client.call_async(req)

    node.get_logger().info("Resetted all PID nodes integral and differential values")