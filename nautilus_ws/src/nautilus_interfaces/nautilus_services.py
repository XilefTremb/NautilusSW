from nautilus_interfaces.srv import SetTargetDepth

def request_depth_change(node, depth):

    if not node.depth_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn('Depth service unavailable')
        return

    req = SetTargetDepth.Request()
    req.depth_m = float(depth)

    future = node.depth_client.call_async(req)

    # future.add_done_callback(
    #     node.depth_change_response
    # )