from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_forward_cam_yaw',
            parameters=[{
                'input_topic': '/control/vision_errors/yaw',
                'output_topic': '/control/cmd/yaw',
                'kp': 130.0,
                'ki': 2.0,
                'kd': 80.0,
            }]
        ),

         Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_forward_cam_forward',
            parameters=[{
                'input_topic': '/control/vision_errors/forward',
                'output_topic': '/control/cmd/forward',
                'kp': 2.0,
                'ki': 0.0,
                'kd': 0.5,
                'flip_output': True
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_forward_cam_lateral',
            parameters=[{
                'input_topic': '/control/vision_errors/lateral',
                'output_topic': '/control/cmd/lateral',
                'kp': 20.0,
                'ki': 0.0,
                'kd': 0.5,
                'flip_output': True
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_forward_cam_throttle',
            parameters=[{
                'input_topic': '/control/vision_errors/throttle',
                'output_topic': '/control/cmd/throttle',
                'kp': 10.0,
                'ki': 0.0,
                'kd': 1.0, #TO BE TUNED
                'flip_outout': True,
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_forward_ekf',
            parameters=[{
                'input_topic': '/control/vision_errors/forward_ekf',
                'output_topic': '/control/cmd/forward',
                'kp': 200.0,
                'ki': 0.1,
                'kd': 0.5,
                'flip_output': False
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_lateral_ekf',
            parameters=[{
                'input_topic': '/control/vision_errors/lateral_ekf',
                'output_topic': '/control/cmd/lateral',
                'kp': 200.0,
                'ki': 0.1,
                'kd': 0.5,
                'flip_output': False
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_bottom_cam_forward',
            parameters=[{
                'input_topic': '/control/vision_errors/bottom_cam/forward',
                'output_topic': '/control/cmd/forward',
                'kp': 140.0,
                'ki': 0.8,
                'kd': 2.5,
                'flip_output': True
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_bottom_cam_lateral',
            parameters=[{
                'input_topic': '/control/vision_errors/bottom_cam/lateral',
                'output_topic': '/control/cmd/lateral',
                'kp': 140.0,
                'ki': 0.8,
                'kd': 2.5,
            }]
        )
    ])