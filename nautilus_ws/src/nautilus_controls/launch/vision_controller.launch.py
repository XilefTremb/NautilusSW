from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_yaw',
            parameters=[{
                'input_topic': '/control/vision_errors/yaw',
                'output_topic': '/control/cmd/yaw',
                'kp': 150.0,
                'ki': 0.0,
                'kd': 32.0,
            }]
        ),

         Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_forward',
            parameters=[{
                'input_topic': '/control/vision_errors/forward',
                'output_topic': '/control/cmd/forward',
                'kp': 5.0,
                'ki': 0.0,
                'kd': 0.5,
                'flip_output': True
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_lateral',
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
            name='pid_forward_ekf',
            parameters=[{
                'input_topic': '/control/vision_errors/forward_ekf',
                'output_topic': '/control/cmd/forward',
                'kp': 60.0,
                'ki': 0.0,
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
                'kp': 40.0,
                'ki': 0.0,
                'kd': 1.0,
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
                'kp': 40.0,
                'ki': 0.0,
                'kd': 1.0,
            }]
        )
    ])