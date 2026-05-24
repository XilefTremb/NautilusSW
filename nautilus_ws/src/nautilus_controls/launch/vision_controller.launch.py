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
                'kp': 0.4,
                'ki': 0.0,
                'kd': 0.1,
            }]
        ),

         Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_forward',
            parameters=[{
                'input_topic': '/control/vision_errors/forward',
                'output_topic': '/control/cmd/forward',
                'kp': 1.2,
                'ki': 0.1,
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
                'kp': 1.2,
                'ki': 0.1,
                'kd': 0.5,
                'flip_output': True
            }]
        ),
    ])