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
                'kp': 0.5,
                'ki': 0.0,
                'kd': 0.1,
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='pid_node',
            name='pid_lateral',
            parameters=[{
                'input_topic': '/control/vision_errors/lateral',
                'output_topic': '/control/cmd/lateral',
                'kp': 0.5,
                'ki': 0.01,
                'kd': 0.06,
            }]
        ),

        Node(
            package='nautilus_controls',
            executable='vision_controller',
            name='vision_controller',
        ),
    ])