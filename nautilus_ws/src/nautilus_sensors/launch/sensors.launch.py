from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    xsens = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("xsens_mti_ros2_driver"),
                "launch",
                "xsens_mti_node.launch.py",
            )
        ),
        launch_arguments={
            "enable_deviceConfig": "true",
        }.items(),
    )


    yolo = Node(
        package="nautilus_sensors",
        executable="yolo_pipeline",
        name="yolo_pipeline",
        output="screen",
        arguments=["--real", "--bbox"],
    )

    return LaunchDescription([
        xsens,
        yolo,
    ])