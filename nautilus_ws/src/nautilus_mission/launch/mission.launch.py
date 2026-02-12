from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    if LaunchConfiguration('use_fake_dvl'):
        params_profile = "--auv"
    else:
        params_profile = "--sitl"

    dvl=Node(
        package='nautilus_sensors',
        executable='fake_dvl',
        condition=IfCondition(LaunchConfiguration('use_fake_dvl'))
    )
    
    test_mission = Node(
        package='nautilus_mission',
        executable='test_mission',
        arguments=[params_profile]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_dvl",
                default_value='False',
                description="launches fake_dvl node and sets ardusub params to accept external nav"
            ),
            dvl,
            test_mission
        ]
    )
