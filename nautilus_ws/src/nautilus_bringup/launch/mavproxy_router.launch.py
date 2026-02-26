from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_arguments():
    """Generate a list of launch arguments"""
    return [
        # MAVProxy fan-out ports
        DeclareLaunchArgument(
            "qgc_out", 
            default_value="14550", 
            description="UDP port for QGC telemetry."
        ),
        DeclareLaunchArgument(
            "cmd_out",
            default_value="14551",
            description="UDP port for command script.",
        ),
        DeclareLaunchArgument(
            "dvl_out",
            default_value="14552",
            description="UDP port for DVL script telemetry.",
        ),
        # Core endpoints
        DeclareLaunchArgument(
            "master",
            default_value="/dev/serial/by-id/usb-CubePilot_CubeOrange+_3E003F000C51333130373434-if00",
            description="Serial MAVLink master endpoint.",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="921600",
            description="Baudrate for the serial connection.",
        ),
    ]

def generate_launch_description():
    launch_arguments = generate_launch_arguments()
    
    # Launch MAVProxy manually with multiple --out endpoints.
    mavproxy_multi_out = ExecuteProcess(
        cmd=[
            "mavproxy.py",
            "--master",
            LaunchConfiguration("master"),
            "--baudrate",
            LaunchConfiguration("baudrate"),
            "--out",
            ["udp:127.0.0.1:", LaunchConfiguration("qgc_out")],
            "--out",
            ["udp:127.0.0.1:", LaunchConfiguration("cmd_out")],
            "--out",
            ["udp:127.0.0.1:", LaunchConfiguration("dvl_out")],
        ],
        output="screen",
    )

    ld = LaunchDescription(launch_arguments)
    ld.add_action(mavproxy_multi_out)

    return ld