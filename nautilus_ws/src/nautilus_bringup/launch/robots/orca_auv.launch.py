# Copyright 2024 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Launch an iris quadcopter in Gazebo and Rviz.

ros2 launch ardupilot_sitl sitl_dds_udp.launch.py
transport:=udp4
port:=2019
synthetic_clock:=True
wipe:=False
model:=json
speedup:=1
slave:=0
instance:=0
defaults:=$(ros2 pkg prefix ardupilot_sitl)
          /share/ardupilot_sitl/config/default_params/gazebo-iris.parm,
          $(ros2 pkg prefix ardupilot_sitl)
          /share/ardupilot_sitl/config/default_params/dds_udp.parm
sim_address:=127.0.0.1
master:=tcp:127.0.0.1:5760
sitl:=127.0.0.1:5501
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_spawn_robot(context):
    """Return a Gazebo spawn robot launch description"""
    # Get substitutions for arguments
    name = LaunchConfiguration("name")
    pos_x = LaunchConfiguration("x")
    pos_y = LaunchConfiguration("y")
    pos_z = LaunchConfiguration("z")
    rot_r = LaunchConfiguration("R")
    rot_p = LaunchConfiguration("P")
    rot_y = LaunchConfiguration("Y")

    # spawn robot
    spawn_robot =  Node(
        package="ros_gz_sim",
        executable="create",
        namespace=name,
        arguments=[
            "-world",
            "",
            "-param",
            "",
            "-name",
            name,
            "-topic",
            "/robot_description",
            "-x",
            pos_x,
            "-y",
            pos_y,
            "-z",
            pos_z,
            "-R",
            rot_r,
            "-P",
            rot_p,
            "-Y",
            rot_y,
        ],
        output="screen",
    )
    return [spawn_robot]


def launch_state_pub_with_bridge(context):
    # Robot description and ros_gz bridge config chosen based on passed lidar_dimension argument
    # lidar_dim = LaunchConfiguration("lidar_dim").perform(context)
    pkg_ardupilot_gz_description = get_package_share_directory("ardupilot_gz_description")
    pkg_project_bringup = get_package_share_directory("nautilus_bringup")


    #xacroPath = os.path.join(pkg_project_bringup, "nautilus_auv", "BlueRov2.urdf.xacro")
    #urdfPath = os.path.join(pkg_project_bringup,'nautilus_auv','BlueRov2.urdf')
    sdfPath = os.path.join(pkg_project_bringup, "models","bluerov2","model.sdf")
    # sdfPath = os.path.join(pkg_project_bringup, "models","bluerov2","model.sdf")


    #os.system("xacro "+ str(xacroPath)+ " -o " + str(urdfPath))   
    #os.system("gz sdf -p " + str(urdfPath) + " > " + str(sdfPath))

    with open(sdfPath, "r") as infp:
        robot_desc = infp.read()
        # print(robot_desc)

    ros_gz_bridge_config = "nautilus_auv_bridge.yaml"
    # ros_gz_bridge_config = "iris_3Dlidar_bridge.yaml"
    log = LogInfo(msg="using nautilus auv")

    # Publish /tf and /tf_static.w
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", ros_gz_bridge_config
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Relay - use instead of transform when Gazebo is only publishing odom -> base_link
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    event = RegisterEventHandler(
                OnProcessStart(
                    target_action=bridge,
                    on_start=[
                        topic_tools_tf
                    ]
                )
            )

    return [log, robot_state_publisher, bridge, event]

def generate_launch_arguments():
    """Generate a list of launch arguments"""
    return [
        DeclareLaunchArgument(
                "use_gz_tf", 
                default_value="true", 
                description="Use Gazebo TF."
            ),
        DeclareLaunchArgument(
            "model",
            default_value="orca_auv",
            description="Name or filepath of the model to load.",
        ),
        DeclareLaunchArgument(
            "name",
            default_value="AUV",
            description="Name for the model instance.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0",
            description="The intial 'x' position (m).",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0",
            description="The intial 'y' position (m).",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0",
            description="The intial 'z' position (m).",
        ),
        DeclareLaunchArgument(
            "R",
            default_value="0",
            description="The intial roll angle (radians).",
        ),
        DeclareLaunchArgument(
            "P",
            default_value="0",
            description="The intial pitch angle (radians).",
        ),
        DeclareLaunchArgument(
            "Y",
            default_value="0",
            description="The intial yaw angle (radians).",
        ),
        # MAVProxy fan-out ports
        DeclareLaunchArgument(
            "qgc_out", default_value="14550", description="UDP port for QGC telemetry."
        ),
        DeclareLaunchArgument(
            "cmd_out",
            default_value="14551",
            description="UDP port for command script telemetry.",
        ),
        DeclareLaunchArgument(
            "dvl_out",
            default_value="14552",
            description="UDP port for DVL script telemetry.",
        ),
        DeclareLaunchArgument(
            "mon_out",
            default_value="14553",
            description="UDP port for monitor script telemetry.",
        ),
        # Core endpoints
        DeclareLaunchArgument(
            "master",
            default_value="tcp:127.0.0.1:5760",
            description="SITL MAVLink master endpoint (TCP).",
        ),
        DeclareLaunchArgument(
            "sitl",
            default_value="127.0.0.1:5501",
            description="SITL internal port (as expected by ardupilot_sitl).",
        ),
    ]

def generate_launch_description():
    launch_arguments = generate_launch_arguments()
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    # micro-ROS agent (same one sitl_dds_udp.launch.py includes)
    micro_ros_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ardupilot_sitl"), "launch", "micro_ros_agent.launch.py"]
                ),
            ]
        )
    )

    # Launch SITL only (no MAVProxy wrapper).
    sitl_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("ardupilot_sitl"), "launch", "sitl.launch.py"]
                ),
            ]
        ),
        launch_arguments={
            # Keep these aligned with your previous setup
            "transport": "udp4",
            "port": "2019",
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "command": "ardusub",
            "defaults": os.path.join(
                pkg_ardupilot_sitl, "config", "default_params", "sub-6dof.parm"
            )
            + ","
            + os.path.join(
                pkg_ardupilot_sitl, "config", "default_params", "dds_udp.parm"
            ),
            "sim_address": "127.0.0.1",
            "master": LaunchConfiguration("master"),
            "sitl": LaunchConfiguration("sitl"),
            # IMPORTANT: don't set SITL "out" here; MAVProxy will be the fan-out hub.
        }.items(),
    )

    # 2) Launch MAVProxy manually with multiple --out endpoints.
    #    We delay it slightly to let SITL bind tcp:5760 first.
    mavproxy_multi_out = ExecuteProcess(
        cmd=[
            "mavproxy.py",
            "--master",
            LaunchConfiguration("master"),
            "--out",
            ["udp:127.0.0.1:", LaunchConfiguration("qgc_out")],
            "--out",
            ["udp:127.0.0.1:", LaunchConfiguration("cmd_out")],
            "--out",
            ["udp:127.0.0.1:", LaunchConfiguration("dvl_out")],
            "--out",
            ["udp:127.0.0.1:", LaunchConfiguration("mon_out")],
        ],
        output="screen",
    )

    # Ensure `SDF_PATH` includes Gazebo resource paths.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        if "SDF_PATH" in os.environ:
            os.environ["SDF_PATH"] = os.environ["SDF_PATH"] + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    opfunc_robot_state_publisher = OpaqueFunction(function=launch_state_pub_with_bridge)
    opfunc_spawn_robot = OpaqueFunction(function=launch_spawn_robot)

    ld = LaunchDescription(launch_arguments)
    ld.add_action(micro_ros_agent)
    ld.add_action(sitl_only)
    ld.add_action(mavproxy_multi_out)
    ld.add_action(opfunc_robot_state_publisher)
    ld.add_action(opfunc_spawn_robot)

    return ld