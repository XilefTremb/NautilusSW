#!/bin/bash
export PATH=$PATH:/home/devs/ardu_ws/install/ardupilot_sitl/bin:/opt/ros/humble/bin:/usr/lib/ccache:/home/devs/ardupilot/Tools/autotest:/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:/home/devs/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin:/home/devs/ardu_ws/src/Micro-XRCE-DDS-Gen/scripts
export PYTHONPATH=$PYTHONPATH:/home/devs/NautilusSW/nautilus_ws/install/nautilus_mission/local/lib/python3.10/dist-packages:/home/devs/NautilusSW/nautilus_ws/install/nautilus_sensors/local/lib/python3.10/dist-packages:/home/devs/NautilusSW/nautilus_ws/install/nautilus_bringup/local/lib/python3.10/dist-packages:/home/devs/ardu_ws/install/ros_ign_interfaces/local/lib/python3.10/dist-packages:/home/devs/ardu_ws/install/ros_gz_interfaces/local/lib/python3.10/dist-packages:/home/devs/ardu_ws/install/ardupilot_dds_tests/lib/python3.10/site-packages:/home/devs/ardu_ws/install/ardupilot_sitl/local/lib/python3.10/dist-packages:/home/devs/ardu_ws/install/ardupilot_msgs/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages

source home/devs/NautilusSW/nautilus_ws/install/setup.bash
source /opt/ros/humble/setup.bash

ros2 launch nautilus_bringup mavproxy_router.launch.py &

wait
