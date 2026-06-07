cmake_minimum_required(VERSION 3.8)
project(nautilus_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTargetDepth.srv"
)

ament_python_install_module(nautilus_services.py)

ament_package()