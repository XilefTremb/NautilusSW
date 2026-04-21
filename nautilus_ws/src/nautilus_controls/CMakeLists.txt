cmake_minimum_required(VERSION 3.8)
project(nautilus_controls)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package modules
ament_python_install_package("controls")

# Install executable Python scripts

install(
  PROGRAMS
    controls/dvl_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dvl_sensor_node
)

install(
  DIRECTORY
    controls/
  DESTINATION share/${PROJECT_NAME}/controls
)

# # Install resource index + package.xml
# install(
#   FILES
#     resource/${PROJECT_NAME}
#   DESTINATION share/ament_index/resource_index/packages
# )

install(
  FILES
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()