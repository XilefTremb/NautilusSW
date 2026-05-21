cmake_minimum_required(VERSION 3.8)
project(nautilus_controls)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package modules
ament_python_install_package("nautilus_controls")

# Install executable Python scripts

install(
  PROGRAMS
    nautilus_controls/cube_interface.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cube_interface
)

install(
  PROGRAMS
    nautilus_controls/pid_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid_node
)

install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    nautilus_controls/
  DESTINATION share/${PROJECT_NAME}/nautilus_controls
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