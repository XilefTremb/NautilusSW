cmake_minimum_required(VERSION 3.5)
project(nautilus_mission)

# --------------------------------------------------------------------------- #
# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# --------------------------------------------------------------------------- #
#  Install.

# Install project launch files.
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# install(
#   DIRECTORY
#     enums/
#   DESTINATION share/${PROJECT_NAME}/enums
# )

install(
  DIRECTORY
    nautilus_mission/
  DESTINATION share/${PROJECT_NAME}/nautilus_mission
)

install(
  PROGRAMS
    nautilus_mission/master_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME master_node
)

install(
  PROGRAMS
    nautilus_mission/rosbag_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rosbag_recorder
)

ament_python_install_package(${PROJECT_NAME})
ament_python_install_package(enums)

install(
  FILES
    package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
