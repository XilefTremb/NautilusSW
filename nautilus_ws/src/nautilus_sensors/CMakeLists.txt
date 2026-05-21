cmake_minimum_required(VERSION 3.8)
project(nautilus_sensors)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package modules
ament_python_install_package("DVL")
ament_python_install_package("vision")

# Install executable Python scripts
install(
  PROGRAMS
    DVL/fake_dvl.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_dvl
)

install(
  PROGRAMS
    DVL/dvl_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dvl_sensor_node
)

install(
  PROGRAMS
    vision/stream.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stream
)

install(
  PROGRAMS
    vision/yolo_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_pipeline
)

install(
  DIRECTORY
    DVL/
  DESTINATION share/${PROJECT_NAME}/DVl
)

install(
  DIRECTORY
    vision/
  DESTINATION share/${PROJECT_NAME}/vision
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