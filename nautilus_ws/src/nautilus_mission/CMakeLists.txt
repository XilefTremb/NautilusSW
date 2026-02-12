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

install(
  PROGRAMS
    nautilus_mission/test_mission.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_mission
)

ament_python_install_package(${PROJECT_NAME})

# --------------------------------------------------------------------------- #
#  Build tests.

if(BUILD_TESTING)
  # Override default flake8 configuration.
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_SOURCE_DIR}/.flake8)

  # Add linters.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------------------- #
#  Call last.

ament_package()
