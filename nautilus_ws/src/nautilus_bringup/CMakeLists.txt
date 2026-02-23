cmake_minimum_required(VERSION 3.5)
project(nautilus_bringup)

# --------------------------------------------------------------------------- #
# Find dependencies.
find_package(ament_cmake REQUIRED)

# --------------------------------------------------------------------------- #
#  Install.

# Install project launch files.
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files.
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)


install(
  DIRECTORY
    nautilus_auv/
  DESTINATION share/${PROJECT_NAME}/nautilus_auv
)

install(
  DIRECTORY
    worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
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
