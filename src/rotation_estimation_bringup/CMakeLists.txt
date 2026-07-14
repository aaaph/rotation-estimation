cmake_minimum_required(VERSION 3.20)
project(rotation_estimation_bringup)

find_package(ament_cmake REQUIRED)

# The workspace build passes this CMake option globally; this launch-only
# package has no compile targets, but still accepts the option quietly.
set(_compile_commands_requested "${CMAKE_EXPORT_COMPILE_COMMANDS}")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
