cmake_minimum_required(VERSION 3.20)
project(rotation_estimation_bringup)

if(APPLE AND EXISTS "/usr/bin/install_name_tool")
  set(CMAKE_INSTALL_NAME_TOOL "/usr/bin/install_name_tool" CACHE FILEPATH
    "Tool used to update Mach-O install names" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# The workspace build passes this CMake option globally; this launch-only
# package has no compile targets, but still accepts the option quietly.
set(_compile_commands_requested "${CMAKE_EXPORT_COMPILE_COMMANDS}")

add_library(${PROJECT_NAME}_lib src/manager_node.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  rclcpp::rclcpp)

add_executable(manager_node src/manager_main.cpp)
target_link_libraries(manager_node PRIVATE ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_lib manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetSystemMode.srv)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  rclcpp::rclcpp
  "${cpp_typesupport_target}")

ament_package()
