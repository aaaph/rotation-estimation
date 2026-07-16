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

get_property(PACKAGE_TARGETS DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(PACKAGE_TARGET IN LISTS PACKAGE_TARGETS)
  if(PACKAGE_TARGET MATCHES "^${PROJECT_NAME}(_s)?__")
    set_property(TARGET ${PACKAGE_TARGET} PROPERTY EXPORT_COMPILE_COMMANDS OFF)
  endif()
endforeach()

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  rclcpp::rclcpp
  "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  set(CLANG_FORMAT_CONFIG "${REPOSITORY_ROOT}/.clang-format")
  set(CLANG_TIDY_CONFIG "${REPOSITORY_ROOT}/.clang-tidy")
  set(CLANG_TIDY_JOBS "4" CACHE STRING "Number of parallel clang-tidy jobs")
  set(TEST_ROS_LOG_DIR "${CMAKE_CURRENT_BINARY_DIR}/ros_logs")
  file(MAKE_DIRECTORY "${TEST_ROS_LOG_DIR}")
  set(TEST_ROS_ENVIRONMENT
    "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    "ROS_DOMAIN_ID=73"
    "ROS_LOG_DIR=${TEST_ROS_LOG_DIR}")
  find_program(AMENT_CLANG_FORMAT_BIN NAMES ament_clang_format REQUIRED)
  get_filename_component(AMENT_LINT_BIN_DIR "${AMENT_CLANG_FORMAT_BIN}" DIRECTORY)
  set(AMENT_LINT_TEST_ENV "PATH=${AMENT_LINT_BIN_DIR}:$ENV{PATH}")

  ament_clang_format(
    CONFIG_FILE "${CLANG_FORMAT_CONFIG}"
    include
    src
    test)
  ament_clang_tidy(
    CONFIG_FILE "${CLANG_TIDY_CONFIG}"
    HEADER_FILTER "src/.*"
    JOBS "${CLANG_TIDY_JOBS}"
    "${CMAKE_CURRENT_BINARY_DIR}")
  set_tests_properties(clang_format clang_tidy
    PROPERTIES ENVIRONMENT "${AMENT_LINT_TEST_ENV}")

  ament_add_gtest(test_manager_node test/test_manager_node.cpp)
  target_link_libraries(test_manager_node ${PROJECT_NAME}_lib)
  set_tests_properties(test_manager_node
    PROPERTIES ENVIRONMENT "${TEST_ROS_ENVIRONMENT}")

  add_launch_test(test/launch_test_manager_node.launch.py
    TIMEOUT 30
    APPEND_ENV "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}"
    APPEND_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")

  set_tests_properties(test_launch_test_manager_node.launch.py
    PROPERTIES ENVIRONMENT "${TEST_ROS_ENVIRONMENT}")
endif()

ament_package()
