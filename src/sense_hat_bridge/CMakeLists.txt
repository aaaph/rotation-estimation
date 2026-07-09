cmake_minimum_required(VERSION 3.20)
project(sense_hat_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/imu_message.cpp
  src/imu_strategy_factory.cpp
  src/sense_hat_node.cpp
  src/stationary_imu_mock.cpp
  src/udp_reader_imu_strategy.cpp
  src/yaw_rotation_imu_mock.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library)

add_executable(sense_hat_node src/sense_hat_node_main.cpp)
target_link_libraries(sense_hat_node PRIVATE ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_lib sense_hat_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sense_hat_node test/test_sense_hat_node.cpp)
  target_link_libraries(test_sense_hat_node ${PROJECT_NAME}_lib)

  ament_add_gtest(test_imu_strategies test/test_imu_strategies.cpp)
  target_link_libraries(test_imu_strategies ${PROJECT_NAME}_lib)
endif()

ament_package()
