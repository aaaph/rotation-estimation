cmake_minimum_required(VERSION 3.20)
project(my_cpp_package)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(my_cpp_node_lib src/my_cpp_node_logic.cpp)
target_include_directories(my_cpp_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(my_cpp_node_lib PUBLIC cxx_std_17)

add_executable(my_cpp_node src/my_cpp_node.cpp)
target_include_directories(my_cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(my_cpp_node PRIVATE my_cpp_node_lib)
target_compile_features(my_cpp_node PUBLIC cxx_std_17)

install(TARGETS my_cpp_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_my_cpp_node test/test_my_cpp_node.cpp)
  target_link_libraries(test_my_cpp_node my_cpp_node_lib)
endif()

ament_package()
