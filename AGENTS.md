My general research direction is Autonomous Navigation.

Unit tests should live inside the folder of the node or package they cover.
For Python nodes, prefer `src/<node_or_package>/tests/unit/` over a top-level
`tests/unit/` directory.

ROS 2 launch tests should live in the covered package's `test/` directory and
be named `launch_test_<subject>.launch.py`. Keep the `launch_test_` prefix so
pytest does not collect these files; launch tests are owned by CTest/ament.

Register every launch test inside the package's `if(BUILD_TESTING)` block using
this CMake pattern:

```cmake
find_package(launch_testing_ament_cmake REQUIRED)

add_launch_test(test/launch_test_<subject>.launch.py
  TIMEOUT 30
  APPEND_ENV "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}"
  APPEND_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")

set_tests_properties(test_launch_test_<subject>.launch.py
  PROPERTIES ENVIRONMENT "RMW_IMPLEMENTATION=rmw_fastrtps_cpp")
```

Declare `launch`, `launch_ros`, `launch_testing`,
`launch_testing_ament_cmake`, and `rclpy` as `<test_depend>` entries in the
package's `package.xml` when the launch test imports them.

Launch tests run through `pixi run test-cpp`, because `add_launch_test`
registers them with CTest. After adding or changing a launch test, verify it
with `pixi run test-cpp`; do not move it under the `test-py` task.
