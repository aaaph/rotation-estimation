"""Create a small ROS 2 C++ package with one node and one gtest."""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import textwrap

CPP_STANDARD = "20"
DEFAULT_CMAKE_STYLE = "modern"
DEFAULT_DESTINATION_DIRECTORY = pathlib.Path("src")
SUPPORTED_CMAKE_STYLES = ("modern", "ament")
ROS_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*[a-z0-9]$")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", help="New ROS package name, for example rotation_estimation_cpp.")
    parser.add_argument("node", help="New executable/node name, for example estimation_node.")
    parser.add_argument(
        "--destination-directory",
        default=DEFAULT_DESTINATION_DIRECTORY,
        type=pathlib.Path,
        help="Directory where the ROS package will be created.",
    )
    parser.add_argument(
        "--cmake-style",
        choices=SUPPORTED_CMAKE_STYLES,
        default=DEFAULT_CMAKE_STYLE,
        help="Dependency linking style to generate. Use `modern` for Jazzy+ target links.",
    )
    return parser.parse_args()


def validate_ros_name(value: str, label: str) -> None:
    """Validate a conservative ROS package or executable name."""
    if ROS_NAME_PATTERN.fullmatch(value) is None or "__" in value:
        msg = (
            f"{label} must be snake_case, start with a letter, end with a letter or digit, "
            "and contain only lowercase letters, digits, and single underscores."
        )
        raise ValueError(msg)


def class_name_from_snake_case(value: str) -> str:
    """Convert snake_case into PascalCase."""
    return "".join(part.capitalize() for part in value.split("_"))


def include_guard(package: str, node: str) -> str:
    """Return the include guard for the node header."""
    return f"{package}_{node}_HPP_".upper()


def header_template(package: str, node: str, node_class: str) -> str:
    """Return the C++ node header template."""
    guard = include_guard(package, node)
    return textwrap.dedent(
        f"""\
        #ifndef {guard}
        #define {guard}

        #include <chrono>
        #include <string>

        #include "rclcpp/node.hpp"
        #include "rclcpp/node_options.hpp"
        #include "rclcpp/publisher.hpp"
        #include "rclcpp/timer.hpp"
        #include "std_msgs/msg/string.hpp"

        namespace {package} {{

        class {node_class} final : public rclcpp::Node {{
        public:
          static constexpr auto kDefaultNodeName = "{node}";

          explicit {node_class}(
              const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        private:
          void publish_heartbeat();

          rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
          rclcpp::TimerBase::SharedPtr heartbeat_timer_;
          std::string frame_id_;
          std::chrono::nanoseconds heartbeat_period_{{std::chrono::seconds{{1}}}};
        }};

        }} // namespace {package}

        #endif // {guard}
        """
    )


def node_template(package: str, node: str, node_class: str) -> str:
    """Return the C++ node implementation template."""
    return textwrap.dedent(
        f"""\
        #include "{package}/{node}.hpp"

        #include <chrono>
        #include <string>
        #include <utility>

        #include "rcl_interfaces/msg/floating_point_range.hpp"
        #include "rcl_interfaces/msg/parameter_descriptor.hpp"
        #include "rclcpp/qos.hpp"

        namespace {package} {{
        namespace {{

        constexpr double kDefaultHeartbeatRateHz = 1.0;
        constexpr double kMinimumHeartbeatRateHz = 0.1;
        constexpr double kMaximumHeartbeatRateHz = 1000.0;

        rcl_interfaces::msg::ParameterDescriptor
        string_parameter_descriptor(const std::string &description) {{
          auto descriptor = rcl_interfaces::msg::ParameterDescriptor{{}};
          descriptor.description = description;
          return descriptor;
        }}

        rcl_interfaces::msg::ParameterDescriptor heartbeat_rate_descriptor() {{
          auto descriptor = rcl_interfaces::msg::ParameterDescriptor{{}};
          descriptor.description = "Heartbeat publication rate in Hz.";
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range[0].from_value = kMinimumHeartbeatRateHz;
          descriptor.floating_point_range[0].to_value = kMaximumHeartbeatRateHz;
          descriptor.floating_point_range[0].step = 0.0;
          return descriptor;
        }}

        }} // namespace

        {node_class}::{node_class}(const rclcpp::NodeOptions &options)
            : rclcpp::Node(kDefaultNodeName, options) {{
          const auto topic =
              declare_parameter("topic", std::string{{"heartbeat"}},
                                string_parameter_descriptor("Heartbeat output topic."));
          frame_id_ = declare_parameter(
              "frame_id", std::string{{"base_link"}},
              string_parameter_descriptor("Frame name reported in heartbeat payload."));
          const auto heartbeat_rate_hz =
              declare_parameter("heartbeat_rate_hz", kDefaultHeartbeatRateHz,
                                heartbeat_rate_descriptor());

          heartbeat_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>{{1.0 / heartbeat_rate_hz}});

          heartbeat_publisher_ =
              create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS{{1}}.reliable());
          heartbeat_timer_ =
              create_wall_timer(heartbeat_period_, [this]() {{ publish_heartbeat(); }});

          RCLCPP_INFO(get_logger(), "%s started, publishing heartbeat on %s",
                      kDefaultNodeName, topic.c_str());
        }}

        void {node_class}::publish_heartbeat() {{
          auto message = std_msgs::msg::String{{}};
          message.data = std::string{{kDefaultNodeName}} + " heartbeat from " + frame_id_;
          heartbeat_publisher_->publish(std::move(message));
        }}

        }} // namespace {package}
        """
    )


def launch_template(package: str, node: str) -> str:
    """Return the ROS 2 Python launch template."""
    return textwrap.dedent(
        f"""\
        from launch_ros.actions import Node
        from launch_ros.substitutions import FindPackageShare

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


        def generate_launch_description() -> LaunchDescription:
            \"\"\"Generate the node launch description.\"\"\"
            config_file = DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("{package}"), "config", "params.yaml"]
                ),
                description="Path to the node parameter file.",
            )
            use_sim_time = DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock.",
            )

            node = Node(
                package="{package}",
                executable="{node}",
                name="{node}",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {{"use_sim_time": LaunchConfiguration("use_sim_time")}},
                ],
            )

            return LaunchDescription([config_file, use_sim_time, node])
        """
    )


def launch_init_template() -> str:
    """Return the launch package marker template."""
    return ""


def params_template(node: str) -> str:
    """Return the default ROS 2 parameter file template."""
    return textwrap.dedent(
        f"""\
        {node}:
          ros__parameters:
            topic: heartbeat
            frame_id: base_link
            heartbeat_rate_hz: 1.0
        """
    )


def cmake_link_block(cmake_style: str) -> str:
    """Return the generated CMake dependency linking block."""
    if cmake_style == "modern":
        return textwrap.dedent(
            """\
            target_link_libraries(${PROJECT_NAME}_lib PUBLIC
              rclcpp::rclcpp
              ${rcl_interfaces_TARGETS}
              ${std_msgs_TARGETS})
            """
        )
    if cmake_style == "ament":
        return textwrap.dedent(
            """\
            ament_target_dependencies(${PROJECT_NAME}_lib
              rclcpp
              rcl_interfaces
              std_msgs)
            """
        )
    msg = f"Unsupported CMake style: {cmake_style}"
    raise ValueError(msg)


def main_template(package: str, node: str, node_class: str) -> str:
    """Return the C++ executable entrypoint template."""
    return textwrap.dedent(
        f"""\
        #include <memory>

        #include "rclcpp/rclcpp.hpp"

        #include "{package}/{node}.hpp"

        int main(int argc, char **argv) {{
          rclcpp::init(argc, argv);
          rclcpp::spin(std::make_shared<{package}::{node_class}>());
          rclcpp::shutdown();

          return 0;
        }}
        """
    )


def test_template(package: str, node: str, node_class: str) -> str:
    """Return the C++ gtest template."""
    return textwrap.dedent(
        f"""\
        #include <string_view>

        #include <gtest/gtest.h>

        #include "{package}/{node}.hpp"

        TEST({node_class}Test, UsesDefaultNodeName) {{
          EXPECT_EQ(std::string_view{{{package}::{node_class}::kDefaultNodeName}},
                    "{node}");
        }}
        """
    )


def cmake_template(package: str, node: str, cmake_style: str) -> str:
    """Return the CMakeLists.txt template."""
    link_block = textwrap.indent(cmake_link_block(cmake_style).rstrip(), " " * 8)
    return textwrap.dedent(
        f"""\
        cmake_minimum_required(VERSION 3.20)
        project({package})

        set(CMAKE_CXX_STANDARD {CPP_STANDARD})
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        set(CMAKE_CXX_EXTENSIONS OFF)

        if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
          add_compile_options(-Wall -Wextra -Wpedantic)
        endif()

        find_package(ament_cmake REQUIRED)
        find_package(rcl_interfaces REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)

        add_library(${{PROJECT_NAME}}_lib src/{node}.cpp)
        target_include_directories(${{PROJECT_NAME}}_lib PUBLIC
          $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}/include>
          $<INSTALL_INTERFACE:include/${{PROJECT_NAME}}>)
{link_block}

        add_executable({node} src/{node}_main.cpp)
        target_link_libraries({node} PRIVATE ${{PROJECT_NAME}}_lib)

        install(TARGETS ${{PROJECT_NAME}}_lib {node}
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION lib/${{PROJECT_NAME}})

        install(DIRECTORY include/
          DESTINATION include)

        install(DIRECTORY config launch
          DESTINATION share/${{PROJECT_NAME}})

        if(BUILD_TESTING)
          find_package(ament_cmake_gtest REQUIRED)
          find_package(ament_cmake_clang_format REQUIRED)
          find_package(ament_cmake_clang_tidy REQUIRED)

          set(REPOSITORY_ROOT "${{CMAKE_CURRENT_SOURCE_DIR}}/../..")
          set(CLANG_FORMAT_CONFIG "${{REPOSITORY_ROOT}}/.clang-format")
          set(CLANG_TIDY_CONFIG "${{REPOSITORY_ROOT}}/.clang-tidy")
          set(CLANG_TIDY_JOBS "4" CACHE STRING "Number of parallel clang-tidy jobs")
          find_program(AMENT_CLANG_FORMAT_BIN NAMES ament_clang_format REQUIRED)
          get_filename_component(AMENT_LINT_BIN_DIR "${{AMENT_CLANG_FORMAT_BIN}}" DIRECTORY)
          set(AMENT_LINT_TEST_ENV "PATH=${{AMENT_LINT_BIN_DIR}}:$ENV{{PATH}}")

          ament_clang_format(
            CONFIG_FILE "${{CLANG_FORMAT_CONFIG}}"
            include
            src
            test)
          ament_clang_tidy(
            CONFIG_FILE "${{CLANG_TIDY_CONFIG}}"
            HEADER_FILTER "src/.*"
            JOBS "${{CLANG_TIDY_JOBS}}"
            "${{CMAKE_CURRENT_BINARY_DIR}}")
          set_tests_properties(clang_format clang_tidy
            PROPERTIES ENVIRONMENT "${{AMENT_LINT_TEST_ENV}}")

          ament_add_gtest(test_{node} test/test_{node}.cpp)
          target_link_libraries(test_{node} ${{PROJECT_NAME}}_lib)
        endif()

        ament_package()
        """
    )


def package_xml_template(package: str) -> str:
    """Return the package.xml template."""
    return textwrap.dedent(
        f"""\
        <?xml version="1.0"?>
        <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
        <package format="3">
          <name>{package}</name>
          <version>0.0.0</version>
          <description>TODO: Package description</description>
          <maintainer email="nekaktotak2@gmail.com">Ivan Aphanasyev</maintainer>
          <license>TODO: License declaration</license>

          <buildtool_depend>ament_cmake</buildtool_depend>

          <depend>rcl_interfaces</depend>
          <depend>rclcpp</depend>
          <depend>std_msgs</depend>

          <exec_depend>launch</exec_depend>
          <exec_depend>launch_ros</exec_depend>

          <test_depend>ament_cmake_gtest</test_depend>
          <test_depend>ament_cmake_clang_format</test_depend>
          <test_depend>ament_cmake_clang_tidy</test_depend>

          <export>
            <build_type>ament_cmake</build_type>
          </export>
        </package>
        """
    )


def target_files(
    destination_directory: pathlib.Path,
    package: str,
    node: str,
    cmake_style: str,
) -> dict[pathlib.Path, str]:
    """Return generated file paths and contents."""
    package_dir = destination_directory / package
    node_class = class_name_from_snake_case(node)

    return {
        package_dir / "CMakeLists.txt": cmake_template(package, node, cmake_style),
        package_dir / "package.xml": package_xml_template(package),
        package_dir / "config" / "params.yaml": params_template(node),
        package_dir / "include" / package / f"{node}.hpp": header_template(package, node, node_class),
        package_dir / "launch" / "__init__.py": launch_init_template(),
        package_dir / "launch" / f"{node}.launch.py": launch_template(package, node),
        package_dir / "src" / f"{node}.cpp": node_template(package, node, node_class),
        package_dir / "src" / f"{node}_main.cpp": main_template(package, node, node_class),
        package_dir / "test" / f"test_{node}.cpp": test_template(package, node, node_class),
    }


def write_files(files: dict[pathlib.Path, str]) -> None:
    """Write generated files, refusing to overwrite anything."""
    existing_files = [path for path in files if path.exists()]
    if existing_files:
        existing = "\n".join(f"  - {path}" for path in existing_files)
        msg = f"Refusing to overwrite existing files:\n{existing}"
        raise FileExistsError(msg)

    for path, content in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")


def run() -> int:
    """Create a ROS 2 C++ node package."""
    args = parse_args()
    package = args.package
    node = args.node
    cmake_style = args.cmake_style
    destination_directory = args.destination_directory

    try:
        validate_ros_name(package, "package")
        validate_ros_name(node, "node")

        package_dir = destination_directory / package
        if package_dir.exists():
            msg = f"Package directory already exists: {package_dir}"
            raise FileExistsError(msg)

        files = target_files(destination_directory, package, node, cmake_style)
        write_files(files)
    except (OSError, ValueError) as error:
        sys.stderr.write(f"Error: {error}\n")
        return 1

    sys.stdout.write(f"Created C++ ROS package `{package}` with node `{node}` in {package_dir}\n")
    sys.stdout.write(f"Generated CMake style: {cmake_style}\n")
    sys.stdout.write(f"Run it with: ros2 run {package} {node}\n")
    sys.stdout.write(f"Or launch it with: ros2 launch {package} {node}.launch.py\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
