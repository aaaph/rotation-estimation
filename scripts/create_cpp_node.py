"""Create a small ROS 2 C++ package with one node and one gtest."""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import textwrap

CPP_STANDARD = "20"
DEFAULT_DESTINATION_DIRECTORY = pathlib.Path("src")
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

        #include "rclcpp/rclcpp.hpp"

        namespace {package} {{

        class {node_class} final : public rclcpp::Node {{
         public:
          static constexpr auto kDefaultNodeName = "{node}";

          explicit {node_class}(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        }};

        }}  // namespace {package}

        #endif  // {guard}
        """
    )


def node_template(package: str, node: str, node_class: str) -> str:
    """Return the C++ node implementation template."""
    return textwrap.dedent(
        f"""\
        #include "{package}/{node}.hpp"

        namespace {package} {{

        {node_class}::{node_class}(const rclcpp::NodeOptions& options)
            : rclcpp::Node(kDefaultNodeName, options) {{
          RCLCPP_INFO(get_logger(), "%s started", kDefaultNodeName);
        }}

        }}  // namespace {package}
        """
    )


def main_template(package: str, node: str, node_class: str) -> str:
    """Return the C++ executable entrypoint template."""
    return textwrap.dedent(
        f"""\
        #include <memory>

        #include "rclcpp/rclcpp.hpp"

        #include "{package}/{node}.hpp"

        int main(int argc, char** argv) {{
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
          EXPECT_EQ(std::string_view{{{package}::{node_class}::kDefaultNodeName}}, "{node}");
        }}
        """
    )


def cmake_template(package: str, node: str) -> str:
    """Return the CMakeLists.txt template."""
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
        find_package(rclcpp REQUIRED)

        add_library(${{PROJECT_NAME}}_lib src/{node}.cpp)
        target_include_directories(${{PROJECT_NAME}}_lib PUBLIC
          $<BUILD_INTERFACE:${{CMAKE_CURRENT_SOURCE_DIR}}/include>
          $<INSTALL_INTERFACE:include/${{PROJECT_NAME}}>)
        target_link_libraries(${{PROJECT_NAME}}_lib PUBLIC rclcpp::rclcpp)

        add_executable({node} src/{node}_main.cpp)
        target_link_libraries({node} PRIVATE ${{PROJECT_NAME}}_lib)

        install(TARGETS ${{PROJECT_NAME}}_lib {node}
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION lib/${{PROJECT_NAME}})

        install(DIRECTORY include/
          DESTINATION include)

        if(BUILD_TESTING)
          find_package(ament_cmake_gtest REQUIRED)
          find_package(ament_cmake_clang_format REQUIRED)
          find_package(ament_cmake_clang_tidy REQUIRED)

          set(REPOSITORY_ROOT "${{CMAKE_CURRENT_SOURCE_DIR}}/../..")
          set(CLANG_FORMAT_CONFIG "${{REPOSITORY_ROOT}}/.clang-format")
          set(CLANG_TIDY_CONFIG "${{REPOSITORY_ROOT}}/.clang-tidy")
          set(CLANG_TIDY_JOBS "4" CACHE STRING "Number of parallel clang-tidy jobs")

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

          <depend>rclcpp</depend>

          <test_depend>ament_cmake_gtest</test_depend>
          <test_depend>ament_cmake_clang_format</test_depend>
          <test_depend>ament_cmake_clang_tidy</test_depend>

          <export>
            <build_type>ament_cmake</build_type>
          </export>
        </package>
        """
    )


def target_files(destination_directory: pathlib.Path, package: str, node: str) -> dict[pathlib.Path, str]:
    """Return generated file paths and contents."""
    package_dir = destination_directory / package
    node_class = class_name_from_snake_case(node)

    return {
        package_dir / "CMakeLists.txt": cmake_template(package, node),
        package_dir / "package.xml": package_xml_template(package),
        package_dir / "include" / package / f"{node}.hpp": header_template(package, node, node_class),
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
    destination_directory = args.destination_directory

    try:
        validate_ros_name(package, "package")
        validate_ros_name(node, "node")

        package_dir = destination_directory / package
        if package_dir.exists():
            msg = f"Package directory already exists: {package_dir}"
            raise FileExistsError(msg)

        files = target_files(destination_directory, package, node)
        write_files(files)
    except (OSError, ValueError) as error:
        sys.stderr.write(f"Error: {error}\n")
        return 1

    sys.stdout.write(f"Created C++ ROS package `{package}` with node `{node}` in {package_dir}\n")
    sys.stdout.write(f"Run it with: ros2 run {package} {node}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
