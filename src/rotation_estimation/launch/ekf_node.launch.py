from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Generate the node launch description."""
    config_file = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution([FindPackageShare("rotation_estimation"), "config", "params.yaml"]),
        description="Path to the node parameter file.",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )
    mode = DeclareLaunchArgument(
        "mode",
        default_value="accel",
        description="Rotation estimator mode.",
    )

    node = Node(
        package="rotation_estimation",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"mode": LaunchConfiguration("mode")},
        ],
    )

    return LaunchDescription([config_file, use_sim_time, mode, node])
