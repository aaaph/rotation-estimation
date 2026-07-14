from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    config_file = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution([FindPackageShare("sense_hat_bridge"), "config", "params.yaml"]),
        description="Path to the node parameter file.",
    )
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )
    mock = DeclareLaunchArgument(
        "mock",
        default_value="false",
        description="Use a synthetic IMU source.",
    )
    mock_mode = DeclareLaunchArgument(
        "mock_mode",
        default_value="yaw_rotation",
        description="Synthetic IMU mode used when mock is true.",
    )

    node = Node(
        package="sense_hat_bridge",
        executable="sense_hat_node",
        name="sense_hat_imu_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"mock": LaunchConfiguration("mock")},
            {"mock_mode": LaunchConfiguration("mock_mode")},
        ],
    )

    return LaunchDescription([config_file, use_sim_time, mock, mock_mode, node])
