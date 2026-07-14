from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Launch the Sense HAT IMU bridge and rotation estimator together."""
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )
    sense_hat_config = DeclareLaunchArgument(
        "sense_hat_config",
        default_value=PathJoinSubstitution([FindPackageShare("sense_hat_bridge"), "config", "params.yaml"]),
        description="Path to the Sense HAT bridge parameter file.",
    )
    sense_hat_mock = DeclareLaunchArgument(
        "sense_hat_mock",
        default_value="false",
        description="Run the Sense HAT bridge with a synthetic IMU source.",
    )
    sense_hat_mock_mode = DeclareLaunchArgument(
        "sense_hat_mock_mode",
        default_value="yaw_rotation",
        description="Synthetic IMU mode used when sense_hat_mock is true.",
    )
    ekf_config = DeclareLaunchArgument(
        "ekf_config",
        default_value=PathJoinSubstitution([FindPackageShare("rotation_estimation"), "config", "params.yaml"]),
        description="Path to the rotation estimator parameter file.",
    )

    sense_hat_node = Node(
        package="sense_hat_bridge",
        executable="sense_hat_node",
        name="sense_hat_imu_node",
        output="screen",
        parameters=[
            LaunchConfiguration("sense_hat_config"),
            {"mock": LaunchConfiguration("sense_hat_mock")},
            {"mock_mode": LaunchConfiguration("sense_hat_mock_mode")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    ekf_node = Node(
        package="rotation_estimation",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[
            LaunchConfiguration("ekf_config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription(
        [
            use_sim_time,
            sense_hat_config,
            sense_hat_mock,
            sense_hat_mock_mode,
            ekf_config,
            sense_hat_node,
            ekf_node,
        ]
    )
