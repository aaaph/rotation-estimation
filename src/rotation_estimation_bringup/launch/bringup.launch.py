from enum import StrEnum

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


class SystemMode(StrEnum):
    """System mode enum."""

    STATIONARY = "stationary"
    MOBILE = "mobile"


class MockStrategy(StrEnum):
    """Mobile mock IMU strategies."""

    YAW_ROTATION = "yaw_rotation"
    YAW_OSCILLATION = "yaw_oscillation"


def get_ekf_node_mode_from_system_mode(system_mode: SystemMode) -> str:
    """Get the EKF node mode from the system mode."""
    return "stationary" if system_mode == SystemMode.STATIONARY else "accel"


def get_sense_hat_node_mode_from_system_mode(
    system_mode: SystemMode, mobile_mock_strategy: MockStrategy
) -> str:
    """Get the Sense HAT node mode from the system mode."""
    return "stationary" if system_mode == SystemMode.STATIONARY else mobile_mock_strategy.value


def launch_nodes(context: LaunchContext) -> list[Node]:
    """Create nodes after launch arguments have been resolved."""
    system_mode = SystemMode(LaunchConfiguration("system_mode").perform(context))
    sense_hat_mock = LaunchConfiguration("sense_hat_mock").perform(context) == "true"
    mobile_mock_strategy = MockStrategy(
        LaunchConfiguration("mobile_mock_strategy").perform(context)
    )

    sense_hat_node = Node(
        package="sense_hat_bridge",
        executable="sense_hat_node",
        name="sense_hat_imu_node",
        output="screen",
        parameters=[
            LaunchConfiguration("sense_hat_config"),
            {"mock": sense_hat_mock},
            {
                "mock_mode": get_sense_hat_node_mode_from_system_mode(
                    system_mode, mobile_mock_strategy
                )
            },
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
            {"mode": get_ekf_node_mode_from_system_mode(system_mode)},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    manager_node = Node(
        package="rotation_estimation_bringup",
        executable="manager_node",
        name="manager_node",
        output="screen",
        parameters=[
            LaunchConfiguration("manager_config"),
            {"system_mode": system_mode.value},
            {"mock_imu": sense_hat_mock},
            {"mobile_mock_strategy": mobile_mock_strategy.value},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return [sense_hat_node, ekf_node, manager_node]


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
        choices=["false", "true"],
    )
    mobile_mock_strategy = DeclareLaunchArgument(
        "mobile_mock_strategy",
        default_value=MockStrategy.YAW_ROTATION.value,
        description="Synthetic IMU strategy used in mobile system mode.",
        choices=[strategy.value for strategy in MockStrategy],
    )
    ekf_config = DeclareLaunchArgument(
        "ekf_config",
        default_value=PathJoinSubstitution([FindPackageShare("rotation_estimation"), "config", "params.yaml"]),
        description="Path to the rotation estimator parameter file.",
    )
    manager_config = DeclareLaunchArgument(
        "manager_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rotation_estimation_bringup"), "config", "params.yaml"]
        ),
        description="Path to the manager parameter file.",
    )

    system_mode = DeclareLaunchArgument(
        "system_mode",
        default_value=SystemMode.STATIONARY.value,
        description="System mode.",
        choices=[mode.value for mode in SystemMode],
    )

    return LaunchDescription(
        [
            use_sim_time,
            sense_hat_config,
            sense_hat_mock,
            mobile_mock_strategy,
            ekf_config,
            manager_config,
            system_mode,
            OpaqueFunction(function=launch_nodes),
        ]
    )
