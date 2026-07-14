"""Smoke tests for the Sense HAT node launch file."""

import math
import signal
import tempfile
import time
import unittest
import uuid
from pathlib import Path

import launch
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

SENSE_HAT_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "sense_hat_node.launch.py"
TEST_TOPIC = f"/test/sense_hat_launch/run_{uuid.uuid4().hex}/imu_raw"
EXPECTED_FRAME_ID = "sense_hat_launch_test_link"
GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665
VALUE_TOLERANCE = 1e-12


def write_test_config():
    """Write a launch-test-specific params file with an isolated topic."""
    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".yaml", encoding="utf-8") as config_file:
        config_file.write(
            f"""sense_hat_imu_node:
  ros__parameters:
    mock: false
    mock_mode: yaw_rotation
    mock_angular_velocity_z: 0.5
    mock_yaw_oscillation_frequency_hz: 0.25
    host: 127.0.0.1
    port: 8765
    frame_id: {EXPECTED_FRAME_ID}
    topic: {TEST_TOPIC}
""",
        )
        return Path(config_file.name)


TEST_CONFIG = write_test_config()


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the Sense HAT node through its package launch file."""
    sense_hat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(SENSE_HAT_LAUNCH)),
        launch_arguments={
            "config": str(TEST_CONFIG),
            "mock": "true",
            "mock_mode": "stationary",
        }.items(),
    )

    return (
        launch.LaunchDescription(
            [
                SetLaunchConfiguration("mock", "true"),
                SetLaunchConfiguration("mock_mode", "stationary"),
                SetLaunchConfiguration("config", str(TEST_CONFIG)),
                sense_hat_launch,
                launch_testing.actions.ReadyToTest(),
            ],
        ),
        {},
    )


class TestSenseHatLaunchSmoke(unittest.TestCase):
    """Smoke tests for the launched Sense HAT node."""

    def setUp(self):
        """Create a ROS helper node for graph and topic checks."""
        rclpy.init()
        self.node = rclpy.create_node("sense_hat_launch_test")

    def tearDown(self):
        """Destroy the ROS helper node."""
        self.node.destroy_node()
        rclpy.shutdown()

    def wait_for_condition(self, condition, timeout_sec=5.0):
        """Spin until a condition becomes true or the timeout expires."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if condition():
                return True
        return condition()

    def test_node_is_visible_in_graph(self):
        """Verify that the launch file starts the expected node."""

        def node_is_visible():
            return any(
                node_name == "sense_hat_imu_node"
                for node_name, _namespace in self.node.get_node_names_and_namespaces()
            )

        assert node_is_visible() or self.wait_for_condition(node_is_visible)

    def test_stationary_mock_imu_is_published(self):
        """Verify that launch arguments configure a stationary mock IMU publisher."""
        messages = []
        subscription = self.node.create_subscription(
            Imu,
            TEST_TOPIC,
            messages.append,
            qos_profile_sensor_data,
        )

        try:
            assert self.wait_for_condition(lambda: bool(messages))
        finally:
            self.node.destroy_subscription(subscription)

        msg = messages[-1]
        assert msg.header.frame_id == EXPECTED_FRAME_ID
        assert msg.orientation_covariance[0] == -1.0
        assert msg.angular_velocity.x == 0.0
        assert msg.angular_velocity.y == 0.0
        assert msg.angular_velocity.z == 0.0
        assert msg.linear_acceleration.x == 0.0
        assert msg.linear_acceleration.y == 0.0
        assert math.isclose(
            msg.linear_acceleration.z,
            GRAVITY_METERS_PER_SECOND_SQUARED,
            abs_tol=VALUE_TOLERANCE,
        )


@launch_testing.post_shutdown_test()
class TestSenseHatLaunchShutdown(unittest.TestCase):
    """Post-shutdown process checks for the Sense HAT launch file."""

    def test_exit_codes(self, proc_info):
        """Verify launched processes exit cleanly."""
        try:
            launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -int(signal.SIGINT)])
        finally:
            TEST_CONFIG.unlink(missing_ok=True)
