"""Smoke tests for the EKF node launch file."""

import signal
import time
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

EKF_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "ekf_node.launch.py"


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the EKF node through its package launch file."""
    ekf_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(EKF_LAUNCH)))

    return (
        launch.LaunchDescription(
            [
                ekf_launch,
                launch_testing.actions.ReadyToTest(),
            ],
        ),
        {},
    )


class TestEkfNodeLaunchSmoke(unittest.TestCase):
    """Smoke tests for the launched EKF node."""

    def setUp(self):
        """Create a ROS helper node for graph checks."""
        rclpy.init()
        self.node = rclpy.create_node("ekf_node_launch_test")

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
        """Verify the launch file starts /ekf_node."""

        def node_is_visible():
            return any(
                node_name == "ekf_node" and namespace == "/"
                for node_name, namespace in self.node.get_node_names_and_namespaces()
            )

        assert node_is_visible() or self.wait_for_condition(node_is_visible)


@launch_testing.post_shutdown_test()
class TestEkfNodeLaunchShutdown(unittest.TestCase):
    """Post-shutdown process checks for the EKF launch file."""

    def test_exit_codes(self, proc_info):
        """Verify the launched process exits cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -int(signal.SIGINT)],
        )
