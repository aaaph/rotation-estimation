"""Smoke and service tests for the manager node process."""

import signal
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the manager node with a stationary initial mode."""
    manager_node = launch_ros.actions.Node(
        package="rotation_estimation_bringup",
        executable="manager_node",
        name="manager_node",
        output="screen",
        parameters=[{"system_mode": "stationary"}],
    )

    return (
        launch.LaunchDescription(
            [
                manager_node,
                launch_testing.actions.ReadyToTest(),
            ],
        ),
        {"manager_node": manager_node},
    )


class TestManagerNodeLaunch(unittest.TestCase):
    """Startup check for the manager node process."""

    def test_process_starts(self, proc_output, manager_node):
        """Verify that the manager process reaches initialization."""
        proc_output.assertWaitFor(
            "Manager node initialized",
            process=manager_node,
            timeout=5.0,
        )


@launch_testing.post_shutdown_test()
class TestManagerNodeShutdown(unittest.TestCase):
    """Post-shutdown process checks for the manager node."""

    def test_exit_codes(self, proc_info):
        """Verify the manager process exits cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -int(signal.SIGINT)],
        )
