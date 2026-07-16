"""Stationary identity integration test for the complete bringup launch file."""

import math
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
from tf2_msgs.msg import TFMessage

BRINGUP_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"
EXPECTED_WORLD_FRAME = "world"
EXPECTED_IMU_FRAME = "sensehat_link"
REQUIRED_TRANSFORM_COUNT = 5
QUATERNION_TOLERANCE = 1e-6


@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the complete system with a stationary mock IMU."""
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(BRINGUP_LAUNCH)),
        launch_arguments={
            "sense_hat_mock": "true",
            "system_mode": "stationary",
        }.items(),
    )

    return (
        launch.LaunchDescription(
            [
                bringup_launch,
                launch_testing.actions.ReadyToTest(),
            ],
        ),
        {},
    )


class TestStationaryBringupIdentity(unittest.TestCase):
    """Verify the complete stationary pipeline estimates identity rotation."""

    def setUp(self):
        """Create a helper node for observing EKF output."""
        rclpy.init()
        self.node = rclpy.create_node("bringup_stationary_identity_test")

    def tearDown(self):
        """Destroy the helper node."""
        self.node.destroy_node()
        rclpy.shutdown()

    def test_stationary_estimate_is_normalized_identity(self):
        """Wait for stationary estimates and verify their rotations."""
        estimates = []

        def record_estimates(message):
            estimates.extend(
                (
                    transform.header.frame_id,
                    transform.child_frame_id,
                    transform.transform.rotation,
                )
                for transform in message.transforms
            )

        subscription = self.node.create_subscription(
            TFMessage,
            "/tf",
            record_estimates,
            10,
        )

        try:
            deadline = time.monotonic() + 10.0
            while len(estimates) < REQUIRED_TRANSFORM_COUNT and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        finally:
            self.node.destroy_subscription(subscription)

        assert len(estimates) >= REQUIRED_TRANSFORM_COUNT
        for world_frame, imu_frame, quaternion in estimates[-REQUIRED_TRANSFORM_COUNT:]:
            assert world_frame == EXPECTED_WORLD_FRAME
            assert imu_frame == EXPECTED_IMU_FRAME
            norm = math.sqrt(quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2)
            assert math.isclose(norm, 1.0, abs_tol=QUATERNION_TOLERANCE)
            assert math.isclose(quaternion.x, 0.0, abs_tol=QUATERNION_TOLERANCE)
            assert math.isclose(quaternion.y, 0.0, abs_tol=QUATERNION_TOLERANCE)
            assert math.isclose(quaternion.z, 0.0, abs_tol=QUATERNION_TOLERANCE)
            assert math.isclose(abs(quaternion.w), 1.0, abs_tol=QUATERNION_TOLERANCE)


@launch_testing.post_shutdown_test()
class TestStationaryBringupShutdown(unittest.TestCase):
    """Verify all bringup processes stop cleanly."""

    def test_exit_codes(self, proc_info):
        """Accept normal exit and launch-testing SIGINT shutdown."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -int(signal.SIGINT)],
        )
