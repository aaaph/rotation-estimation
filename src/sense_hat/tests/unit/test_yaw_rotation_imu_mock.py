import pytest
from builtin_interfaces.msg import Time

from sense_hat.imu_message import G
from sense_hat.yaw_rotation_imu_mock import YawRotationImuMock


class TestYawRotationImuMock:
    """Test the YawRotationImuMock class."""

    @pytest.fixture
    def yaw_rotation_imu_mock(self) -> YawRotationImuMock:
        """Fixture for a yaw rotation IMU mock."""
        return YawRotationImuMock(
            frame_id="sensehat_link",
            stamp_factory=lambda: Time(sec=1, nanosec=2),
        )

    def test_get_imu_message(self, yaw_rotation_imu_mock: YawRotationImuMock) -> None:
        """Test the get_imu_message method."""
        imu_message = yaw_rotation_imu_mock.get_imu_message()
        assert imu_message.header.frame_id == "sensehat_link"
        assert imu_message.header.stamp.sec == 1
        assert imu_message.header.stamp.nanosec == 2
        assert imu_message.orientation_covariance[0] == -1.0
        assert imu_message.angular_velocity.x == 0.0
        assert imu_message.angular_velocity.y == 0.0
        assert imu_message.angular_velocity.z == 0.25
        assert imu_message.linear_acceleration.x == 0.0
        assert imu_message.linear_acceleration.y == 0.0
        assert imu_message.linear_acceleration.z == pytest.approx(G)

    def test_get_imu_message_uses_configured_yaw_rate(self) -> None:
        """Test the configured yaw rate."""
        imu_message = YawRotationImuMock(angular_velocity_z=1.2).get_imu_message()
        assert imu_message.angular_velocity.z == 1.2
