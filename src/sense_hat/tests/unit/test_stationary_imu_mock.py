import pytest
from builtin_interfaces.msg import Time

from sense_hat.imu_message import G
from sense_hat.stationary_imu_mock import StationaryImuMock


class TestStationaryImuMock:
    """Test the StationaryImuMock class."""

    @pytest.fixture
    def stationary_imu_mock(self) -> StationaryImuMock:
        """Fixture for a stationary IMU mock."""
        return StationaryImuMock(
            frame_id="sensehat_link",
            stamp_factory=lambda: Time(sec=1, nanosec=2),
        )

    def test_get_imu_message(self, stationary_imu_mock: StationaryImuMock) -> None:
        """Test the get_imu_message method."""
        imu_message = stationary_imu_mock.get_imu_message()
        assert imu_message.header.frame_id == "sensehat_link"
        assert imu_message.header.stamp.sec == 1
        assert imu_message.header.stamp.nanosec == 2
        assert imu_message.orientation_covariance[0] == -1.0
        assert imu_message.angular_velocity.z == 0.0
        assert imu_message.linear_acceleration.z == pytest.approx(G)
