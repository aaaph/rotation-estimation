import pytest

from sense_hat.stationary_imu_mock import StationaryImuMock


class TestStationaryImuMock:
    """Test the StationaryImuMock class."""

    @pytest.fixture
    def stationary_imu_mock(self) -> StationaryImuMock:
        """Fixture for a stationary IMU mock."""
        return StationaryImuMock()

    def test_get_imu_message(self, stationary_imu_mock: StationaryImuMock) -> None:
        """Test the get_imu_message method."""
        imu_message = stationary_imu_mock.get_imu_message()
        assert imu_message.angular_velocity.z == 0.0
        assert imu_message.linear_acceleration.z == 9.81
