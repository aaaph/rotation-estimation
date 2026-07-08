from typing import Protocol

from sensor_msgs.msg import Imu


class ImuStrategy(Protocol):
    """A protocol for IMU strategies."""

    def get_imu_message(self) -> Imu:
        """Get the IMU data."""
