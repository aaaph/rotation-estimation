from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

from sense_hat.imu_strategy_protocol import ImuStrategy


class StationaryImuMock(ImuStrategy):
    """A mock IMU that is stationary."""

    def __init__(self, g: float = 9.81) -> None:
        """Initialize the mock IMU."""
        self.g = g

    def get_imu_message(self) -> Imu:
        """Get the IMU message."""
        angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        linear_acceleration = Vector3(x=0.0, y=0.0, z=self.g)
        return Imu(
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration,
        )
