from collections.abc import Callable

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

from sense_hat.imu_message import G, new_imu_message
from sense_hat.imu_strategy_protocol import ImuStrategy


def _default_stamp() -> Time:
    return Time()


class YawRotationImuMock(ImuStrategy):
    """A mock IMU that rotates around the yaw axis."""

    def __init__(
        self,
        frame_id: str = "",
        stamp_factory: Callable[[], Time] | None = None,
        angular_velocity_z: float = 0.25,
        g: float = G,
    ) -> None:
        """Initialize the mock IMU."""
        self.frame_id = frame_id
        self.stamp_factory = stamp_factory if stamp_factory is not None else _default_stamp
        self.angular_velocity_z = angular_velocity_z
        self.g = g

    def get_imu_message(self) -> Imu:
        """Get the IMU message."""
        msg = new_imu_message(self.stamp_factory(), self.frame_id)
        msg.angular_velocity = Vector3(x=0.0, y=0.0, z=self.angular_velocity_z)
        msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=self.g)
        return msg

    def close(self) -> None:
        """Release strategy resources."""
