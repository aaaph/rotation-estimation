from collections.abc import Mapping

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu

G = 9.80665
NumberLike = float | int | str


def new_imu_message(stamp: Time, frame_id: str) -> Imu:
    """Create a new IMU message."""
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.orientation_covariance[0] = -1.0
    return msg


def imu_message_from_sample(
    stamp: Time,
    frame_id: str,
    gyro: Mapping[str, NumberLike],
    accel_g: Mapping[str, NumberLike],
) -> Imu:
    """Create an IMU message from a sample."""
    msg = new_imu_message(stamp, frame_id)

    msg.angular_velocity.x = float(gyro["x"])
    msg.angular_velocity.y = float(gyro["y"])
    msg.angular_velocity.z = float(gyro["z"])

    msg.linear_acceleration.x = float(accel_g["x"]) * G
    msg.linear_acceleration.y = float(accel_g["y"]) * G
    msg.linear_acceleration.z = float(accel_g["z"]) * G

    return msg
