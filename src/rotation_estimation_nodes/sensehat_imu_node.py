"""Publish raw Sense HAT IMU samples as sensor_msgs/Imu."""

from __future__ import annotations

import math
import time
from typing import Protocol

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

STANDARD_GRAVITY_MPS2 = 9.80665


class SenseHatLike(Protocol):
    def get_gyroscope_raw(self) -> dict[str, float]:
        ...

    def get_accelerometer_raw(self) -> dict[str, float]:
        ...


class MockSenseHat:
    def get_gyroscope_raw(self) -> dict[str, float]:
        now = time.monotonic()
        return {"x": 0.0, "y": 0.0, "z": 0.25 * math.sin(now)}

    def get_accelerometer_raw(self) -> dict[str, float]:
        return {"x": 0.0, "y": 0.0, "z": 1.0}


def make_sense_hat(mock: bool) -> SenseHatLike:
    if mock:
        return MockSenseHat()

    try:
        from sense_hat import SenseHat
    except ImportError as exc:
        raise RuntimeError(
            "The sense_hat Python package is not available in this Pixi environment. "
            "Install it for pi-runtime or run with '--ros-args -p mock:=true'."
        ) from exc

    sense = SenseHat()
    sense.set_imu_config(False, True, True)
    return sense


class SenseHatImuNode(Node):
    def __init__(self) -> None:
        super().__init__("sensehat_imu_node")

        self.declare_parameter("topic", "/sensehat/imu_raw")
        self.declare_parameter("frame_id", "sensehat_link")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("mock", False)

        self._topic = self.get_parameter("topic").value
        self._frame_id = self.get_parameter("frame_id").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        mock = bool(self.get_parameter("mock").value)

        self._sense = make_sense_hat(mock)
        self._publisher = self.create_publisher(Imu, self._topic, qos_profile_sensor_data)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_sample)

        self.get_logger().info(
            f"Publishing Sense HAT IMU samples on {self._topic} at {rate_hz:g} Hz"
        )

    def _publish_sample(self) -> None:
        gyro = self._sense.get_gyroscope_raw()
        accel = self._sense.get_accelerometer_raw()

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(gyro["x"])
        msg.angular_velocity.y = float(gyro["y"])
        msg.angular_velocity.z = float(gyro["z"])

        msg.linear_acceleration.x = float(accel["x"]) * STANDARD_GRAVITY_MPS2
        msg.linear_acceleration.y = float(accel["y"]) * STANDARD_GRAVITY_MPS2
        msg.linear_acceleration.z = float(accel["z"]) * STANDARD_GRAVITY_MPS2

        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SenseHatImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
