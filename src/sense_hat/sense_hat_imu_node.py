# src/sense_hat/sense_hat_imu_node.py
import json
import socket
from collections.abc import Mapping
from typing import cast

import rclpy
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import ParameterValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

G = 9.80665
NumberLike = float | int | str
MOCK_MODE_STATIONARY = "stationary"
MOCK_MODE_YAW_ROTATION = "yaw_rotation"
MOCK_MODES = {MOCK_MODE_STATIONARY, MOCK_MODE_YAW_ROTATION}


def _parameter_default(value: object) -> ParameterValue:
    # rclpy accepts scalar defaults at runtime, but its overloads only expose ParameterValue.
    return cast("ParameterValue", value)


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


def mock_imu_message(
    stamp: Time,
    frame_id: str,
    mode: str,
    angular_velocity_z: float,
) -> Imu:
    """Create a mock IMU message."""
    if mode == MOCK_MODE_STATIONARY:
        angular_velocity_z = 0.0
    elif mode != MOCK_MODE_YAW_ROTATION:
        msg = f"Unsupported mock_mode '{mode}'. Use one of {sorted(MOCK_MODES)}."
        raise ValueError(msg)

    return imu_message_from_sample(
        stamp,
        frame_id,
        gyro={"x": 0.0, "y": 0.0, "z": angular_velocity_z},
        accel_g={"x": 0.0, "y": 0.0, "z": 1.0},
    )


class SenseHatUdpImuNode(Node):
    """A node that publishes IMU data from a SenseHat UDP interface."""

    def __init__(self) -> None:
        """Initialize the node."""
        super().__init__("sensehat_udp_imu_node")

        self.declare_parameter("mock", _parameter_default(value=False))
        self.declare_parameter("mock_mode", _parameter_default(MOCK_MODE_YAW_ROTATION))
        self.declare_parameter("mock_rate_hz", _parameter_default(50.0))
        self.declare_parameter("mock_angular_velocity_z", _parameter_default(0.25))
        self.declare_parameter("host", _parameter_default("127.0.0.1"))
        self.declare_parameter("port", _parameter_default(8765))
        self.declare_parameter("frame_id", _parameter_default("sensehat_link"))
        self.declare_parameter("topic", _parameter_default("/sensehat/imu_raw"))

        mock = bool(self.get_parameter("mock").value)
        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("topic").value

        self.pub = self.create_publisher(Imu, topic, qos_profile_sensor_data)

        if mock:
            rate_hz = float(self.get_parameter("mock_rate_hz").value)
            if rate_hz <= 0.0:
                raise ValueError("mock_rate_hz must be greater than zero")

            self.mock_angular_velocity_z = float(self.get_parameter("mock_angular_velocity_z").value)
            self.mock_mode = str(self.get_parameter("mock_mode").value)
            if self.mock_mode not in MOCK_MODES:
                msg = f"Unsupported mock_mode '{self.mock_mode}'. Use one of {sorted(MOCK_MODES)}."
                raise ValueError(msg)

            self.timer = self.create_timer(1.0 / rate_hz, self.publish_mock)
            self.get_logger().info(f"Publishing {self.mock_mode} mock IMU on {topic} at {rate_hz:.1f} Hz")
            return

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.0)

        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(f"Listening UDP {host}:{port}, publishing {topic}")

    def publish_mock(self) -> None:
        """Publish a mock IMU message."""
        msg = mock_imu_message(
            self.get_clock().now().to_msg(),
            self.frame_id,
            self.mock_mode,
            self.mock_angular_velocity_z,
        )
        self.pub.publish(msg)

    def poll(self) -> None:
        """Poll the UDP socket for IMU data."""
        while True:
            try:
                data, _addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                return

            packet = json.loads(data.decode("utf-8"))
            gyro = packet["gyro"]
            accel = packet["accel"]

            msg = imu_message_from_sample(
                self.get_clock().now().to_msg(),
                self.frame_id,
                gyro,
                accel,
            )
            self.pub.publish(msg)


def main() -> None:
    """Start the node."""
    rclpy.init()
    node = SenseHatUdpImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
