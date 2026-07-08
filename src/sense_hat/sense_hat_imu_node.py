# src/sense_hat/sense_hat_imu_node.py
import json
import socket
from collections.abc import Mapping
from typing import cast

from builtin_interfaces.msg import Time
import rclpy
from rcl_interfaces.msg import ParameterValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

G = 9.80665
NumberLike = float | int | str


def _parameter_default(value: object) -> ParameterValue:
    # rclpy accepts scalar defaults at runtime, but its overloads only expose ParameterValue.
    return cast(ParameterValue, value)


def new_imu_message(stamp: Time, frame_id: str) -> Imu:
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
    msg = new_imu_message(stamp, frame_id)

    msg.angular_velocity.x = float(gyro["x"])
    msg.angular_velocity.y = float(gyro["y"])
    msg.angular_velocity.z = float(gyro["z"])

    msg.linear_acceleration.x = float(accel_g["x"]) * G
    msg.linear_acceleration.y = float(accel_g["y"]) * G
    msg.linear_acceleration.z = float(accel_g["z"]) * G

    return msg


def mock_imu_message(stamp: Time, frame_id: str, angular_velocity_z: float) -> Imu:
    return imu_message_from_sample(
        stamp,
        frame_id,
        gyro={"x": 0.0, "y": 0.0, "z": angular_velocity_z},
        accel_g={"x": 0.0, "y": 0.0, "z": 1.0},
    )


class SenseHatUdpImuNode(Node):
    def __init__(self):
        super().__init__("sensehat_udp_imu_node")

        self.declare_parameter("mock", _parameter_default(False))
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

            self.mock_angular_velocity_z = float(
                self.get_parameter("mock_angular_velocity_z").value
            )
            self.timer = self.create_timer(1.0 / rate_hz, self.publish_mock)
            self.get_logger().info(
                f"Publishing mock IMU on {topic} at {rate_hz:.1f} Hz"
            )
            return

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(f"Listening UDP {host}:{port}, publishing {topic}")

    def publish_mock(self):
        msg = mock_imu_message(
            self.get_clock().now().to_msg(),
            self.frame_id,
            self.mock_angular_velocity_z,
        )
        self.pub.publish(msg)

    def poll(self):
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


def main():
    rclpy.init()
    node = SenseHatUdpImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
