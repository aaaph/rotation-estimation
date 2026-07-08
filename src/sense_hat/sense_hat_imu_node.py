from __future__ import annotations

from typing import TYPE_CHECKING, cast

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from sense_hat.imu_strategy_factory import (
    MOCK_MODE_YAW_ROTATION,
    MOCK_MODES,
    STRATEGY_MODE_UDP_READER,
    ImuStrategyConfig,
    create_imu_strategy,
)

if TYPE_CHECKING:
    from rcl_interfaces.msg import ParameterValue

    from sense_hat.imu_strategy_protocol import ImuStrategy


def _parameter_default(value: object) -> ParameterValue:
    # rclpy accepts scalar defaults at runtime, but its overloads only expose ParameterValue.
    return cast("ParameterValue", value)


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
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        self.imu_strategy: ImuStrategy | None = None
        self.drain_strategy = not mock

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

            strategy_mode = self.mock_mode
            timer_period_s = 1.0 / rate_hz
            log_message = f"Publishing {self.mock_mode} mock IMU on {topic} at {rate_hz:.1f} Hz"
        else:
            strategy_mode = STRATEGY_MODE_UDP_READER
            timer_period_s = 0.001
            self.mock_angular_velocity_z = 0.0
            self.mock_mode = ""
            log_message = f"Listening UDP {host}:{port}, publishing {topic}"

        self.imu_strategy = create_imu_strategy(
            ImuStrategyConfig(
                mode=strategy_mode,
                host=host,
                port=port,
                frame_id=self.frame_id,
                stamp_factory=lambda: self.get_clock().now().to_msg(),
                angular_velocity_z=self.mock_angular_velocity_z,
            )
        )
        self.timer = self.create_timer(timer_period_s, self.publish_from_strategy)
        self.get_logger().info(log_message)

    def publish_from_strategy(self) -> None:
        """Publish IMU messages from the configured strategy."""
        if self.imu_strategy is None:
            return

        while True:
            msg = self.imu_strategy.get_imu_message()
            if msg is None:
                return

            self.pub.publish(msg)
            if not self.drain_strategy:
                return

    def destroy_node(self) -> None:
        """Destroy the node and close the IMU strategy."""
        if self.imu_strategy is not None:
            self.imu_strategy.close()
            self.imu_strategy = None
        super().destroy_node()


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
