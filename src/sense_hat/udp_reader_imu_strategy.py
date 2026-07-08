import json
import socket
from collections.abc import Callable
from typing import Protocol, cast

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu

from sense_hat.imu_message import imu_message_from_sample
from sense_hat.imu_strategy_protocol import ImuStrategy


class DatagramSocket(Protocol):
    """Minimal UDP socket interface used by the UDP reader strategy."""

    def bind(self, address: tuple[str, int]) -> None:
        """Bind the socket."""

    def settimeout(self, value: float) -> None:
        """Set socket timeout."""

    def recvfrom(self, bufsize: int) -> tuple[bytes, object]:
        """Receive a UDP datagram."""

    def close(self) -> None:
        """Close the socket."""


class UdpReaderImuStrategy(ImuStrategy):
    """Read IMU messages from a non-blocking UDP socket."""

    def __init__(
        self,
        host: str,
        port: int,
        frame_id: str,
        stamp_factory: Callable[[], Time],
        sock: DatagramSocket | None = None,
    ) -> None:
        """Initialize the UDP reader strategy."""
        self.frame_id = frame_id
        self.stamp_factory = stamp_factory
        self.sock: DatagramSocket = (
            sock if sock is not None else cast("DatagramSocket", socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
        )
        self.sock.bind((host, port))
        self.sock.settimeout(0.0)

    def get_imu_message(self) -> Imu | None:
        """Read one pending IMU message if a UDP datagram is available."""
        try:
            data, _addr = self.sock.recvfrom(4096)
        except BlockingIOError:
            return None

        packet = json.loads(data.decode("utf-8"))
        return imu_message_from_sample(
            self.stamp_factory(),
            self.frame_id,
            packet["gyro"],
            packet["accel"],
        )

    def close(self) -> None:
        """Close the underlying UDP socket."""
        self.sock.close()
