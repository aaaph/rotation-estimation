import json

import pytest
from builtin_interfaces.msg import Time

from sense_hat.imu_message import G
from sense_hat.udp_reader_imu_strategy import UdpReaderImuStrategy


class FakeDatagramSocket:
    """Fake UDP socket for UdpReaderImuStrategy tests."""

    def __init__(self, packets: list[bytes]) -> None:
        """Initialize the fake socket."""
        self.packets = packets
        self.bound_address: tuple[str, int] | None = None
        self.timeout: float | None = None
        self.last_bufsize: int | None = None
        self.closed = False

    def bind(self, address: tuple[str, int]) -> None:
        """Record the socket bind address."""
        self.bound_address = address

    def settimeout(self, value: float) -> None:
        """Record the configured timeout."""
        self.timeout = value

    def recvfrom(self, bufsize: int) -> tuple[bytes, tuple[str, int]]:
        """Return the next fake datagram."""
        self.last_bufsize = bufsize
        if not self.packets:
            raise BlockingIOError
        return self.packets.pop(0), ("127.0.0.1", 8765)

    def close(self) -> None:
        """Record that the socket was closed."""
        self.closed = True


def test_udp_reader_imu_strategy_converts_udp_packet_to_imu_message():
    packet = {
        "gyro": {"x": "0.1", "y": -0.2, "z": 0.3},
        "accel": {"x": 0.0, "y": "-1.0", "z": 1.0},
    }
    sock = FakeDatagramSocket([json.dumps(packet).encode("utf-8")])
    reader = UdpReaderImuStrategy(
        host="127.0.0.1",
        port=8765,
        frame_id="sensehat_link",
        stamp_factory=lambda: Time(sec=10, nanosec=20),
        sock=sock,
    )

    msg = reader.get_imu_message()

    assert msg is not None
    assert sock.bound_address == ("127.0.0.1", 8765)
    assert sock.timeout == 0.0
    assert sock.last_bufsize == 4096
    assert msg.header.stamp.sec == 10
    assert msg.header.stamp.nanosec == 20
    assert msg.header.frame_id == "sensehat_link"
    assert msg.angular_velocity.x == pytest.approx(0.1)
    assert msg.angular_velocity.y == pytest.approx(-0.2)
    assert msg.angular_velocity.z == pytest.approx(0.3)
    assert msg.linear_acceleration.x == pytest.approx(0.0)
    assert msg.linear_acceleration.y == pytest.approx(-G)
    assert msg.linear_acceleration.z == pytest.approx(G)

    reader.close()
    assert sock.closed is True


def test_udp_reader_imu_strategy_returns_none_when_no_packet_is_available():
    sock = FakeDatagramSocket([])
    reader = UdpReaderImuStrategy(
        host="127.0.0.1",
        port=8765,
        frame_id="sensehat_link",
        stamp_factory=lambda: Time(sec=10, nanosec=20),
        sock=sock,
    )

    assert reader.get_imu_message() is None
