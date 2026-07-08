import json

import pytest
from builtin_interfaces.msg import Time

from sense_hat.imu_message import G
from sense_hat.imu_strategy_factory import (
    MOCK_MODE_STATIONARY,
    MOCK_MODE_YAW_ROTATION,
    STRATEGY_MODE_UDP_READER,
    ImuStrategyConfig,
    create_imu_strategy,
)
from sense_hat.stationary_imu_mock import StationaryImuMock
from sense_hat.udp_reader_imu_strategy import UdpReaderImuStrategy
from sense_hat.yaw_rotation_imu_mock import YawRotationImuMock


class FakeDatagramSocket:
    """Fake UDP socket for factory tests."""

    def __init__(self, packets: list[bytes]) -> None:
        """Initialize the fake socket."""
        self.packets = packets
        self.bound_address: tuple[str, int] | None = None
        self.timeout: float | None = None

    def bind(self, address: tuple[str, int]) -> None:
        """Record the socket bind address."""
        self.bound_address = address

    def settimeout(self, value: float) -> None:
        """Record the configured timeout."""
        self.timeout = value

    def recvfrom(self, bufsize: int) -> tuple[bytes, tuple[str, int]]:
        """Return the next fake datagram."""
        if not self.packets:
            raise BlockingIOError
        return self.packets.pop(0), ("127.0.0.1", 8765)

    def close(self) -> None:
        """Release fake socket resources."""


def _config(mode: str) -> ImuStrategyConfig:
    return ImuStrategyConfig(
        mode=mode,
        host="127.0.0.1",
        port=8765,
        frame_id="sensehat_link",
        stamp_factory=lambda: Time(sec=1, nanosec=2),
        angular_velocity_z=1.2,
    )


def test_create_imu_strategy_creates_stationary_mock():
    strategy = create_imu_strategy(_config(MOCK_MODE_STATIONARY))

    assert isinstance(strategy, StationaryImuMock)
    msg = strategy.get_imu_message()
    assert msg is not None
    assert msg.header.frame_id == "sensehat_link"
    assert msg.header.stamp.sec == 1
    assert msg.angular_velocity.z == pytest.approx(0.0)
    assert msg.linear_acceleration.z == pytest.approx(G)


def test_create_imu_strategy_creates_yaw_rotation_mock():
    strategy = create_imu_strategy(_config(MOCK_MODE_YAW_ROTATION))

    assert isinstance(strategy, YawRotationImuMock)
    msg = strategy.get_imu_message()
    assert msg is not None
    assert msg.header.frame_id == "sensehat_link"
    assert msg.angular_velocity.z == pytest.approx(1.2)
    assert msg.linear_acceleration.z == pytest.approx(G)


def test_create_imu_strategy_creates_udp_reader():
    packet = {
        "gyro": {"x": 0.0, "y": 0.0, "z": 0.3},
        "accel": {"x": 0.0, "y": 0.0, "z": 1.0},
    }
    sock = FakeDatagramSocket([json.dumps(packet).encode("utf-8")])

    strategy = create_imu_strategy(_config(STRATEGY_MODE_UDP_READER), sock=sock)

    assert isinstance(strategy, UdpReaderImuStrategy)
    assert sock.bound_address == ("127.0.0.1", 8765)
    assert sock.timeout == 0.0
    msg = strategy.get_imu_message()
    assert msg is not None
    assert msg.angular_velocity.z == pytest.approx(0.3)


def test_create_imu_strategy_rejects_unknown_mode():
    with pytest.raises(ValueError, match="Unsupported IMU strategy mode"):
        create_imu_strategy(_config("unknown"))
