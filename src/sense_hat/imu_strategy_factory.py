from collections.abc import Callable
from dataclasses import dataclass

from builtin_interfaces.msg import Time

from sense_hat.imu_strategy_protocol import ImuStrategy
from sense_hat.stationary_imu_mock import StationaryImuMock
from sense_hat.udp_reader_imu_strategy import DatagramSocket, UdpReaderImuStrategy
from sense_hat.yaw_rotation_imu_mock import YawRotationImuMock

STRATEGY_MODE_UDP_READER = "udp_reader"
MOCK_MODE_STATIONARY = "stationary"
MOCK_MODE_YAW_ROTATION = "yaw_rotation"
MOCK_MODES = {MOCK_MODE_STATIONARY, MOCK_MODE_YAW_ROTATION}
STRATEGY_MODES = {STRATEGY_MODE_UDP_READER, *MOCK_MODES}


@dataclass(frozen=True, kw_only=True)
class ImuStrategyConfig:
    """Configuration for creating an IMU strategy."""

    mode: str
    frame_id: str
    stamp_factory: Callable[[], Time]
    host: str = "127.0.0.1"
    port: int = 8765
    angular_velocity_z: float = 0.25


def create_imu_strategy(config: ImuStrategyConfig, sock: DatagramSocket | None = None) -> ImuStrategy:
    """Create an IMU strategy from configuration."""
    if config.mode == STRATEGY_MODE_UDP_READER:
        return UdpReaderImuStrategy(
            host=config.host,
            port=config.port,
            frame_id=config.frame_id,
            stamp_factory=config.stamp_factory,
            sock=sock,
        )

    if config.mode == MOCK_MODE_STATIONARY:
        return StationaryImuMock(
            frame_id=config.frame_id,
            stamp_factory=config.stamp_factory,
        )

    if config.mode == MOCK_MODE_YAW_ROTATION:
        return YawRotationImuMock(
            frame_id=config.frame_id,
            stamp_factory=config.stamp_factory,
            angular_velocity_z=config.angular_velocity_z,
        )

    msg = f"Unsupported IMU strategy mode '{config.mode}'. Use one of {sorted(STRATEGY_MODES)}."
    raise ValueError(msg)
