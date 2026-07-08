import pytest
from builtin_interfaces.msg import Time

from sense_hat.sense_hat_imu_node import (
    MOCK_MODE_STATIONARY,
    MOCK_MODE_YAW_ROTATION,
    G,
    imu_message_from_sample,
    mock_imu_message,
)


def test_imu_message_from_sample_converts_accel_g_to_meters_per_second_squared():
    stamp = Time(sec=123, nanosec=456)

    msg = imu_message_from_sample(
        stamp=stamp,
        frame_id="sensehat_link",
        gyro={"x": "0.1", "y": -0.2, "z": 0},
        accel_g={"x": 0.0, "y": "-1.0", "z": 1.0},
    )

    assert msg.header.stamp.sec == 123
    assert msg.header.stamp.nanosec == 456
    assert msg.header.frame_id == "sensehat_link"
    assert msg.orientation_covariance[0] == -1.0

    assert msg.angular_velocity.x == pytest.approx(0.1)
    assert msg.angular_velocity.y == pytest.approx(-0.2)
    assert msg.angular_velocity.z == pytest.approx(0.0)

    assert msg.linear_acceleration.x == pytest.approx(0.0)
    assert msg.linear_acceleration.y == pytest.approx(-G)
    assert msg.linear_acceleration.z == pytest.approx(G)


def test_stationary_mock_imu_message_represents_level_sensor_without_rotation():
    msg = mock_imu_message(
        stamp=Time(sec=1, nanosec=2),
        frame_id="mock_link",
        mode=MOCK_MODE_STATIONARY,
        angular_velocity_z=0.5,
    )

    assert msg.header.frame_id == "mock_link"
    assert msg.angular_velocity.x == pytest.approx(0.0)
    assert msg.angular_velocity.y == pytest.approx(0.0)
    assert msg.angular_velocity.z == pytest.approx(0.0)
    assert msg.linear_acceleration.x == pytest.approx(0.0)
    assert msg.linear_acceleration.y == pytest.approx(0.0)
    assert msg.linear_acceleration.z == pytest.approx(G)


def test_yaw_rotation_mock_imu_message_uses_configured_z_rotation():
    msg = mock_imu_message(
        stamp=Time(sec=1, nanosec=2),
        frame_id="mock_link",
        mode=MOCK_MODE_YAW_ROTATION,
        angular_velocity_z=0.5,
    )

    assert msg.angular_velocity.z == pytest.approx(0.5)
    assert msg.linear_acceleration.x == pytest.approx(0.0)
    assert msg.linear_acceleration.y == pytest.approx(0.0)
    assert msg.linear_acceleration.z == pytest.approx(G)


def test_mock_imu_message_rejects_unknown_mode():
    with pytest.raises(ValueError, match="Unsupported mock_mode"):
        mock_imu_message(
            stamp=Time(),
            frame_id="mock_link",
            mode="unknown",
            angular_velocity_z=0.5,
        )
