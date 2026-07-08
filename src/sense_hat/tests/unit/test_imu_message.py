import pytest
from builtin_interfaces.msg import Time

from sense_hat.imu_message import G, imu_message_from_sample


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
