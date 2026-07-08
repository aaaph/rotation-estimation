from sense_hat.sense_hat_udp_publisher import build_packet


def test_build_packet_uses_sense_hat_raw_dict_values() -> None:
    packet = build_packet(
        seq=7,
        now=123.5,
        gyro={"x": "0.1", "y": -0.2, "z": 0},
        accel={"x": 0.0, "y": "-0.062", "z": 1.0},
    )

    assert packet == {
        "seq": 7,
        "t": 123.5,
        "gyro": {
            "x": 0.1,
            "y": -0.2,
            "z": 0.0,
        },
        "accel": {
            "x": 0.0,
            "y": -0.062,
            "z": 1.0,
        },
    }
