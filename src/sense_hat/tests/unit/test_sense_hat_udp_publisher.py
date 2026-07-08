import pathlib

from sense_hat.sense_hat_udp_publisher import build_packet, clean_system_import_paths


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


def test_clean_system_import_paths_removes_pixi_python_and_repo_src_paths() -> None:
    repo_src = pathlib.Path("/repo/src")

    assert clean_system_import_paths(
        [
            "",
            "/repo/src",
            "/repo/.pixi/envs/pi-runtime/lib/python3.12/site-packages",
            "/usr/lib/python3/dist-packages",
        ],
        repo_src,
    ) == [
        "",
        "/usr/lib/python3/dist-packages",
    ]
