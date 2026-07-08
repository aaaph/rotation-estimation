#!/usr/bin/python3
# src/sense_hat/sense_hat_udp_publisher.py  # noqa: EXE001

import argparse
import json
import pathlib
import socket
import time

import RTIMU  # pyright: ignore[reportMissingImports]  # ty: ignore[unresolved-import]


def default_settings_path() -> pathlib.Path:
    """Return the default RTIMU settings base path."""
    return pathlib.Path.home() / ".config" / "rotation_estimation" / "RTIMULib"


def parse_args() -> argparse.Namespace:
    """Parse the command line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--settings", default=default_settings_path())
    parser.add_argument("--rate-hz", type=float, default=0.0)
    return parser.parse_args()


def main() -> None:
    """Run the SenseHat UDP publisher."""
    args = parse_args()

    settings_path = pathlib.Path(args.settings).expanduser()
    settings_path.parent.mkdir(parents=True, exist_ok=True)

    settings = RTIMU.Settings(str(settings_path))
    imu = RTIMU.RTIMU(settings)

    if not imu.IMUInit():
        raise RuntimeError("RTIMU init failed")

    imu.setCompassEnable(False)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)

    poll_interval_s = imu.IMUGetPollInterval() / 1000.0
    min_period_s = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)

    seq = 0
    last_send = 0.0

    while True:
        if not imu.IMURead():
            time.sleep(poll_interval_s)
            continue

        now = time.time()
        monotonic_now = time.monotonic()

        if min_period_s > 0 and monotonic_now - last_send < min_period_s:
            time.sleep(poll_interval_s)
            continue

        data = imu.getIMUData()

        if not data.get("gyroValid", False) or not data.get("accelValid", False):
            time.sleep(poll_interval_s)
            continue

        gyro = data["gyro"]  # rad/s
        accel = data["accel"]  # G

        packet = {
            "seq": seq,
            "t": now,
            "gyro": {
                "x": float(gyro[0]),
                "y": float(gyro[1]),
                "z": float(gyro[2]),
            },
            "accel": {
                "x": float(accel[0]),
                "y": float(accel[1]),
                "z": float(accel[2]),
            },
        }

        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        sock.sendto(payload, target)

        seq += 1
        last_send = monotonic_now

        time.sleep(poll_interval_s)


if __name__ == "__main__":
    main()
