#!/usr/bin/python3
# scripts/run_sense_hat_udp.py  # noqa: EXE001

import argparse
import importlib
import json
import os
import pathlib
import socket
import sys
import time
from collections.abc import Iterable, Mapping
from typing import Any, Protocol, Union, cast

# Keep compatibility with the system Python shipped on older target images.
NumberLike = Union[float, int, str]  # noqa: UP007


class SenseHatDevice(Protocol):
    """Minimal Sense HAT API used by the UDP publisher."""

    def get_gyroscope_raw(self) -> Mapping[str, NumberLike]:
        """Read raw gyroscope data."""

    def get_accelerometer_raw(self) -> Mapping[str, NumberLike]:
        """Read raw accelerometer data."""


def parse_args() -> argparse.Namespace:
    """Parse the command line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--settings", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--rate-hz", type=float, default=0.0)
    return parser.parse_args()


def should_drop_system_import_path(path: str, repo_src: pathlib.Path) -> bool:
    """Return whether a path should be hidden from system Python imports."""
    if path == "":
        return False

    resolved = pathlib.Path(path).expanduser().resolve()
    if resolved == repo_src:
        return True

    return ".pixi" in resolved.parts and ("site-packages" in resolved.parts or "dist-packages" in resolved.parts)


def clean_system_import_paths(paths: Iterable[str], repo_src: pathlib.Path) -> list[str]:
    """Remove Pixi Python and local repo package paths from system Python imports."""
    return [path for path in paths if not should_drop_system_import_path(path, repo_src)]


def prepare_system_sense_hat_import() -> None:
    """Prevent system Python from importing Pixi Python packages."""
    repo_src = pathlib.Path(__file__).resolve().parents[1]
    sys.path[:] = clean_system_import_paths(sys.path, repo_src)

    python_path = os.environ.get("PYTHONPATH")
    if python_path is not None:
        os.environ["PYTHONPATH"] = os.pathsep.join(
            clean_system_import_paths(python_path.split(os.pathsep), repo_src)
        )


def create_sense_hat() -> SenseHatDevice:
    """Create a Sense HAT device."""
    prepare_system_sense_hat_import()

    try:
        sense_hat_module = importlib.import_module("sense_hat")
    except ModuleNotFoundError as exc:
        msg = "python3-sense-hat is not installed in this interpreter"
        raise RuntimeError(msg) from exc

    sense_hat_class = getattr(sense_hat_module, "SenseHat", None)
    if sense_hat_class is None:
        msg = "imported sense_hat package does not expose SenseHat"
        raise RuntimeError(msg)

    sense: Any = sense_hat_class()
    sense.set_imu_config(compass_enabled=False, gyro_enabled=True, accel_enabled=True)
    return cast("SenseHatDevice", sense)


def sleep_until_next_sample(started_at: float, min_period_s: float) -> None:
    """Sleep until the configured sample period elapses."""
    if min_period_s <= 0.0:
        return

    elapsed_s = time.monotonic() - started_at
    remaining_s = min_period_s - elapsed_s
    if remaining_s > 0.0:
        time.sleep(remaining_s)


def build_packet(
    seq: int,
    now: float,
    gyro: Mapping[str, NumberLike],
    accel: Mapping[str, NumberLike],
) -> dict[str, object]:
    """Build the UDP IMU packet."""
    return {
        "seq": seq,
        "t": now,
        "gyro": {
            "x": float(gyro["x"]),
            "y": float(gyro["y"]),
            "z": float(gyro["z"]),
        },
        "accel": {
            "x": float(accel["x"]),
            "y": float(accel["y"]),
            "z": float(accel["z"]),
        },
    }


def main() -> None:
    """Run the SenseHat UDP publisher."""
    args = parse_args()

    min_period_s = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0
    sense = create_sense_hat()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, args.port)

    seq = 0

    while True:
        started_at = time.monotonic()
        gyro = sense.get_gyroscope_raw()  # rad/s
        accel = sense.get_accelerometer_raw()  # G
        now = time.time()

        packet = build_packet(seq, now, gyro, accel)

        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        sock.sendto(payload, target)

        seq += 1

        sleep_until_next_sample(started_at, min_period_s)


if __name__ == "__main__":
    main()
