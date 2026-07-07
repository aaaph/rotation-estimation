#!/usr/bin/python3
# src/sensehat_udp_sidecar/sensehat_udp_reader.py

import argparse
import json
import socket
import time

import RTIMU  # pyright: ignore[reportMissingImports]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--settings", default="/tmp/rotation_estimation/RTIMULib")
    parser.add_argument("--rate-hz", type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()

    settings = RTIMU.Settings(args.settings)
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

    print(f"Sending Sense HAT IMU UDP to {args.host}:{args.port}")
    print(f"RTIMU poll interval: {poll_interval_s:.6f}s")


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

        gyro = data["gyro"]    # rad/s
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