#!/usr/bin/env python3

import sys
import time
from pathlib import Path
from typing import Optional

sys.path.append(str(Path(__file__).resolve().parents[2]))

from IMU.YbImuLib import YbImuSerial


SERIAL_PORT = "/dev/ttyUSB1"
POST_CALIBRATION_WAIT = 2.0


def create_serial_device() -> YbImuSerial:
    imu = YbImuSerial(SERIAL_PORT, debug=False)
    imu.create_receive_threading()
    time.sleep(0.1)
    return imu


def format_state(state: Optional[int]) -> str:
    if state == 1:
        return "success"
    if state == 0 or state is None:
        return "incomplete"
    return f"code {state}"


def main() -> None:
    imu = create_serial_device()

    try:
        version = imu.get_version()
        print(f"Firmware version: {version}")
    except Exception:
        print("Firmware version: unavailable.")

    print("Starting magnetometer calibration (rotate the device through all axes)...")
    state = imu.calibration_mag()
    print(f"Magnetometer calibration result: {format_state(state)}")

    time.sleep(POST_CALIBRATION_WAIT)
    print("Calibration script finished.")


if __name__ == "__main__":
    main()
