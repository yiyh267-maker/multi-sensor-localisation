#!/usr/bin/env python3

import time
import json
from datetime import datetime
from IMU.YbImuLib import YbImuSerial

SERIAL_PORT = "/dev/ttyUSB1"
REPORT_HZ = 10


def build_imu_packet(imu):
    accel = imu.get_accelerometer_data()
    gyro = imu.get_gyroscope_data()
    mag = imu.get_magnetometer_data()
    euler = imu.get_imu_attitude_data()
    quat = imu.get_imu_quaternion_data()
    baro = imu.get_baro_data()

    packet = {
        "timestamp": datetime.now().isoformat(),
        "accel_g": {
            "x": accel[0],
            "y": accel[1],
            "z": accel[2]
        },
        "gyro_rad_s": {
            "x": gyro[0],
            "y": gyro[1],
            "z": gyro[2]
        },
        "mag_uT": {
            "x": mag[0],
            "y": mag[1],
            "z": mag[2]
        },
        "euler_deg": {
            "roll": euler[0],
            "pitch": euler[1],
            "yaw": euler[2]
        },
        "quaternion": {
            "w": quat[0],
            "x": quat[1],
            "y": quat[2],
            "z": quat[3]
        },
        "barometer": {
            "height_m": baro[0],
            "temperature_c": baro[1],
            "pressure": baro[2],
            "pressure_contrast": baro[3]
        }
    }
    return packet


def print_imu_packet(packet):
    print("=" * 70)
    print("Timestamp:", packet["timestamp"])
    print("ACC  (g):      ", packet["accel_g"])
    print("GYRO (rad/s):  ", packet["gyro_rad_s"])
    print("MAG  (uT):     ", packet["mag_uT"])
    print("EULER (deg):   ", packet["euler_deg"])
    print("QUAT:          ", packet["quaternion"])
    print("BARO:          ", packet["barometer"])


def main():
    imu = YbImuSerial(SERIAL_PORT, debug=False)
    imu.create_receive_threading()

    time.sleep(1.0)

    version = imu.get_version()
    print("Firmware version:", version)

    try:
        while True:
            packet = build_imu_packet(imu)
            print_imu_packet(packet)


            time.sleep(1.0 / REPORT_HZ)

    except KeyboardInterrupt:
        print("\nIMU reader stopped.")


if __name__ == "__main__":
    main()
