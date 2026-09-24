#!/usr/bin/env python3
import os
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, FluidPressure

PROJECT_ROOT = os.path.expanduser('~/rtls_project')
IMU_ROOT = os.path.join(PROJECT_ROOT, 'IMU')
if IMU_ROOT not in sys.path:
    sys.path.append(IMU_ROOT)

from YbImuLib.YbImuSerialLib import YbImuSerial  # noqa: E402


class YbImuPublisher(Node):
    def __init__(self) -> None:
        super().__init__('ybimu_publisher')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('report_rate_hz', 50)
        self.declare_parameter('algo_type', 9)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.report_rate_hz = self.get_parameter('report_rate_hz').get_parameter_value().integer_value
        self.algo_type = self.get_parameter('algo_type').get_parameter_value().integer_value

        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 20)
        self.pressure_pub = self.create_publisher(FluidPressure, '/fluid_pressure', 20)

        self.imu_device: Optional[YbImuSerial] = None
        self._connect_imu()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.publish_once)

    def _connect_imu(self) -> None:
        try:
            self.imu_device = YbImuSerial(self.port, debug=False)
            self.imu_device.create_receive_threading()
            self.imu_device.set_report_rate(int(self.report_rate_hz))
            self.imu_device.set_algo_type(int(self.algo_type))
            self.get_logger().info(f'YBIMU connected on {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open YBIMU on {self.port}: {e}')
            raise

    def publish_once(self) -> None:
        if self.imu_device is None:
            return

        try:
            accel = self.imu_device.get_accelerometer_data()
            gyro = self.imu_device.get_gyroscope_data()
            quat = self.imu_device.get_imu_quaternion_data()
            baro = self.imu_device.get_baro_data()

            now = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.frame_id

            g_to_ms2 = 9.80665
            imu_msg.linear_acceleration.x = float(accel[0]) * g_to_ms2
            imu_msg.linear_acceleration.y = float(accel[1]) * g_to_ms2
            imu_msg.linear_acceleration.z = float(accel[2]) * g_to_ms2

            imu_msg.angular_velocity.x = float(gyro[0])
            imu_msg.angular_velocity.y = float(gyro[1])
            imu_msg.angular_velocity.z = float(gyro[2])

            imu_msg.orientation.w = float(quat[0])
            imu_msg.orientation.x = float(quat[1])
            imu_msg.orientation.y = float(quat[2])
            imu_msg.orientation.z = float(quat[3])

            imu_msg.orientation_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.05
            ]
            imu_msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.04
            ]
            imu_msg.linear_acceleration_covariance = [
                0.2, 0.0, 0.0,
                0.0, 0.2, 0.0,
                0.0, 0.0, 0.3
            ]

            self.imu_pub.publish(imu_msg)

            pressure_msg = FluidPressure()
            pressure_msg.header.stamp = now
            pressure_msg.header.frame_id = self.frame_id
            pressure_kpa = float(baro[2])
            pressure_msg.fluid_pressure = pressure_kpa * 1000.0
            pressure_msg.variance = 0.0
            self.pressure_pub.publish(pressure_msg)

        except Exception as e:
            self.get_logger().warn(f'Publish failed: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YbImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
