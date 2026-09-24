import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from ybimu_ros2.YbImuLib.YbImuSerialLib import YbImuSerial


SERIAL_PORT = "/dev/imu"
FRAME_ID = "imu_link"
PUBLISH_TOPIC = "/imu/data_raw"
TIMER_PERIOD = 0.02  # 50 Hz timer; independent of the sensor report rate.


class YbImuPublisher(Node):
    def __init__(self):
        super().__init__("ybimu_publisher")

        self.publisher_ = self.create_publisher(Imu, PUBLISH_TOPIC, 10)

        self.get_logger().info(f"Opening IMU on {SERIAL_PORT}")
        self.imu = YbImuSerial(SERIAL_PORT)
        self.imu.create_receive_threading()

        # Allow the serial receive thread to start.
        time.sleep(0.1)

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.get_logger().info("YB IMU publisher started")

    def timer_callback(self):
        try:
            acc = self.imu.get_accelerometer_data()      # [ax, ay, az] in g
            gyro = self.imu.get_gyroscope_data()         # [gx, gy, gz] in rad/s
            quat = self.imu.get_imu_quaternion_data()    # [w, x, y, z]

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = FRAME_ID

            # Convert acceleration from g to m/s^2.
            msg.linear_acceleration.x = float(acc[0]) * 9.80665
            msg.linear_acceleration.y = float(acc[1]) * 9.80665
            msg.linear_acceleration.z = float(acc[2]) * 9.80665

            msg.angular_velocity.x = float(gyro[0])
            msg.angular_velocity.y = float(gyro[1])
            msg.angular_velocity.z = float(gyro[2])

            msg.orientation.w = float(quat[0])
            msg.orientation.x = float(quat[1])
            msg.orientation.y = float(quat[2])
            msg.orientation.z = float(quat[3])

            # Fixed covariance estimates; these are not measured sensor uncertainties.
            msg.orientation_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]
            msg.linear_acceleration_covariance = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 0.5
            ]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"IMU read failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YbImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
