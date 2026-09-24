#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


class OdomPrettyPrint(Node):
    def __init__(self) -> None:
        super().__init__('odom_pretty_print')

        self.declare_parameter('topic_name', '/odometry/filtered')
        self.declare_parameter('print_rate_hz', 5.0)

        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.print_rate_hz = self.get_parameter('print_rate_hz').get_parameter_value().double_value

        self.latest_msg: Optional[Odometry] = None

        self.create_subscription(
            Odometry,
            self.topic_name,
            self.odom_callback,
            10
        )

        period = 1.0 / max(self.print_rate_hz, 0.1)
        self.create_timer(period, self.print_latest)

        self.get_logger().info(
            f'Subscribing to {self.topic_name}, print_rate_hz={self.print_rate_hz}'
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_msg = msg

    def print_latest(self) -> None:
        if self.latest_msg is None:
            print('尚未收到里程计数据...\n')
            return

        msg = self.latest_msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_deg = quaternion_to_yaw_deg(qx, qy, qz, qw)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        print(
            f'x方向位移: {x:.4f} m\n'
            f'y方向位移: {y:.4f} m\n'
            f'航向角: {yaw_deg:.2f} °\n'
            f'x方向速度: {vx:.4f} m/s\n'
            f'y方向速度: {vy:.4f} m/s\n'
            f'z角速度: {wz:.4f} rad/s\n'
            f'------------------------------'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomPrettyPrint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

