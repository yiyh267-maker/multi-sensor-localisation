#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class YuyvToBgrNode(Node):
    def __init__(self):
        super().__init__('yuyv_to_bgr_node')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            Image,
            '/image_raw_bgr',
            10
        )

        self.get_logger().info('YUYV -> BGR converter started.')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert packed YUYV directly; use cv_bridge for other encodings.
            if msg.encoding == 'yuv422_yuy2':
                bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            else:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            out_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Conversion failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YuyvToBgrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
