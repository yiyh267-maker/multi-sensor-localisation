import queue
import threading
import time

import serial
import pynmea2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from gnss_ros2.coord_utils import CoordConverter
from gnss_ros2.gnss_config import configure_gnss, PORT, TARGET_BAUD


FRAME_ID = "gnss_link"


class GnssPublisher(Node):
    def __init__(self):
        super().__init__("gnss_publisher")

        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.enu_pub = self.create_publisher(PointStamped, "/gnss/enu", 10)
        self.status_pub = self.create_publisher(String, "/gnss/status", 10)
        self.raw_pub = self.create_publisher(String, "/gnss/raw_nmea", 50)

        self.converter = None
        self.ref_alt = 0.0
        # Bound the serial backlog; new lines are dropped if the queue is full.
        self.line_queue = queue.Queue(maxsize=500)

        self.get_logger().info("Configuring GNSS to 115200 + 10Hz ...")
        configure_gnss()

        self.ser = serial.Serial(PORT, TARGET_BAUD, timeout=0.2)
        time.sleep(0.5)

        self.reader_running = True
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

        self.timer = self.create_timer(0.02, self.process_queue)
        self.get_logger().info(f"GNSS publisher started on {PORT} @ {TARGET_BAUD}")

    def reader_loop(self):
        while self.reader_running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                text = raw.decode("ascii", errors="ignore").strip()
                if not text.startswith("$"):
                    continue
                try:
                    self.line_queue.put_nowait(text)
                except queue.Full:
                    pass
            except Exception as e:
                self.get_logger().warn(f"GNSS read failed: {e}")
                time.sleep(0.1)

    def process_queue(self):
        processed = 0
        while not self.line_queue.empty() and processed < 50:
            line = self.line_queue.get()
            self.handle_line(line)
            processed += 1

    def publish_raw_and_status(self, line: str):
        raw_msg = String()
        raw_msg.data = line
        self.raw_pub.publish(raw_msg)

        status_msg = String()
        status_msg.data = line
        self.status_pub.publish(status_msg)

    def publish_fix(self, lat: float, lon: float, alt: float, has_fix: bool):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_ID
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        msg.status.service = NavSatStatus.SERVICE_GPS

        if has_fix:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = 0.0
            msg.longitude = 0.0
            msg.altitude = 0.0

        self.fix_pub.publish(msg)

        if has_fix:
            # Use the first valid fix as the local ENU origin.
            if self.converter is None:
                self.converter = CoordConverter(lat, lon)
                self.ref_alt = alt
                self.get_logger().info(
                    f"ENU origin set: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}"
                )

            x, y = self.converter.wgs84_to_enu(lat, lon)
            enu = PointStamped()
            enu.header = msg.header
            enu.header.frame_id = "map"
            enu.point.x = x
            enu.point.y = y
            enu.point.z = alt - self.ref_alt
            self.enu_pub.publish(enu)

    def handle_line(self, line: str):
        self.publish_raw_and_status(line)

        try:
            msg = pynmea2.parse(line)
        except Exception:
            return

        if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
            try:
                lat = float(msg.latitude or 0.0)
                lon = float(msg.longitude or 0.0)
                alt = float(msg.altitude) if msg.altitude not in [None, ""] else 0.0
                gps_qual = int(msg.gps_qual)
                has_fix = gps_qual > 0 and lat != 0.0 and lon != 0.0
                self.publish_fix(lat, lon, alt, has_fix)
            except Exception:
                self.publish_fix(0.0, 0.0, 0.0, False)
            return

        if line.startswith("$GNRMC") or line.startswith("$GPRMC"):
            try:
                lat = float(msg.latitude or 0.0)
                lon = float(msg.longitude or 0.0)
                has_fix = (msg.status == "A") and lat != 0.0 and lon != 0.0
                # RMC has no altitude; this branch publishes zero altitude.
                self.publish_fix(lat, lon, 0.0, has_fix)
            except Exception:
                self.publish_fix(0.0, 0.0, 0.0, False)

    def destroy_node(self):
        self.reader_running = False
        try:
            self.reader_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GnssPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
