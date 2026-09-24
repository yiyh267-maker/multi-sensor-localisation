#!/usr/bin/env python3
import os

import math
import threading
import time
import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from flask import Flask, render_template
from flask_socketio import SocketIO

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from rtabmap_msgs.msg import Info
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped

# Convert WGS84 coordinates to GCJ-02 for AMap.
pi = 3.1415926535897932384626
a = 6378245.0
ee = 0.00669342162296594323


def wgs84_to_gcj02(lng, lat):
    if not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271):
        return lng, lat

    def _transformlat(lng, lat):
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(math.fabs(lng))
        ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0)) * 2.0 / 3.0
        return ret

    def _transformlng(lng, lat):
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(math.fabs(lng))
        ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * pi) + 40.0 * math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 * math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
        return ret

    dlat = _transformlat(lng - 105.0, lat - 35.0)
    dlng = _transformlng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
    return lng + dlng, lat + dlat


def quat_to_yaw_deg(x, y, z, w):
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


app = Flask(__name__, template_folder='../Display/templates', static_folder='../Display/static')
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

bridge_node = None


class DashboardBridge(Node):
    def __init__(self):
        super().__init__('dashboard_bridge')
        self.lock = threading.Lock()

        self.active_mode = 'SLAM'
        self.needs_recenter = True
        self.needs_path_reset = True
        self.rtabmap_ignore_count = 0

        self.origin_tx = 0.0
        self.origin_ty = 0.0
        self.origin_yaw = 0.0

        self.gnss_enu = {'x': 0.0, 'y': 0.0}
        self.gnss_last_xy = None
        self.gnss_yaw = 0.0
        self.gnss_seq = 0
        self.gnss_data = {
            'online': 'Not Fixed',
            'lon': 120.445,
            'lat': 36.389,
            'raw_lon': 120.445,
            'raw_lat': 36.389,
            'fixed': False,
            'stamp': 0.0,
            'server_time': 0.0,
            'seq': 0,
            'age': None,
            'status': -1,
            'service': 0,
        }

        self.last_scan_time = None
        self.last_imu_time = None
        self.vision_data = {'last_msg_time': 0, 'features': '等待...', 'matches': '等待...', 'last_loop_time': '--'}

        self.scan_times = deque(maxlen=50)
        self.imu_times = deque(maxlen=100)
        self.pose_odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.path_odom = []
        self.optimized_path = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 50)
        self.create_subscription(NavSatFix, '/fix', self.gnss_callback, 10)
        self.create_subscription(PointStamped, '/gnss/enu', self.enu_callback, 10)
        self.create_subscription(Info, '/info', self.vision_callback, 10)
        self.create_subscription(Path, '/mapPath', self.path_callback, 10)

    def calc_freq(self, times_deque):
        if len(times_deque) < 2:
            return 0.0
        dt = times_deque[-1] - times_deque[0]
        if dt <= 0:
            return 0.0
        return round((len(times_deque) - 1) / dt, 1)

    def scan_callback(self, msg):
        with self.lock:
            now = time.time()
            self.last_scan_time = now
            self.scan_times.append(now)

    def imu_callback(self, msg):
        with self.lock:
            now = time.time()
            self.last_imu_time = now
            self.imu_times.append(now)

    def gnss_callback(self, msg):
        now = time.time()
        raw_lon = float(msg.longitude)
        raw_lat = float(msg.latitude)
        fixed = (msg.status.status != -1) and (abs(raw_lon) > 1e-7) and (abs(raw_lat) > 1e-7)

        with self.lock:
            self.gnss_seq += 1
            seq = self.gnss_seq

            if fixed:
                map_lon, map_lat = wgs84_to_gcj02(raw_lon, raw_lat)
                self.gnss_data = {
                    'online': 'Online',
                    'fixed': True,
                    # Keep the original WGS84 fix alongside the map coordinates.
                    'raw_lon': raw_lon,
                    'raw_lat': raw_lat,
                    # GCJ-02 coordinates for AMap.
                    'lon': float(map_lon),
                    'lat': float(map_lat),
                    'stamp': now,
                    'server_time': now,
                    'seq': seq,
                    'age': 0.0,
                    'status': int(msg.status.status),
                    'service': int(msg.status.service),
                    'ros_stamp_sec': int(msg.header.stamp.sec),
                    'ros_stamp_nanosec': int(msg.header.stamp.nanosec),
                }
                print(
                    f'[BRIDGE /fix] seq={seq} status={msg.status.status} '
                    f'raw=({raw_lon:.9f},{raw_lat:.9f}) '
                    f'map=({map_lon:.9f},{map_lat:.9f})',
                    flush=True,
                )
            else:
                self.gnss_data = {
                    'online': 'Not Fixed',
                    'fixed': False,
                    'raw_lon': raw_lon,
                    'raw_lat': raw_lat,
                    'lon': None,
                    'lat': None,
                    'stamp': now,
                    'server_time': now,
                    'seq': seq,
                    'age': 0.0,
                    'status': int(msg.status.status),
                    'service': int(msg.status.service),
                    'ros_stamp_sec': int(msg.header.stamp.sec),
                    'ros_stamp_nanosec': int(msg.header.stamp.nanosec),
                }
                print(
                    f'[BRIDGE /fix] seq={seq} status={msg.status.status} '
                    f'NOT_FIXED raw=({raw_lon:.9f},{raw_lat:.9f})',
                    flush=True,
                )

    def enu_callback(self, msg):
        with self.lock:
            new_x, new_y = float(msg.point.x), float(msg.point.y)
            if self.active_mode == 'GNSS' and self.gnss_last_xy:
                dx = new_x - self.gnss_last_xy[0]
                dy = new_y - self.gnss_last_xy[1]
                if math.hypot(dx, dy) > 0.15:
                    self.gnss_yaw = math.degrees(math.atan2(dy, dx))

            self.gnss_enu['x'] = new_x
            self.gnss_enu['y'] = new_y
            self.gnss_last_xy = (new_x, new_y)

    def vision_callback(self, msg):
        with self.lock:
            self.vision_data['last_msg_time'] = time.time()
            if msg.loop_closure_id > 0:
                self.vision_data['last_loop_time'] = datetime.datetime.now().strftime('%H:%M:%S')
                self.vision_data['features'], self.vision_data['matches'] = '400+ 完成', '16+ 回环'
            else:
                self.vision_data['features'], self.vision_data['matches'] = '提取中...', '搜索中...'

    def path_callback(self, msg):
        with self.lock:
            if self.active_mode != 'SLAM':
                return
            if self.needs_recenter:
                return

            if self.needs_path_reset:
                self.rtabmap_ignore_count = len(msg.poses)
                self.needs_path_reset = False

            if len(msg.poses) < self.rtabmap_ignore_count:
                self.rtabmap_ignore_count = 0

            # Discard path entries that predate the mode reset.
            valid_poses = msg.poses[self.rtabmap_ignore_count:]
            transformed_path = []
            rad = math.radians(-self.origin_yaw)
            last_added_x, last_added_y = None, None

            for p in valid_poses:
                px = float(p.pose.position.x)
                py = float(p.pose.position.y)
                if last_added_x is not None and last_added_y is not None:
                    if math.hypot(px - last_added_x, py - last_added_y) < 0.05:
                        continue

                last_added_x, last_added_y = px, py
                dx = px - self.origin_tx
                dy = py - self.origin_ty
                nx = dx * math.cos(rad) - dy * math.sin(rad)
                ny = dx * math.sin(rad) + dy * math.cos(rad)
                transformed_path.append({'x': nx, 'y': ny})

            self.optimized_path = transformed_path


@socketio.on('switch_mode')
def handle_switch_mode(data):
    target_mode = data.get('mode', 'SLAM')
    if target_mode not in ('SLAM', 'GNSS'):
        target_mode = 'SLAM'

    global bridge_node
    if bridge_node:
        with bridge_node.lock:
            bridge_node.active_mode = target_mode
            bridge_node.needs_recenter = True
            bridge_node.needs_path_reset = True
            bridge_node.path_odom = []
            bridge_node.optimized_path = []
            bridge_node.pose_odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            bridge_node.gnss_last_xy = None
    print(f'🔄 模式已切换为: {target_mode}，后端已按模式隔离 GNSS 与 SLAM 显示数据。', flush=True)


def build_status(node, mode_str, now):
    return {
        'mode': mode_str,
        'active_mode': node.active_mode,
        'lidar': {
            'online': '在线' if (node.last_scan_time and now - node.last_scan_time < 2) else '离线',
            'frequency': node.calc_freq(node.scan_times),
        },
        'imu': {
            'online': '在线' if (node.last_imu_time and now - node.last_imu_time < 2) else '离线',
            'frequency': node.calc_freq(node.imu_times),
        },
        'vision': node.vision_data,
    }


def background_broadcaster(node):
    while True:
        time.sleep(0.1)
        try:
            mode_str = '定位中...'
            use_map = False
            raw_tx, raw_ty, raw_yaw = None, None, None

            active_mode = node.active_mode

            if active_mode == 'SLAM':
                try:
                    # Prefer the map transform; fall back to local odometry below.
                    t = node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    mode_str, use_map = '室内 SLAM', True
                    raw_tx = float(t.transform.translation.x)
                    raw_ty = float(t.transform.translation.y)
                    raw_yaw = quat_to_yaw_deg(
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w,
                    )
                except Exception:
                    try:
                        t = node.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                        mode_str = '室内里程计'
                        raw_tx = float(t.transform.translation.x)
                        raw_ty = float(t.transform.translation.y)
                        raw_yaw = quat_to_yaw_deg(
                            t.transform.rotation.x,
                            t.transform.rotation.y,
                            t.transform.rotation.z,
                            t.transform.rotation.w,
                        )
                    except Exception:
                        pass
            else:
                with node.lock:
                    mode_str = '室外 GNSS'
                    raw_tx = float(node.gnss_enu['x'])
                    raw_ty = float(node.gnss_enu['y'])
                    raw_yaw = float(node.gnss_yaw)

            with node.lock:
                active_mode = node.active_mode

                if active_mode == 'SLAM' and raw_tx is not None and raw_yaw is not None:
                    if node.needs_recenter:
                        node.origin_tx = raw_tx
                        node.origin_ty = raw_ty
                        node.origin_yaw = raw_yaw
                        node.needs_recenter = False

                    # Express the display pose relative to the first pose after a reset.
                    dx = raw_tx - node.origin_tx
                    dy = raw_ty - node.origin_ty
                    rad = math.radians(-node.origin_yaw)
                    tx = dx * math.cos(rad) - dy * math.sin(rad)
                    ty = dx * math.sin(rad) + dy * math.cos(rad)

                    yaw = raw_yaw - node.origin_yaw
                    while yaw > 180:
                        yaw -= 360
                    while yaw < -180:
                        yaw += 360

                    node.pose_odom = {'x': tx, 'y': ty, 'yaw': yaw}

                    if use_map and node.optimized_path:
                        node.path_odom = list(node.optimized_path)
                    else:
                        if not node.path_odom or math.hypot(tx - node.path_odom[-1]['x'], ty - node.path_odom[-1]['y']) > 0.05:
                            node.path_odom.append({'x': tx, 'y': ty})
                            if len(node.path_odom) > 1000:
                                node.path_odom.pop(0)

                    out_pose = dict(node.pose_odom)
                    out_path = list(node.path_odom)
                else:
                    # Clear the canvas path in GNSS mode or when no SLAM pose is available.
                    node.pose_odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
                    node.path_odom = []
                    node.optimized_path = []
                    out_pose = None
                    out_path = []

                now = time.time()
                gnss_out = dict(node.gnss_data)
                if gnss_out.get('stamp'):
                    gnss_out['age'] = round(now - float(gnss_out['stamp']), 3)
                else:
                    gnss_out['age'] = None
                gnss_out['server_time'] = now

                data = {
                    'pose': out_pose,
                    'path': out_path,
                    'gnss': gnss_out,
                    'status': build_status(node, mode_str, now),
                }

            socketio.emit('map_update', data)

        except Exception as e:
            print(f'[bridge_server] broadcaster error: {e}', flush=True)


@app.route('/')
def index():
    amap_api_key = os.environ.get('AMAP_API_KEY', '').strip()
    if not amap_api_key:
        return (
            'AMAP_API_KEY is not configured. Export it in the server environment '
            'before starting the dashboard; see RELEASE_SETUP.md.',
            503,
        )
    return render_template('index.html', amap_api_key=amap_api_key)


def ros_thread(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    rclpy.init()
    bridge_node = DashboardBridge()
    threading.Thread(target=ros_thread, args=(bridge_node,), daemon=True).start()
    threading.Thread(target=background_broadcaster, args=(bridge_node,), daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5001, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
