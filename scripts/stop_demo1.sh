#!/bin/bash

echo "🔴 [STOP] 开始关闭全系统进程..."
sudo pkill -f gamepad_teleop_diff.py

echo "🧠 正在关闭 SLAM 与融合里程计..."
pkill -f rtabmap
pkill -f rf2o_laser_odometry
pkill -f ekf_node

echo "📡 正在关闭传感器驱动 (Camera, LiDAR, IMU, GNSS)..."
pkill -f usb_cam_node_exe
pkill -f rplidar_node
pkill -f imu_publisher
pkill -f gnss_publisher

echo "🌐 正在关闭 Web 桥接与前端界面 (WebSocket)..."
pkill -f bridge_server.py

echo "🧰 正在关闭 TF 坐标发布与 QoS 桥接..."
pkill -f static_transform_publisher
pkill -f relay

echo "🧹 强制清理残留进程与共享内存..."
sleep 1
sudo killall -9 rtabmap rplidar_node usb_cam_node_exe imu_publisher rf2o_laser_odometry_node ekf_node 2>/dev/null || true
sudo rm -rf /dev/shm/fastrtps* 2>/dev/null || true

# Release the dashboard port.
sudo kill -9 $(sudo lsof -t -i:5001) 2>/dev/null

# Remove the saved RTAB-Map database and project logs.
echo "💾 正在清空 SD 卡中的 SLAM 历史数据库释放空间..."
rm -f ~/.ros/rtabmap.db
sudo rm -f ~/rtls_project/logs/*.log

echo "=================================================="
echo "✅ Demo 1 系统已安全下线！所有硬件、网络服务及 SD 卡缓存已彻底清空。"
echo "=================================================="
