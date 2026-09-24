#!/bin/bash

# ==========================================
# Start the alternate ICP odometry pipeline on ROS2 Jazzy.
# Launch sensor drivers, static transforms, QoS relays and EKF.
# ==========================================

echo "🚀 [START] 开始启动底层系统..."

# Use the preconfigured device links.
LIDAR_PORT="/dev/lidar"
IMU_PORT="/dev/imu"
GNSS_PORT="/dev/gnss"

echo "🧹 [1/7] 清理残留进程，释放串口资源..."
# Stop previous nodes before reopening the serial ports.
sudo killall -9 rplidar_node imu_publisher relay icp_odometry ekf_node static_transform_publisher 2>/dev/null
sleep 2

echo "🔓 [2/7] 赋予硬件端口读写权限..."
sudo chmod 777 $LIDAR_PORT $IMU_PORT $GNSS_PORT

echo "📡 [3/7] 启动传感器驱动层..."
source /opt/ros/jazzy/setup.bash

# Start RPLIDAR A1 in the background.
nohup ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:=$LIDAR_PORT \
    serial_baudrate:=115200 > /tmp/lidar.log 2>&1 &

# Use the IMU publisher from the LIO2D workspace.
source ~/rtls_project/LIO2D/ros2_ws/install/setup.bash
nohup ros2 run ybimu_ros2 imu_publisher > /tmp/imu.log 2>&1 &
sleep 4 # Allow sensor initialization.

echo "📐 [4/7] 发布静态 TF 坐标变换 (base_link -> laser/imu)..."
source /opt/ros/jazzy/setup.bash
# Laser mounting transform: zero translation and 180-degree yaw.
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 3.14159 0 0 base_link laser > /tmp/tf_laser.log 2>&1 &
# IMU mounting offset in metres.
nohup ros2 run tf2_ros static_transform_publisher 0 -0.042 -0.045 0 0 0 base_link imu_link > /tmp/tf_imu.log 2>&1 &

echo "🌉 [5/7] 启动 QoS 桥接 (将传感器数据洗成 Reliable)..."
nohup ros2 run topic_tools relay /scan /scan_reliable --ros-args -p reliability:=reliable > /tmp/relay_scan.log 2>&1 &
nohup ros2 run topic_tools relay /imu/data_raw /imu/data_reliable --ros-args -p reliability:=reliable > /tmp/relay_imu.log 2>&1 &
sleep 1

echo "🧠 [6/7] 启动 ICP 激光里程计前端..."
nohup ros2 run rtabmap_odom icp_odometry \
    --ros-args \
    -r scan:=/scan_reliable \
    -r scan_cloud:=/dummy_cloud \
    -r odom:=/odom_lidar \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p publish_tf:=false \
    -p expected_update_rate:=15.0 > /tmp/icp.log 2>&1 &

echo "🎯 [7/7] 启动 EKF 核心融合引擎 (消除11.6°偏航漂移)..."
nohup ros2 run robot_localization ekf_node \
    --ros-args \
    --params-file ~/rtls_project/Indoor/ekf.yaml > /tmp/ekf.log 2>&1 &

echo "=================================================="
echo "✅ 底座系统已就绪！"
echo "📊 查看数据流请执行: ros2 topic echo /odometry/filtered"
echo "📂 日志文件存储路径: /tmp/*.log"
echo "=================================================="
