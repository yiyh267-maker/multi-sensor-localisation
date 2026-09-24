#!/bin/bash

# Discover serial devices and load their port mapping.
echo "[0] 正在执行硬件指纹识别..."
python3 ~/rtls_project/scripts/hw_handshake.py
if [ -f /tmp/hw_map.env ]; then
    source /tmp/hw_map.env
else
    echo "硬件指纹扫描失败！"
    exit 1
fi

LIDAR_PORT=$LIDAR_DEV
IMU_PORT=$IMU_DEV
CAMERA_DEV="/dev/video0"

# Reset processes, device links and the SLAM database.
echo "[1] 清理残留进程并重置 SLAM 数据库..."
sudo rm -f /dev/imu
sudo ln -s $IMU_PORT /dev/imu
# Start with a fresh RTAB-Map database.
rm -f ~/.ros/rtabmap.db  

sudo killall -9 rplidar_node imu_publisher relay rf2o_laser_odometry_node ekf_node static_transform_publisher usb_cam_node_exe rtabmap 2>/dev/null
# Remove previous logs and serial lock files.
sudo rm -f ~/rtls_project/logs/*.log /var/lock/LCK..ttyUSB*
sleep 2
sudo chmod 777 /dev/imu $LIDAR_PORT $IMU_PORT $CAMERA_DEV

# Start gamepad control.
echo "[2] 启动手柄后台控制服务..."
sudo -b python3 ~/rtls_project/Car/gamepad_teleop_diff.py > ~/rtls_project/logs/gamepad_teleop.log 2>&1

# Start the LiDAR and IMU drivers.
echo "[3] 启动传感器驱动..."
source /opt/ros/jazzy/setup.bash
# LiDAR
nohup ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=$LIDAR_PORT \
    -p serial_baudrate:=115200 \
    -p frame_id:=laser > ~/rtls_project/logs/lidar.log 2>&1 &

# IMU
source ~/rtls_project/LIO2D/ros2_ws/install/setup.bash
nohup ros2 run ybimu_ros2 imu_publisher > ~/rtls_project/logs/imu.log 2>&1 &
sleep 5

# Publish the sensor mounting transforms.
echo "[4] 发布静态 TF 变换..."
source /opt/ros/jazzy/setup.bash
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 1.5708 0 0 base_link laser > ~/rtls_project/logs/tf_laser.log 2>&1 &
nohup ros2 run tf2_ros static_transform_publisher 0.015 -0.057 -0.081 0 0 0 base_link imu_link > ~/rtls_project/logs/tf_imu.log 2>&1 &
# Camera optical frame.
nohup ros2 run tf2_ros static_transform_publisher 0.06 0.048 -0.052 -1.5708 0 -1.5708 base_link default_cam > ~/rtls_project/logs/tf_cam.log 2>&1 &
sleep 2

# Relay sensor topics with reliable QoS.
echo "[5] 启动数据质量桥接..."
nohup ros2 run topic_tools relay /scan /scan_reliable --ros-args -p reliability:=reliable > ~/rtls_project/logs/relay_scan.log 2>&1 &
nohup ros2 run topic_tools relay /imu/data_raw /imu/data_reliable --ros-args -p reliability:=reliable > ~/rtls_project/logs/relay_imu.log 2>&1 &
sleep 1

# Start camera capture and YUYV conversion.
echo "[6] 启动相机节点..."
nohup ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:=$CAMERA_DEV \
    -p image_width:=640 \
    -p image_height:=480 \
    -p framerate:=20.0 \
    -p pixel_format:="yuyv" \
    -p camera_frame_id:="default_cam" \
    -p io_method:=mmap \
    -p buffer_size:=2 > ~/rtls_project/logs/camera.log 2>&1 &
sleep 2

nohup bash -c "source /opt/ros/jazzy/setup.bash && source ~/rtls_project/LIO2D/ros2_ws/install/setup.bash && ros2 run image_converter_cpp yuyv_to_bgr_node" > ~/rtls_project/logs/yuyv_to_bgr.log 2>&1 &
sleep 1

# Start RF2O laser odometry.
echo "[7] 启动 RF2O 激光里程计..."
source ~/rtls_project/Indoor/ros2_ws/install/setup.bash
nohup ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
    --params-file ~/rtls_project/Indoor/ros2_ws/config/rf2o.yaml \
    -r odom:=/odom_lidar > ~/rtls_project/logs/rf2o.log 2>&1 &
sleep 2

# Fuse RF2O odometry and IMU measurements.
echo "[8] 启动 EKF 核心..."
nohup ros2 run robot_localization ekf_node --ros-args \
    --params-file ~/rtls_project/Indoor/ekf.yaml \
    -r __node:=ekf_filter_node > ~/rtls_project/logs/ekf.log 2>&1 &
sleep 2

# Start RTAB-Map with scan, image and filtered odometry inputs.
echo "[9] 启动 RTAB-Map 全局优化..."

nohup ros2 run rtabmap_slam rtabmap --ros-args \
    -p frame_id:=base_link \
    -p subscribe_scan:=true \
    -p subscribe_rgb:=true \
    -p subscribe_depth:=false \
    -p approx_sync:=true \
    -p approx_sync_max_interval:=1.0 \
    -p sync_queue_size:=400 \
    -p wait_for_transform:=1.0 \
    -p Mem/IncrementalMemory:="'true'" \
    -p RGBD/Enabled:="'true'" \
    -p Mem/BinDataKept:="'false'" \
    -p Rtabmap/MemoryThr:="'0'" \
    -p Vis/FeatureType:="'6'" \
    -p Kp/MaxFeatures:="'1000'" \
    -p Kp/DetectorStrategy:="'6'" \
    -p RGBD/OptimizeMaxError:="'1.0'" \
    -p Vis/MinInliers:="'70'" \
    -p Vis/EpipolarGeometryVar:="'1.0'" \
    -p Rtabmap/LoopThr:="'0.45'" \
    -p Icp/CorrespondenceRatio:="'0.6'" \
    -p Icp/MaxTranslation:="'2.0'" \
    -p Icp/MaxCorrespondenceDistance:="'1.0'" \
    -p Rtabmap/DetectionRate:="'0'" \
    -p RGBD/OptimizeFromGraphEnd:="'true'" \
    -p Reg/Strategy:="'1'" \
    -p Vis/EstimationType:="'2'" \
    -r rgb/image:=/image_raw_bgr \
    -r rgb/camera_info:=/camera_info \
    -r scan:=/scan_reliable \
    -r odom:=/odometry/filtered > ~/rtls_project/logs/rtabmap.log 2>&1 &

# Start the GNSS publisher, which configures the receiver on initialization.
echo "[10] 启动 GNSS 硬件配置与 ROS 2 发布节点..."
source ~/rtls_project/LIO2D/ros2_ws/install/setup.bash
nohup ros2 run gnss_ros2 gnss_publisher > ~/rtls_project/logs/gnss_node.log 2>&1 &
sleep 2

# Start the ROS2-to-WebSocket dashboard bridge.
echo "[11] 启动 4G 远程图传与地图 UI 桥接服务..."
nohup bash -c "cd ~/rtls_project/display_bridge && source venv/bin/activate && source /opt/ros/jazzy/setup.bash && python3 bridge_server.py" > ~/rtls_project/logs/bridge_v2.log 2>&1 &

echo "======================================================"
echo "全部级联节点启动完成。"
echo "请执行: ros2 topic hz /info 验证最终输出。"
echo "192.168.0.78:5001"
echo "======================================================"
