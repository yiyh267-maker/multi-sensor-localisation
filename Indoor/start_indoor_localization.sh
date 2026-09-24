#!/bin/bash

set -e

ROS_SETUP="/opt/ros/jazzy/setup.bash"
LIDAR_WS="$HOME/rtls_project/LiDAR/ros2_ws/install/setup.bash"
INDOOR_WS="$HOME/rtls_project/Indoor/ros2_ws/install/setup.bash"

PID_FILE="$HOME/rtls_project/Indoor/indoor_localization.pids"
LOG_DIR="$HOME/rtls_project/Indoor/logs"

mkdir -p "$LOG_DIR"
rm -f "$PID_FILE"

echo "Starting indoor localization..."

# 1. LiDAR driver
bash -lc "source $ROS_SETUP && source $LIDAR_WS && ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0" \
  > "$LOG_DIR/lidar.log" 2>&1 &
echo $! >> "$PID_FILE"
sleep 3

# 2. Static TF: base_link -> laser
bash -lc "source $ROS_SETUP && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser" \
  > "$LOG_DIR/tf_laser.log" 2>&1 &
echo $! >> "$PID_FILE"
sleep 1

# 3. Static TF: base_link -> imu_link
bash -lc "source $ROS_SETUP && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link" \
  > "$LOG_DIR/tf_imu.log" 2>&1 &
echo $! >> "$PID_FILE"
sleep 1

# 4. rf2o
bash -lc "source $ROS_SETUP && source $LIDAR_WS && source $INDOOR_WS && ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args --params-file $HOME/rtls_project/Indoor/ros2_ws/config/rf2o.yaml" \
  > "$LOG_DIR/rf2o.log" 2>&1 &
echo $! >> "$PID_FILE"
sleep 2

# 5. IMU raw publisher
bash -lc "source $ROS_SETUP && source $INDOOR_WS && ros2 run ybimu_ros2 imu_publisher --ros-args -p port:=/dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0 -p report_rate_hz:=30 -p publish_rate_hz:=30.0" \
  > "$LOG_DIR/imu_raw.log" 2>&1 &
echo $! >> "$PID_FILE"
sleep 2

# 6. Madgwick
bash -lc "source $ROS_SETUP && source $INDOOR_WS && ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -r imu/data_raw:=/imu/data_raw -r imu/data:=/imu/data -p use_mag:=false -p world_frame:=enu -p publish_tf:=false" \
  > "$LOG_DIR/madgwick.log" 2>&1 &
echo $! >> "$PID_FILE"
sleep 2

# 7. EKF
bash -lc "source $ROS_SETUP && source $INDOOR_WS && ros2 run robot_localization ekf_node --ros-args --params-file $HOME/rtls_project/Indoor/ros2_ws/config/ekf.yaml -r __node:=ekf_filter_node" \
  > "$LOG_DIR/ekf.log" 2>&1 &
echo $! >> "$PID_FILE"
sleep 2

echo "Indoor localization started."
echo "PIDs saved to: $PID_FILE"
echo "Logs saved to: $LOG_DIR"
echo
echo "Quick checks:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/rtls_project/Indoor/ros2_ws/install/setup.bash"
echo "  ros2 topic hz /odometry/filtered"
echo "  ros2 topic echo /odometry/filtered --once"
