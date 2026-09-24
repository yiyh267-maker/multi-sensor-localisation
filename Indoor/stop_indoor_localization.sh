#!/bin/bash

PID_FILE="$HOME/rtls_project/Indoor/indoor_localization.pids"

if [ ! -f "$PID_FILE" ]; then
  echo "No PID file found."
  exit 0
fi

echo "Stopping indoor localization..."

while read -r pid; do
  if ps -p "$pid" > /dev/null 2>&1; then
    kill "$pid"
  fi
done < "$PID_FILE"

rm -f "$PID_FILE"

pkill -f rplidar_node
pkill -f rf2o_laser_odometry_node
pkill -f static_transform_publisher
pkill -f imu_publisher
pkill -f imu_filter_madgwick
pkill -f ekf_node

echo "Stopped."
