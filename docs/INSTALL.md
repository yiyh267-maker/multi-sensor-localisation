# Installation and build

This procedure is derived from the source and launch scripts. It has not been executed on a fresh Raspberry Pi for this release. Use Ubuntu 24.04 ARM64 with ROS2 Jazzy on the Pi. The scripts expect `~/rtls_project` and `/opt/ros/jazzy/setup.bash`.

## 1. ROS2 and system dependencies

Configure the ROS2 apt repository and install ROS2 Jazzy using the [official Ubuntu instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). The [Jazzy platform documentation](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html) lists Ubuntu Noble 64-bit ARM as supported.

With the ROS repository configured, the following package list covers the main demo's directly identified runtime/build dependencies:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools \
  python3-venv python3-pip python3-serial python3-pynmea2 \
  python3-smbus python3-evdev libeigen3-dev libboost-dev libopencv-dev \
  ros-jazzy-cv-bridge ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-robot-localization ros-jazzy-rtabmap-ros \
  ros-jazzy-usb-cam ros-jazzy-topic-tools
```

This is a source-derived dependency list, not a captured apt lockfile. Optional Cartographer, Madgwick and vendor navigation examples require additional packages and are outside the main-demo build below. Package metadata is not fully consistent: for example, the included RPLIDAR manifest lists `rclpy_components`, and the IMU license fields differ between files. A blanket `rosdep install` over every retained vendor package may therefore require follow-up; do not treat it as proof of a reproducible environment.

## 2. Build the selected workspaces

Use a fresh terminal with the system Python interpreter, outside a Python virtual environment. Avoid sourcing old project overlays first.

```bash
source /opt/ros/jazzy/setup.bash

cd ~/rtls_project/LiDAR/ros2_ws
colcon build --symlink-install --packages-select rplidar_ros
source install/setup.bash

cd ~/rtls_project/LIO2D/ros2_ws
colcon build --symlink-install --packages-select \
  gnss_ros2 ybimu_ros2 image_converter_cpp
source install/setup.bash

cd ~/rtls_project/Indoor/ros2_ws
colcon build --symlink-install --packages-select rf2o_laser_odometry
source install/setup.bash
```

Stop at any build error and inspect its output. The selected build excludes the alternative `Indoor` IMU node and the additional `LiDAR/src` vendor workspace. Building all packages recursively from the repository root can discover duplicate package names.

## 3. Recreate the dashboard environment

The GNSS and IMU ROS nodes use system Python, so serial/NMEA dependencies are installed through apt above. Only the dashboard uses the virtual environment expected by the launch script:

```bash
cd ~/rtls_project
python3 -m venv --system-site-packages display_bridge/venv
display_bridge/venv/bin/python -m pip install -r requirements-web.txt
```

Using the system interpreter and its ROS packages matters for binary compatibility; see the [ROS2 Python package guide](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html). The two pinned web versions were found in the supplied environment. Transitive dependencies are not locked, and this environment has not been tested on the current robot.

## 4. Configure and start on the robot

Use [.env.example](../.env.example) and [the map configuration guide](../RELEASE_SETUP.md) to set `AMAP_API_KEY`. Then, from a launch terminal:

```bash
cd ~/rtls_project
source /opt/ros/jazzy/setup.bash
source LiDAR/ros2_ws/install/setup.bash
source LIO2D/ros2_ws/install/setup.bash
source Indoor/ros2_ws/install/setup.bash
set -a
source .env
set +a
mkdir -p logs
bash scripts/start_demo1.sh
```

Explicitly sourcing the LiDAR overlay is important because the main script invokes its driver before sourcing LIO2D. Visit `http://<robot-ip>:5001` on the same network. The IP printed at the end of the existing script is a historical value, not device discovery.

The script probes serial devices, changes device permissions, starts gamepad motor control, terminates previous nodes and deletes the previous RTAB-Map database/logs. Check hardware connections, I2C availability, joystick detection, camera device and mounting transforms before use. Do not run it as an offline documentation preview.

To stop the demo:

```bash
bash ~/rtls_project/scripts/stop_demo1.sh
```

The stop script also removes the RTAB-Map database and project logs. Save any experiment data you want to retain before stopping or restarting.

## 5. Inspect live output when hardware is available

```bash
ros2 topic list
ros2 topic hz /scan_reliable
ros2 topic hz /imu/data_reliable
ros2 topic hz /odom_rf2o
ros2 topic hz /fix
ros2 topic echo /odometry/filtered --once
ros2 topic echo /mapPath --once
```

Run frequency commands one at a time and stop each with Ctrl+C. A topic list confirms discovery, not positioning accuracy. Record timings and compare known physical paths separately; see [VALIDATION.md](VALIDATION.md).
