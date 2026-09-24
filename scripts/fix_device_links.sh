#!/bin/bash
echo "===== Fixing device links (Mixed Ports) ====="

# Replace the existing device links.
sudo rm -f /dev/imu /dev/gnss /dev/lidar

# IMU on the first native USB controller.
sudo ln -s /dev/serial/by-path/platform-xhci-hcd.0-usb-0:2:1.0-port0 /dev/imu

# GNSS on the second native USB controller.
sudo ln -s /dev/serial/by-path/platform-xhci-hcd.1-usb-0:2:1.0-port0 /dev/gnss

# LiDAR on the PCIe USB expansion board.
sudo ln -s /dev/serial/by-path/platform-1000110000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0 /dev/lidar

# Grant device access to all users.
sudo chmod 777 /dev/imu /dev/gnss /dev/lidar

ls -l /dev/imu /dev/gnss /dev/lidar
echo "===== Device links fixed ====="
