#!/bin/bash
echo ""
echo "Remap the serial port of the roboclaw to roboclaw"
sudo cp `rospack find swarmus_pioneer`/../../scripts/40-swarmus_pioneer.rules /etc/udev/rules.d

echo "Call the rplidar udev set script:"
sh `rospack find rplidar_ros`/scripts/create_udev_rules.sh
echo ""
