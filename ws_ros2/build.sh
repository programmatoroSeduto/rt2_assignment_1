#! /bin/bash

source /opt/ros/foxy/setup.sh
rm -rf log
rm -rf install
rm -rf build

set -e 

# BUILD rt2_assignment_1
clear && colcon build --packages-select rt2_assignment_1
source /root/rt2_assignment_1/ws_ros2/install/setup.sh

# BUILD ros2_bridge_support_pkg
colcon build --packages-select ros2_bridge_support_pkg
source /root/rt2_assignment_1/ws_ros2/install/setup.sh
