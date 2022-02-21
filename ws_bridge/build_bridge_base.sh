#! /bin/bash

cd /root/rt2_assignment_1/ws_bridge
rm -rf src 
rm -rf build
rm -rf install
rm -rf log
# rm build_log.log

set -e

mkdir src
cd src
git clone https://github.com/ros2/ros1_bridge.git
cd ..

source /opt/ros/foxy/setup.sh
source /opt/ros/noetic/setup.bash
colcon build --packages-select ros1_bridge --cmake-force-configure

source ./install/setup.sh
ros2 run ros1_bridge dynamic_bridge --print-pairs
