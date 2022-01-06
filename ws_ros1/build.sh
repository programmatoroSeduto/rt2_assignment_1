#! /bin/bash

# cd /root/simple_bridge/ws_ros1
source /opt/ros/noetic/setup.bash
rm -rf devel
rm -rf build
# rm build_log.log

set -e

catkin_make
source /root/rt2_assignment_1/ws_ros1/devel/setup.bash
