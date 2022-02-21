#! /bin/bash

# build all the packages from ws_ros1, ws_ros2 and ws_bridge

rm -rf ../logs
mkdir ../logs
terminator --title="ROS1 build" --working-directory="/root/rt2_assignment_1/ws_ros1" -e "source ./build.sh > ../logs/ws_ros1_build.log"
terminator --title="ROS2 build" --working-directory="/root/rt2_assignment_1/ws_ros2" -e "source ./build.sh > ../logs/ws_ros2_build.log"
terminator --working-directory="/root/rt2_assignment_1/ws_bridge" --title="BRIDGE build" -e "source ./build_bridge_base.sh > ../logs/ws_bridge_build.log"
