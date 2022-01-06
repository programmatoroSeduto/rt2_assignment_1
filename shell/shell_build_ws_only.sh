#! /bin/bash

# build all the packages from ws_ros1, ws_ros2 and ws_bridge

rm -rf ../logs
mkdir ../logs
terminator --working-directory=../ws_ros1 --title="ROS1 build" -e "source ./build.sh > ../logs/ws_ros1_build.log"
terminator --working-directory=../ws_ros2 --title="ROS2 build" -e "source ./build.sh > ../logs/ws_ros2_build.log"
# terminator --working-directory=../ws_bridge --title="BRIDGE build" -e "source ./build_bridge_base.sh > ../logs/ws_bridge_build.log"
