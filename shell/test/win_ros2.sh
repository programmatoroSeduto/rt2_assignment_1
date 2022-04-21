#! /bin/bash

source /opt/ros/foxy/setup.sh && source /root/rt2_assignment_1/ws_ros2/install/setup.sh

echo "running position_service on ROS2 ... "
ros2 run rt2_assignment_1 position_service & 
echo "OK"

echo "running state_machine on ROS2 ... "
ros2 run rt2_assignment_1 state_machine &
echo "OK"

echo "running bridge_support in ROS2 ... "
ros2 run ros2_bridge_support_pkg ros2_bridge_support_node