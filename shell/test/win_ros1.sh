#! /bin/bash

source /opt/ros/noetic/setup.bash && source /root/rt2_assignment_1/ws_ros1/devel/setup.bash

echo "running the launch file 'sim_bridge.launch in ROS1 ... '"
roslaunch rt2_assignment1 sim_bridge.launch &
echo "OK"

echo "running bridge support on ROS1 ... "
rosrun ros1_bridge_support_pkg ros1_bridge_support_node