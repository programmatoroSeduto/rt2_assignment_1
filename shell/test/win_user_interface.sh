#! /bin/bash

source /opt/ros/noetic/setup.bash && source /root/rt2_assignment_1/ws_ros1/devel/setup.bash

echo "running the user interface on ROS1 ... "
rosrun rt2_assignment1 user_interface.py