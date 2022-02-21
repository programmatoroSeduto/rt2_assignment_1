# SOURCE ROS2
source /opt/ros/foxy/setup.sh && clear
source /opt/ros/foxy/setup.sh && source /root/rt2_assignment_1/ws_ros2/install/setup.sh && clear

# SOURCE ROS1
source /opt/ros/noetic/setup.bash && source /root/rt2_assignment_1/ws_ros1/devel/setup.bash

# LAUNCH ROS MASTER
terminator -e "source /opt/ros/noetic/setup.bash && source /root/rt2_assignment_1/ws_ros1/devel/setup.bash && roscore"

# SOURCE ROS2 and ws_ros2
source /opt/ros/foxy/setup.sh && source /root/rt2_assignment_1/ws_ros2/install/setup.sh && clear

# CREATE A ROS2 C++ PACKAGE AND BUILD
ros2 pkg create --build-type ament_cmake <package_name>
clear && colcon build --packages-select rt2_assignment_1
clear && /root/rt2_assignment_1/ws_ros2/build.sh && source /opt/ros/foxy/setup.sh && source /root/rt2_assignment_1/ws_ros2/install/setup.sh

# OTHER COMMANDS
ros2 service call /user_interface rt2_assignment_1/Command "{command: 'ciao'}"






# LAUNCH THE PROJECT
# first launch the nodes in ROS2; source ROS2
source /opt/ros/foxy/setup.sh && source /root/rt2_assignment_1/ws_ros2/install/setup.sh
ros2 run rt2_assignment_1 position_service &
ros2 run rt2_assignment_1 state_machine &
ros2 run ros2_bridge_support_pkg ros2_bridge_support_node
# check if everything is running fine on the ROS2 side
source /opt/ros/foxy/setup.sh && source /root/rt2_assignment_1/ws_ros2/install/setup.sh

# source ROS1
source /opt/ros/noetic/setup.bash && source /root/rt2_assignment_1/ws_ros1/devel/setup.bash
roscore
# the low bridge
# terminator -e "source /root/rt2_assignment_1/shell/shell_bridge.sh"
source /root/rt2_assignment_1/shell/shell_bridge.sh
# nodes side ROS1
source /opt/ros/noetic/setup.bash && source /root/rt2_assignment_1/ws_ros1/devel/setup.bash
roslaunch rt2_assignment1 sim_bridge.launch
# bridge ROS1 side
source /opt/ros/noetic/setup.bash && source /root/rt2_assignment_1/ws_ros1/devel/setup.bash
rosrun ros1_bridge_support_pkg ros1_bridge_support_node

# check if everything is working fine
source /opt/ros/noetic/setup.bash && source /root/rt2_assignment_1/ws_ros1/devel/setup.bash
rosservice call /user_interface "command: 'start'"

#tests for ROS2 bridge support 
ros2 topic echo /bridge_service/user_interface_response &
ros2 topic pub /bridge_service/user_interface_request std_msgs/String "data: /{//}"


