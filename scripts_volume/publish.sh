#!/bin/bash
# Source the ROS setup script
source /opt/ros/noetic/setup.bash
cd /home/ubuntu/catkin_ws/
catkin build

source /home/ubuntu/catkin_ws/devel/setup.bash
# Run your ROS launch file
roslaunch /home/ubuntu/catkin_ws/src/capstone_robot/launch/capstone_mast_rpi.launch

