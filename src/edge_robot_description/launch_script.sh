#!/bin/bash
# Script to run the description package with one file

cleanup() {
    echo "Cleaning up..."
    pkill -f "ros2|rviz2|robot_state_publisher|joint_state_publisher"
}

trap 'cleanup' SIGINT SIGTERM

echo "Sourcing ROS2..."
source /opt/ros/jazzy/setup.bash
source ~/edge_ws/install/setup.bash

# echo "Launching URDF in RViz..."
# ros2 launch urdf_tutorial display.launch.py \
#     model:=/home/kunal-jazzy/edge_ws/src/edge_robot_description/urdf/rover.urdf.xacro


echo "Launching gazebo and Rviz..."
ros2 launch edge_robot_bringup robot.launch.py

wait