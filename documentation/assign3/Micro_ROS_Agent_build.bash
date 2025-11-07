#!/bin/bash

# https://github.com/micro-ROS/micro_ros_arduino

# download zip-file of this repository, and add to arduino-IDE

# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir ~/microros_ws
cd ~/microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32


# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
