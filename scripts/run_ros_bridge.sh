#!/bin/bash
# ROS 2 Bridge başlat
set -e

source /opt/ros/jazzy/setup.bash
source "$HOME/drone_project/install/setup.bash"

echo "🌉 ROS 2 Bridge başlatılıyor..."
ros2 launch drone_sim_bringup bridge.launch.py
