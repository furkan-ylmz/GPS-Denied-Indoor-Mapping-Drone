#!/bin/bash
# ROS 2 workspace build
set -e

source /opt/ros/jazzy/setup.bash
cd "$HOME/drone_project"

# px4_msgs yoksa clone et
if [ ! -d "src/px4_msgs" ]; then
    echo "📦 px4_msgs clone ediliyor..."
    git clone --depth 1 --branch main https://github.com/PX4/px4_msgs.git src/px4_msgs
fi

echo "🔨 Workspace build ediliyor..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

echo "✅ Build tamamlandı!"
echo "  source ~/drone_project/install/setup.bash"
