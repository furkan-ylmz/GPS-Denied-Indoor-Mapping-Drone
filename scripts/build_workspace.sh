#!/bin/bash
# ─────────────────────────────────────────────────
# Container içinde ROS 2 workspace'i build et
# Bu scripti container içinde çalıştırın:
#   bash /root/drone_project/scripts/build_workspace.sh
# ─────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="/root/drone_project/models:/opt/PX4-Autopilot/Tools/simulation/gz/models"

cd /root/drone_project

# px4_msgs yoksa clone et
if [ ! -d "src/px4_msgs" ]; then
    echo "📦 px4_msgs clone ediliyor..."
    git clone --branch main https://github.com/PX4/px4_msgs.git src/px4_msgs
fi

echo "🔨 ROS 2 workspace build ediliyor..."
echo "   Paketler: px4_msgs, px4_offboard, drone_sim_bringup"
echo ""
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

echo ""
echo "✅ Workspace build tamamlandı!"
echo ""
echo "Kullanmak için:"
echo "  source /root/drone_project/install/setup.bash"
