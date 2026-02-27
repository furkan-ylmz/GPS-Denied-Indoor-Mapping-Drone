#!/bin/bash
# ─────────────────────────────────────────────────
# Container içinde ROS 2 workspace'i build et
# Bu scripti container içinde çalıştırın:
#   bash /root/drone_project/scripts/build_workspace.sh
# ─────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=/root/drone_project/models

cd /root/drone_project

echo "🔨 ROS 2 workspace build ediliyor..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

echo "✅ Workspace build tamamlandı!"
echo ""
echo "Kullanmak için:"
echo "  source /root/drone_project/install/setup.bash"
