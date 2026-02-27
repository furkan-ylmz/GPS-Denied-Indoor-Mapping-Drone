#!/bin/bash
# ─────────────────────────────────────────────────
# ROS 2 bridge + offboard kontrolü başlat
# Container içinde ayrı terminal:
#   bash /root/drone_project/scripts/run_ros_bridge.sh
# ─────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash
source /root/drone_project/install/setup.bash 2>/dev/null || {
    echo "⚠️  Workspace build edilmemiş. Önce build edin:"
    echo "    bash /root/drone_project/scripts/build_workspace.sh"
    exit 1
}
export GZ_SIM_RESOURCE_PATH="/root/drone_project/models:/opt/PX4-Autopilot/Tools/simulation/gz/models"

echo "🌉 ROS 2 Bridge başlatılıyor..."
echo "   Sensör topic'leri: LiDAR, Kamera, IMU, Odometry"
echo ""

ros2 launch drone_sim_bringup bridge.launch.py
