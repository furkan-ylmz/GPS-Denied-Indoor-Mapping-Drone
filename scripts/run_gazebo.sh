#!/bin/bash
# ─────────────────────────────────────────────────
# Sadece Gazebo simülasyonunu başlat (GUI test için)
# Container içinde çalıştırın:
#   bash /root/drone_project/scripts/run_gazebo.sh
# ─────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=/root/drone_project/models

echo "🚀 Gazebo simülasyonu başlatılıyor..."
echo "   World: test_building.sdf"
echo "   Model path: $GZ_SIM_RESOURCE_PATH"
echo ""

gz sim /root/drone_project/worlds/test_building.sdf -r
