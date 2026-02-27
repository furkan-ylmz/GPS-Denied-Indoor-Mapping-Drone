#!/bin/bash
# ─────────────────────────────────────────────────
# Launch dosyası ile tüm simülasyonu başlat
# Container içinde çalıştırın:
#   bash /root/drone_project/scripts/run_sim.sh
# ─────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash
source /root/drone_project/install/setup.bash 2>/dev/null || true
export GZ_SIM_RESOURCE_PATH=/root/drone_project/models

echo "🚀 Tam simülasyon başlatılıyor (Gazebo + Bridge + RViz)..."

ros2 launch drone_sim_bringup sim_bringup.launch.py
