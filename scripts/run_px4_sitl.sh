#!/bin/bash
# ─────────────────────────────────────────────────
# PX4 SITL + Gazebo + bizim world dosyamız ile başlat
# Container içinde çalıştırın:
#   bash /root/drone_project/scripts/run_px4_sitl.sh
# ─────────────────────────────────────────────────
set -e

source /opt/ros/jazzy/setup.bash

# PX4 ve model path'leri
export PX4_ROOT=/opt/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH="/root/drone_project/models:${PX4_ROOT}/Tools/simulation/gz/models:${PX4_ROOT}/Tools/simulation/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_ROOT}/build/px4_sitl_default/build_gz_plugins"

# PX4 SITL parametreleri
export PX4_GZ_WORLD=test_building_world
export PX4_GZ_MODEL=x500_lidar
export PX4_GZ_MODEL_POSE="0,0,1.0,0,0,0"
export PX4_SIM_MODEL=gz_x500

echo "🚁 PX4 SITL başlatılıyor..."
echo "   World:  test_building_world"
echo "   Model:  x500_lidar (x500 + 3D LiDAR + Kamera)"
echo "   Spawn:  (0, 0, 1.0)"
echo ""
echo "   GZ_SIM_RESOURCE_PATH: $GZ_SIM_RESOURCE_PATH"
echo ""

# World dosyamızı PX4'ün bulabileceği yere symlink
ln -sf /root/drone_project/worlds/test_building.sdf \
       ${PX4_ROOT}/Tools/simulation/gz/worlds/test_building_world.sdf 2>/dev/null || true

cd ${PX4_ROOT}
make px4_sitl gz_x500_lidar 2>/dev/null || {
    echo ""
    echo "⚠️  'gz_x500_lidar' airframe bulunamadı, varsayılan gz_x500 kullanılıyor..."
    echo "   Sensörler (lidar, kamera) yine de x500_lidar modelinden yüklenecek."
    echo ""
    make px4_sitl gz_x500
}
