#!/bin/bash
# PX4 SITL + Gazebo başlat
# Mod: bash run_px4_sitl.sh default | custom (default: custom)
# default  → default world + x500_lidar (3D LiDAR + kamera)
# custom   → test_building world + x500_lidar (3D LiDAR + kamera)
# bare     → default world + x500 (sensörsüz, hızlı test)
#
# Ön koşul: PX4 derlenmiş olmalı (make px4_sitl_default)
# Model: ~/drone_project/models/x500_lidar → PX4 models dizinine symlink
# Airframe: 4022_gz_x500_lidar (PX4 ROMFS'e eklenmiş)
set -e

source /opt/ros/jazzy/setup.bash

export PX4_ROOT="$HOME/PX4-Autopilot"

MODE="${1:-custom}"

cd ${PX4_ROOT}

if [ "$MODE" = "bare" ]; then
    echo "🚁 PX4 SITL — DEFAULT WORLD + x500 (sensörsüz)"
    make px4_sitl gz_x500

elif [ "$MODE" = "default" ]; then
    echo "🚁 PX4 SITL — DEFAULT WORLD + x500_lidar (3D LiDAR + kamera)"
    make px4_sitl gz_x500_lidar

else
    echo "🚁 PX4 SITL — CUSTOM WORLD (test_building) + x500_lidar"

    ln -sf "$HOME/drone_project/worlds/test_building.sdf" \
           "${PX4_ROOT}/Tools/simulation/gz/worlds/test_building_world.sdf" 2>/dev/null || true

    export PX4_GZ_WORLD=test_building_world
    export PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0"

    make px4_sitl gz_x500_lidar
fi
