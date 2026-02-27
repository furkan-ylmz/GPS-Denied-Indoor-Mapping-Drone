#!/bin/bash
# PX4 SITL + Gazebo başlat
# Mod: bash run_px4_sitl.sh default | custom (default: custom)
set -e

source /opt/ros/jazzy/setup.bash

export PX4_ROOT="$HOME/PX4-Autopilot"
export GZ_SIM_RESOURCE_PATH="$HOME/drone_project/models:${PX4_ROOT}/Tools/simulation/gz/models:${PX4_ROOT}/Tools/simulation/gz/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_ROOT}/build/px4_sitl_default/build_gz_plugins"

MODE="${1:-custom}"

if [ "$MODE" = "default" ]; then
    echo "🚁 PX4 SITL — DEFAULT WORLD"
    cd ${PX4_ROOT}
    make px4_sitl gz_x500
else
    echo "🚁 PX4 SITL — CUSTOM WORLD (test_building)"

    ln -sf "$HOME/drone_project/worlds/test_building.sdf" \
           "${PX4_ROOT}/Tools/simulation/gz/worlds/test_building_world.sdf" 2>/dev/null || true

    export PX4_GZ_WORLD=test_building_world
    export PX4_GZ_MODEL_POSE="0,0,0.5,0,0,0"

    cd ${PX4_ROOT}
    make px4_sitl gz_x500
fi
