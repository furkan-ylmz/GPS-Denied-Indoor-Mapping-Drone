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

# Build sonrası eski logları temizle (Son 5 buildi tut)
echo "🧹 Eski loglar temizleniyor..."
ls -1dt log/build.* 2>/dev/null | tail -n +6 | xargs rm -rf 2>/dev/null || true
ls -1dt log/build_* 2>/dev/null | tail -n +6 | xargs rm -rf 2>/dev/null || true

source install/setup.bash

echo "✅ Build tamamlandı!"
echo "  source ~/drone_project/install/setup.bash"
