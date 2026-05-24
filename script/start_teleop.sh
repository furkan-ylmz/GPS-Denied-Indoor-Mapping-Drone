#!/bin/bash

# Çıkışta (Ctrl+C) arka plan süreçlerini temizlemek için fonksiyon
cleanup() {
    echo -e "\n[!] Çıkış yapılıyor. Lütfen bekleyin, RTAB-Map veritabanı güvenli bir şekilde kaydediliyor..."
    killall -2 rtabmap 2>/dev/null
    sleep 3
    echo "Diğer süreçler sonlandırılıyor..."
    killall px4 ruby gz MicroXRCEAgent ros2 python3 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Eski süreçler temizleniyor..."
killall px4 ruby gz MicroXRCEAgent ros2 rtabmap python3 2>/dev/null

PROJECT_DIR="/home/furkan/drone_project"

# ROS ve Workspace env
source /opt/ros/jazzy/setup.bash
source ${PROJECT_DIR}/install/setup.bash

export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PROJECT_DIR}/worlds:/home/furkan/PX4-Autopilot/Tools/simulation/gz/models:/home/furkan/PX4-Autopilot/Tools/simulation/gz/worlds"

# Snap (VS Code) ortam değişkenlerinin Gazebo GUI'sini bozmasını engellemek için temizliyoruz
unset GTK_PATH GIO_MODULE_DIR LOCPATH GSETTINGS_SCHEMA_DIR XDG_DATA_HOME

echo "Gazebo başlatılıyor..."
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH}:/usr/share/gz"
gz sim "${PROJECT_DIR}/worlds/test_building.sdf" > ${PROJECT_DIR}/logs/gazebo.log 2>&1 &

echo "=========================================================="
echo "Sistem ROS 2 Launch üzerinden arka planda başlatılıyor..."
echo "=========================================================="

# Tüm arka plan düğümlerini (Simülasyon, SLAM, Bridge, TF, RViz) launch ile başlatıyoruz
ros2 launch drone_sim_bringup bringup.launch.py &
LAUNCH_PID=$!

echo "Gazebo'nun hazır olması bekleniyor..."
# Gazebo topiclerinin aktif olmasını bekle ki PX4 başlatıldığında Gazebo'yu görüp bağlanabilsin
while ! gz topic -l | grep -q "/clock"; do
  sleep 1
done
echo "Gazebo hazır."

echo "PX4 başlatılıyor..."
cd /home/furkan/PX4-Autopilot
export PX4_GZ_NO_FOLLOW=1
export PX4_GZ_MODEL=x500_lidar
export PX4_GZ_MODEL_POSE="10.00,-2.00,0.62,0,0,0"
make px4_sitl gz_x500_lidar > ${PROJECT_DIR}/logs/px4_gazebo.log 2>&1 &
sleep 5

echo "======================================="
echo "NOT: Gazebo'da PLAY (Oynat) tuşuna basmayı GZ GUI üzerinden unutmayın!"
echo "Hazır olduğunuzda 't' tuşu ile kalkış yapabilirsiniz."
echo "======================================="

# Yeni oluşturduğumuz resmi ROS 2 paketi üzerinden teleop'u başlatıyoruz
ros2 run px4_offboard teleop

# Teleop kapanırsa launch dosyasını da kapat
kill $LAUNCH_PID
cleanup
