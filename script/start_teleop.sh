#!/bin/bash

# Çıkışta (Ctrl+C) arka plan süreçlerini temizlemek için fonksiyon
cleanup() {
    echo -e "\n[!] Çıkış yapılıyor. Arka plandaki Gazebo, PX4 ve Agent süreçleri sonlandırılıyor..."
    killall -9 px4 ruby gz MicroXRCEAgent 2>/dev/null
    exit 0
}
# SIGINT (Ctrl+C) ve SIGTERM sinyallerini yakala
trap cleanup SIGINT SIGTERM

echo "Eski süreçler temizleniyor..."
killall -9 px4 ruby gz MicroXRCEAgent 2>/dev/null

PROJECT_DIR="/home/furkan/drone_project"
PX4_DIR="/home/furkan/PX4-Autopilot"

# Log klasörünü oluştur
LOG_DIR="${PROJECT_DIR}/logs"
mkdir -p ${LOG_DIR}

# ROS ve Workspace env
source /opt/ros/jazzy/setup.bash
source ${PROJECT_DIR}/install/setup.bash

# Gecikmeli başlatma için pencereleri tmux, gnome-terminal vb. yerine 
# arka planda (&) çalıştırıp loglarını projeye yazdıracağız.

echo "1) MicroXRCEAgent başlatılıyor..."
MicroXRCEAgent udp4 -p 8888 > ${LOG_DIR}/microdds.log 2>&1 &
sleep 2

# Gazebo Env
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PROJECT_DIR}/worlds:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds"

echo "2) Gazebo başlatılıyor..."
# İlk olarak Gazebo'yu kendi dünyamızla başlatıyoruz (Bu komut hem server hem GUI'yi açar)
gz sim "${PROJECT_DIR}/worlds/test_building.sdf" > ${LOG_DIR}/gazebo.log 2>&1 &

echo "Gazebo'nun hazır olması bekleniyor..."
# Gazebo topiclerinin aktif olmasını bekle ki PX4 başlatıldığında Gazebo'yu görüp bağlanabilsin
while ! gz topic -l | grep -q "/clock"; do
  sleep 1
done
echo "Gazebo hazır."

echo "3) PX4 başlatılıyor..."
cd ${PX4_DIR}
# Kameranın otomatik olarak drona kilitlenmesini (Follow mode) engelle:
export PX4_GZ_NO_FOLLOW=1
# Dronun başlangıç (spawn) koordinatlarını ayarla (x,y,z,roll,pitch,yaw)
export PX4_GZ_MODEL_POSE="10.00,-2.00,0.62,0,0,0"
# PX4 arka planda başlatılıyor. Otomatik olarak halihazırda çalışan Gazebo'yu tespit edip bağlanacak.
make px4_sitl gz_x500_lidar > ${LOG_DIR}/px4_gazebo.log 2>&1 &
sleep 5

echo "4) Teleop Python Scripti Başlatılıyor..."
echo "======================================="
echo "NOT: Gazebo'da PLAY (Oynat) tuşuna basmayı GZ GUI üzerinden unutmayın!"
echo "Hazır olduğunuzda 't' tuşu ile kalkış yapabilirsiniz."
echo "======================================="

cd ${PROJECT_DIR}
python3 src/teleop.py