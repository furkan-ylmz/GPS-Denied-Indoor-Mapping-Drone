#!/bin/bash

# Çıkışta (Ctrl+C) arka plan süreçlerini temizlemek için fonksiyon
cleanup() {
    echo -e "\n[!] Çıkış yapılıyor. Arka plandaki Gazebo, PX4, Agent ve Bridge süreçleri sonlandırılıyor..."
    killall -9 px4 ruby gz MicroXRCEAgent ros2 rtabmap python3 2>/dev/null
    exit 0
}
# SIGINT (Ctrl+C) ve SIGTERM sinyallerini yakala
trap cleanup SIGINT SIGTERM

echo "Eski süreçler temizleniyor..."
killall -9 px4 ruby gz MicroXRCEAgent ros2 rtabmap python3 2>/dev/null

PROJECT_DIR="/home/furkan/drone_project"
PX4_DIR="/home/furkan/PX4-Autopilot"

# Log klasörünü oluştur
LOG_DIR="${PROJECT_DIR}/logs"
mkdir -p ${LOG_DIR}

# ROS ve Workspace env
source /opt/ros/jazzy/setup.bash
source ${PROJECT_DIR}/install/setup.bash

echo "1) MicroXRCEAgent başlatılıyor..."
MicroXRCEAgent udp4 -p 8888 > ${LOG_DIR}/microdds.log 2>&1 &
sleep 2

echo "2) ROS-Gazebo Bridge başlatılıyor..."
# Lidar ve TF verilerini Gazebo'dan ROS2'ye taşımak için bridge başlatıyoruz
ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
    /camera@sensor_msgs/msg/Image[gz.msgs.Image \
    /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    > ${LOG_DIR}/bridge.log 2>&1 &

echo "2.1) PX4 Odometry TF Broadcaster başlatılıyor..."
# Dronun anlık konumunu Haritalama için (odom -> base_link) TF yayını olarak dönüştürür.
python3 ${PROJECT_DIR}/src/px4_tf_broadcaster.py --ros-args -p use_sim_time:=true > ${LOG_DIR}/tf_broadcaster.log 2>&1 &

echo "2.2) Lidar ve Kamera statik bağlantıları kuruluyor..."
# Dron gövdesi ile Lidar arasındaki fiziksel konumu (base_link -> lidar_link)
ros2 run tf2_ros static_transform_publisher 0.15 0 0 0 1.570796 0 base_link lidar_link --ros-args -p use_sim_time:=true > /dev/null 2>&1 &
# Dron gövdesi ile Kamera arasındaki fiziksel konumu (ROS kamera eksenlerine göre: Z ileri, X sağ, Y aşağı)
ros2 run tf2_ros static_transform_publisher 0.11 0 0 -1.570796 0 -1.570796 base_link camera_link --ros-args -p use_sim_time:=true > /dev/null 2>&1 &

echo "2.3) RTAB-Map (3D SLAM / Octomap) başlatılıyor..."
# Gerçek zamanlı haritalama algoritması
ros2 run rtabmap_slam rtabmap \
    --ros-args \
    -p use_sim_time:=true \
    -p delete_db_on_start:=true \
    -p subscribe_depth:=false \
    -p subscribe_rgb:=true \
    -p subscribe_scan_cloud:=true \
    -p approx_sync:=true \
    -p frame_id:=base_link \
    -p map_frame_id:=map \
    -p odom_frame_id:=odom \
    --params-file ${PROJECT_DIR}/config/rtabmap_params.yaml \
    --remap scan_cloud:=/lidar/points \
    --remap rgb/image:=/camera \
    --remap rgb/camera_info:=/camera_info \
    > ${LOG_DIR}/rtabmap.log 2>&1 &

# Gazebo Env
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PROJECT_DIR}/worlds:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds"
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH}:/usr/share/gz"

echo "3) Gazebo başlatılıyor..."
# Snap (VS Code) ortam değişkenlerinin Gazebo GUI'sini bozmasını engellemek için temizliyoruz
unset GTK_PATH GIO_MODULE_DIR LOCPATH GSETTINGS_SCHEMA_DIR XDG_DATA_HOME

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
# Gazebo'da kendi oluşturduğumuz `x500_lidar` modelini kullanması için:
export PX4_GZ_MODEL=x500_lidar
# Dronun başlangıç (spawn) koordinatlarını ayarla (x,y,z,roll,pitch,yaw)
export PX4_GZ_MODEL_POSE="10.00,-2.00,0.62,0,0,0"
# PX4 arka planda başlatılıyor. Otomatik olarak halihazırda çalışan Gazebo'yu tespit edip bağlanacak.
make px4_sitl gz_x500_lidar > ${LOG_DIR}/px4_gazebo.log 2>&1 &
sleep 5

echo "3.5) RViz2 başlatılıyor..."
# Kullanıcının kaydettiği rviz ayar dosyası varsa onunla, yoksa boş rviz başlat
if [ -f "${PROJECT_DIR}/rviz/teleop.rviz" ]; then
    ros2 run rviz2 rviz2 -d ${PROJECT_DIR}/rviz/teleop.rviz --ros-args -p use_sim_time:=true > ${LOG_DIR}/rviz.log 2>&1 &
else
    echo "  [Uyarı] rviz/teleop.rviz dosyası bulunamadı. Boş RViz açılıyor."
    ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true > ${LOG_DIR}/rviz.log 2>&1 &
fi
sleep 2

echo "4) Teleop Python Scripti Başlatılıyor..."
echo "======================================="
echo "NOT: Gazebo'da PLAY (Oynat) tuşuna basmayı GZ GUI üzerinden unutmayın!"
echo "Hazır olduğunuzda 't' tuşu ile kalkış yapabilirsiniz."
echo "======================================="

cd ${PROJECT_DIR}
python3 src/teleop.py