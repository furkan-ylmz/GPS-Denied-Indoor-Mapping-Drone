#!/bin/bash

# Çıkışta (Ctrl+C) arka plan süreçlerini temizlemek için fonksiyon
cleanup() {
    echo -e "\n[!] Çıkış yapılıyor. Lütfen bekleyin, RTAB-Map veritabanı güvenli bir şekilde kaydediliyor..."
    killall -2 rtabmap 2>/dev/null
    sleep 3
    echo "Diğer süreçler sonlandırılıyor..."
    killall px4 ruby gz MicroXRCEAgent ros2 rviz2 python3 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Eski süreçler temizleniyor..."
killall px4 ruby gz MicroXRCEAgent ros2 rtabmap rviz2 python3 2>/dev/null

# Get the absolute path of the workspace root (one level up from this script)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
ENABLE_OCR="${ENABLE_OCR:-false}"
for arg in "$@"; do
    case "${arg}" in
        ocr|--ocr)
            ENABLE_OCR=true
            ;;
        no-ocr|--no-ocr)
            ENABLE_OCR=false
            ;;
        *)
            echo "Bilinmeyen arguman: ${arg}"
            echo "Kullanim: $0 [ocr|--ocr]"
            exit 1
            ;;
    esac
done
OCR_IMAGE_TOPIC="${OCR_IMAGE_TOPIC:-/camera}"
OCR_MAP_JSON_PATH="${OCR_MAP_JSON_PATH:-${PROJECT_DIR}/src/door_ocr/docs/door_labels.json}"
mkdir -p "${PROJECT_DIR}/logs" "$(dirname "${OCR_MAP_JSON_PATH}")"

# ROS ve Workspace env
source /opt/ros/jazzy/setup.bash
source ${PROJECT_DIR}/install/setup.bash

PX4_DIR="${HOME}/PX4-Autopilot"
export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${PROJECT_DIR}/worlds:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds"

# Snap (VS Code) ortam değişkenlerinin Gazebo GUI'sini bozmasını engellemek için temizliyoruz
unset GTK_PATH GIO_MODULE_DIR LOCPATH GSETTINGS_SCHEMA_DIR XDG_DATA_HOME

echo "Gazebo başlatılıyor..."
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH}:/usr/share/gz"
gz sim "${PROJECT_DIR}/worlds/test_building.sdf" > ${PROJECT_DIR}/logs/gazebo.log 2>&1 &

echo "======================================================================"
echo "  OTONOM NAVİGASYON MODU"
echo "  Sistem ROS 2 Launch üzerinden arka planda başlatılıyor..."
echo "  (SLAM + Nav2 + Drone Navigator)"
echo "  OCR modu: ${ENABLE_OCR}"
echo "======================================================================"

# Tüm arka plan düğümlerini (Simülasyon, SLAM, Bridge, TF, RViz, Nav2, DroneNavigator) launch ile başlatıyoruz
ros2 launch drone_sim_bringup bringup.launch.py \
  "ocr:=${ENABLE_OCR}" \
  "ocr_image_topic:=${OCR_IMAGE_TOPIC}" \
  "ocr_map_json_path:=${OCR_MAP_JSON_PATH}" &
LAUNCH_PID=$!

echo "Gazebo'nun hazır olması bekleniyor..."
# Gazebo topiclerinin aktif olmasını bekle ki PX4 başlatıldığında Gazebo'yu görüp bağlanabilsin
while ! gz topic -l | grep -q "/clock"; do
  sleep 1
done
echo "Gazebo hazır."

echo "PX4 başlatılıyor..."
cd "${PX4_DIR}"
export PX4_GZ_NO_FOLLOW=1
export PX4_GZ_MODEL=x500_lidar
export PX4_GZ_MODEL_POSE="10.00,-2.00,0.62,0,0,0"
make px4_sitl gz_x500_lidar > ${PROJECT_DIR}/logs/px4_gazebo.log 2>&1 &
sleep 5

echo "======================================================================"
echo "  OTONOM NAVİGASYON SİSTEMİ HAZIR"
echo "======================================================================"
echo ""
echo "  1. Gazebo'da PLAY (▶) tuşuna basın"
echo "  2. Dron otomatik olarak kalkacak ve 1.5m yükseklikte hover edecek"
echo "  3. RViz'de harita oluştuğunda '2D Goal Pose' ile hedef verin"
echo "  4. Dron A* rotasını MPPI ile takip edecek"
echo ""
echo "  Çıkış için: Ctrl+C"
echo "======================================================================"

# Launch çalışırken bekle
wait $LAUNCH_PID
cleanup
