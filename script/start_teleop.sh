#!/bin/bash
# ============================================================
# Otonom Haritalama Dronu — Gerçek Donanım Başlatma Script'i
# Mod: Manuel Kontrol (teleop)
# ============================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Otonom Haritalama Dronu — GERÇEK DONANIM  ${NC}"
echo -e "${CYAN}  Mod: Manuel Kontrol (Teleop)              ${NC}"
echo -e "${CYAN}============================================${NC}"

cleanup() {
    echo -e "\n${YELLOW}[!] Çıkış yapılıyor. RTAB-Map veritabanı güvenli bir şekilde kaydediliyor...${NC}"
    killall -2 rtabmap 2>/dev/null
    sleep 3
    echo -e "${YELLOW}Diğer süreçler sonlandırılıyor...${NC}"
    killall MicroXRCEAgent ros2 python3 2>/dev/null
    sleep 1
    echo -e "${GREEN}[✓] Temizlik tamamlandı.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$PROJECT_DIR"

echo -e "${YELLOW}[1/4] ROS 2 ortamı yükleniyor...${NC}"
source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo -e "${RED}[HATA] Workspace build edilmemiş! Önce colcon build yapın.${NC}"
    exit 1
fi

# USB latency fix
echo -e "${YELLOW}[2/4] USB seri port ayarlanıyor...${NC}"
if [ -e /sys/bus/usb-serial/devices/ttyUSB0/latency_timer ]; then
    echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer > /dev/null
    echo -e "${GREEN}  ✓ USB latency 1ms olarak ayarlandı${NC}"
fi

# Seri port izinleri
echo -e "${YELLOW}[3/4] Seri port izinleri kontrol ediliyor...${NC}"
for port in /dev/ttyUSB0 /dev/ttyAMA0; do
    if [ -e "$port" ]; then
        if [ ! -r "$port" ] || [ ! -w "$port" ]; then
            sudo chmod 666 "$port"
        fi
        echo -e "${GREEN}  ✓ $port erişilebilir${NC}"
    else
        echo -e "${YELLOW}  ⚠ $port bulunamadı${NC}"
    fi
done

echo -e "${YELLOW}[4/4] ROS 2 launch başlatılıyor (mod: manual)...${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  Web Arayüzü: http://localhost:8080       ${NC}"
echo -e "${GREEN}  Teleop aktif — klavye ile kontrol edin    ${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

ros2 launch drone_bringup bringup.launch.py mode:=manual &
LAUNCH_PID=$!

# Teleop'u ayrı terminal'de başlat (klavye girişi gerekli)
sleep 12
echo -e "${CYAN}Teleop başlatılıyor...${NC}"
ros2 run px4_offboard teleop &

wait $LAUNCH_PID
