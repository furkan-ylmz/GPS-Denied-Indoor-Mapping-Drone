#!/bin/bash
# ============================================================
# Otonom Haritalama Dronu — Gerçek Donanım Başlatma Script'i
# Mod: Otonom Navigasyon (autonomous)
# ============================================================

set -e

# Renkli çıktı
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}  Otonom Haritalama Dronu — GERÇEK DONANIM  ${NC}"
echo -e "${CYAN}  Mod: Otonom Navigasyon                    ${NC}"
echo -e "${CYAN}============================================${NC}"

# Çıkışta arka plan süreçlerini temizlemek için fonksiyon
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

# ── Ortam Değişkenleri ──────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$PROJECT_DIR"

echo -e "${YELLOW}[1/5] ROS 2 ortamı yükleniyor...${NC}"
source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo -e "${RED}[HATA] Workspace build edilmemiş! Önce colcon build yapın.${NC}"
    exit 1
fi

# ── USB Seri Port Latency Fix ───────────────────────────────
echo -e "${YELLOW}[2/5] USB seri port ayarlanıyor...${NC}"
if [ -e /sys/bus/usb-serial/devices/ttyUSB0/latency_timer ]; then
    echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer > /dev/null
    echo -e "${GREEN}  ✓ USB latency 1ms olarak ayarlandı${NC}"
else
    echo -e "${YELLOW}  ⚠ ttyUSB0 bulunamadı — LiDAR bağlı değil mi?${NC}"
fi

# ── Seri Port İzinleri ──────────────────────────────────────
echo -e "${YELLOW}[3/5] Seri port izinleri kontrol ediliyor...${NC}"
for port in /dev/ttyUSB0 /dev/ttyAMA0; do
    if [ -e "$port" ]; then
        if [ ! -r "$port" ] || [ ! -w "$port" ]; then
            sudo chmod 666 "$port"
            echo -e "${GREEN}  ✓ $port izinleri ayarlandı${NC}"
        else
            echo -e "${GREEN}  ✓ $port erişilebilir${NC}"
        fi
    else
        echo -e "${YELLOW}  ⚠ $port bulunamadı${NC}"
    fi
done

# ── Sensör Bağlantı Kontrolü ───────────────────────────────
echo -e "${YELLOW}[4/5] Sensör bağlantıları kontrol ediliyor...${NC}"
READY=true
if [ ! -e /dev/ttyUSB0 ]; then
    echo -e "${RED}  ✗ LiDAR (ttyUSB0) bağlı değil!${NC}"
    READY=false
fi
if [ ! -e /dev/ttyAMA0 ]; then
    echo -e "${RED}  ✗ Pixhawk UART (ttyAMA0) bağlı değil!${NC}"
    READY=false
fi
# Kamera CSI kontrolü
if ! ls /dev/video* &>/dev/null; then
    echo -e "${YELLOW}  ⚠ Kamera cihazı bulunamadı (libcamera ile erişilebilir olabilir)${NC}"
fi

if [ "$READY" = false ]; then
    echo -e "${RED}[HATA] Kritik sensörler bağlı değil. Bağlantıları kontrol edin.${NC}"
    read -p "Yine de devam etmek istiyor musunuz? (e/H): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Ee]$ ]]; then
        exit 1
    fi
fi

# ── ROS 2 Launch ────────────────────────────────────────────
echo -e "${YELLOW}[5/5] ROS 2 launch başlatılıyor (mod: autonomous)...${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  Web Arayüzü: http://localhost:8080       ${NC}"
echo -e "${GREEN}  Operatör komutu bekleniyor: ARM           ${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

ros2 launch drone_bringup bringup.launch.py mode:=autonomous &
LAUNCH_PID=$!

# Launch sürecinin bitmesini bekle
wait $LAUNCH_PID
