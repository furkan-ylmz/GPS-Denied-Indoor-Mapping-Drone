#!/bin/bash
# ==============================================================================
# Autonomous Mapping Drone System — Bağımlılık ve Ortam Kurulum Betiği
# Hedef İşletim Sistemi: Ubuntu 24.04 LTS (Noble Numbat)
# Hedef ROS 2 Sürümü : ROS 2 Jazzy Jalisco
# ==============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=====================================================================${NC}"
echo -e "${BLUE}  Autonomous Mapping Drone System — Sistem & Bağımlılık Kurulumu    ${NC}"
echo -e "${BLUE}=====================================================================${NC}"

# 1. İşletim Sistemi Kontrolü (OS Verification)
if [[ "$(uname -s)" != "Linux" ]]; then
    echo -e "${RED}[HATA] Bu proje doğrudan Windows veya macOS üzerinde çalıştırılamaz!${NC}"
    echo -e "${YELLOW}Windows kullanıyorsanız lütfen WSL 2 (Ubuntu 24.04 LTS) kurun.${NC}"
    exit 1
fi

if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS_NAME=$ID
    OS_VER=$VERSION_ID
else
    echo -e "${RED}[HATA] /etc/os-release bulunamadı. Desteklenmeyen işletim sistemi.${NC}"
    exit 1
fi

echo -e "Tespit edilen işletim sistemi: ${GREEN}${OS_NAME} ${OS_VER}${NC}"

if [[ "$OS_NAME" != "ubuntu" ]] || [[ "$OS_VER" != "24.04" ]]; then
    echo -e "\n${YELLOW}⚠️  UYARI: Bu proje resmi olarak sadece Ubuntu 24.04 LTS (ROS 2 Jazzy) üzerinde test edilmiştir.${NC}"
    echo -e "${YELLOW}  Diğer Ubuntu sürümlerinde (22.04, 26.04 vb.) ROS 2 Jazzy, Gazebo Harmonic ve Nav2 bağımlılıkları doğrudan desteklenmemektedir.${NC}"
    read -p "Yine de devam etmek istiyor musunuz? (e/H): " choice
    case "$choice" in 
      e|E ) echo "Devam ediliyor...";;
      * ) echo "Kurulum iptal edildi."; exit 1;;
    esac
fi

# 2. Sistem Paket Güncellemesi
echo -e "\n${BLUE}[1/4] Sistem paketleri güncelleniyor...${NC}"
sudo apt update

# 3. Temel Araçlar ve ROS 2 Jazzy Bağımlılıkları
echo -e "\n${BLUE}[2/4] ROS 2 Jazzy ve Simülasyon paketleri yükleniyor...${NC}"
sudo apt install -y \
  git \
  cmake \
  build-essential \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-tf2-ros \
  ros-jazzy-visualization-msgs

# 4. rosdep İlklendirme ve Paket Bağımlılıklarının Çözülmesi
echo -e "\n${BLUE}[3/4] rosdep bağımlılıkları taranıyor ve kuruluyor...${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "${SCRIPT_DIR}")"
cd "${WORKSPACE_ROOT}"

rosdep install --from-paths src --ignore-src -r -y || true

# 5. Micro XRCE-DDS Agent Kontrolü
echo -e "\n${BLUE}[4/4] Micro XRCE-DDS Agent kontrol ediliyor...${NC}"
if ! command -v MicroXRCEAgent &> /dev/null; then
    echo -e "${YELLOW}MicroXRCEAgent bulunamadı. Kaynaktan derleniyor...${NC}"
    BUILD_DIR=$(mktemp -d)
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${BUILD_DIR}/Micro-XRCE-DDS-Agent"
    cd "${BUILD_DIR}/Micro-XRCE-DDS-Agent"
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig /usr/local/lib/
    rm -rf "${BUILD_DIR}"
    echo -e "${GREEN}✅ MicroXRCEAgent başarıyla kuruldu.${NC}"
else
    echo -e "${GREEN}✅ MicroXRCEAgent zaten kurulu.${NC}"
fi

echo -e "\n${GREEN}=====================================================================${NC}"
echo -e "${GREEN}  Tüm bağımlılıklar başarıyla yüklendi!                             ${NC}"
echo -e "${GREEN}=====================================================================${NC}"
echo -e "Sonraki adımlar:"
echo -e "1. Eğer henüz kurmadıysanız PX4 SITL'ı kurun: https://docs.px4.io"
echo -e "2. Çalışma alanını derleyin: ${BLUE}colcon build --symlink-install${NC}"
echo -e "3. Ortamı kaynaklayın: ${BLUE}source install/setup.bash${NC}"
echo -e "4. Simülasyonu başlatın: ${BLUE}./script/start_autonomous.sh${NC}"
