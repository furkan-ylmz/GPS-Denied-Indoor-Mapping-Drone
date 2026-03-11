#!/bin/bash
###############################################################################
# start_all.sh — Drone SLAM Simülasyonu Tam Başlatma Script'i
# ─────────────────────────────────────────────────────────────
# Sıra:
#   1. PX4 SITL + Gazebo   (arka plan)
#   2. Micro XRCE-DDS Agent (arka plan)
#   3. ROS-Gazebo Bridge    (arka plan)
#   4. SLAM (odom + RTAB-Map) (arka plan)
#   5. Nav2 Planner          (arka plan, sadece nav modunda)
#   6. RViz2 görselleştirme  (arka plan, opsiyonel)
#   7. Uçuş kontrolü         (interaktif)
#
# Kullanım:
#   bash scripts/start_all.sh             # Otomatik hover (offboard_control)
#   bash scripts/start_all.sh nav         # Nav2 otonom nav (RViz2'den hedef ver)
#   bash scripts/start_all.sh explore     # Otonom keşif (frontier exploration)
#   bash scripts/start_all.sh teleop      # Klavye ile kontrol
#   bash scripts/start_all.sh novis       # RViz2 kapalı
#   bash scripts/start_all.sh nav novis   # Nav2 + RViz yok
#
# Kapatma:
#   bash scripts/stop_all.sh
#   veya Ctrl+C (foreground process'i kapatır)
###############################################################################
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PX4_DIR="$HOME/PX4-Autopilot"
DDS_DIR="$HOME/Micro-XRCE-DDS-Agent"

# ── Argümanlar ──
MODE="offboard"      # offboard | teleop | nav
VIS="true"           # true | false
for arg in "$@"; do
    case "$arg" in
        teleop)  MODE="teleop" ;;
        nav)     MODE="nav" ;;
        explore) MODE="explore" ;;
        novis)   VIS="false" ;;
    esac
done

# ── Renk kodları ──
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
ok()    { echo -e "${GREEN}[ OK ]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
err()   { echo -e "${RED}[ERR ]${NC}  $1"; }

# ── Temizlik: varsa eski süreçleri kes ──
info "Eski süreçler temizleniyor..."
bash "$SCRIPT_DIR/stop_all.sh" 2>/dev/null || true
sleep 2

# ── ROS 2 ortamı ──
source /opt/ros/jazzy/setup.bash
if [ -f "$PROJECT_DIR/install/setup.bash" ]; then
    source "$PROJECT_DIR/install/setup.bash"
fi

echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║        Drone SLAM Simülasyonu Başlatılıyor          ║${NC}"
echo -e "${GREEN}║   Mod: ${YELLOW}${MODE}${GREEN}  |  RViz: ${YELLOW}${VIS}${GREEN}                            ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════════╝${NC}"
echo ""

###############################################################################
# 1) PX4 SITL + Gazebo
###############################################################################
info "1/7 PX4 SITL + Gazebo başlatılıyor..."
cd "$PX4_DIR"

# World symlink'i güncelle
ln -sf "$PROJECT_DIR/worlds/test_building.sdf" \
       "$PX4_DIR/Tools/simulation/gz/worlds/test_building_world.sdf" 2>/dev/null || true

# PX4 her zaman sunucu modunda başlar (HEADLESS=1)
# Gazebo GUI client ayrıca başlatılacak
HEADLESS=1 PX4_GZ_WORLD=test_building_world PX4_GZ_MODEL_POSE="0,0,0.65,0,0,0" \
  make px4_sitl gz_x500_lidar > /tmp/px4_sitl.log 2>&1 &
PX4_PID=$!
echo "$PX4_PID" > /tmp/drone_sim_px4.pid

info "     Gazebo yükleniyor (30s)..."
for i in $(seq 1 30); do
    if grep -q "Ready for takeoff" /tmp/px4_sitl.log 2>/dev/null || \
       grep -q "Takeoff detected" /tmp/px4_sitl.log 2>/dev/null || \
       grep -q "\[gz_bridge\]" /tmp/px4_sitl.log 2>/dev/null; then
        break
    fi
    sleep 1
    printf "\r     Bekleniyor... %ds/30s" "$i"
done
echo ""

if pgrep -f "gz sim" > /dev/null 2>&1; then
    ok "PX4 SITL + Gazebo çalışıyor (PID: $PX4_PID)"
else
    warn "Gazebo henüz başlamadı, 15s daha bekleniyor..."
    sleep 15
    if pgrep -f "gz sim" > /dev/null 2>&1; then
        ok "PX4 SITL + Gazebo çalışıyor"
    else
        err "Gazebo başlatılamadı! Log: /tmp/px4_sitl.log"
        exit 1
    fi
fi

# Gazebo GUI client — novis değilse ayrıca başlat
if [ "$VIS" != "false" ]; then
    info "     Gazebo GUI client başlatılıyor..."
    sleep 2
    # -g: sadece GUI client, mevcut sunucuya bağlanır
    gz sim -g > /tmp/gz_gui.log 2>&1 &
    GZ_GUI_PID=$!
    echo "$GZ_GUI_PID" > /tmp/drone_sim_gz_gui.pid
    sleep 2
    if kill -0 "$GZ_GUI_PID" 2>/dev/null; then
        ok "Gazebo GUI çalışıyor (PID: $GZ_GUI_PID)"
    else
        warn "Gazebo GUI başlatılamadı (WSL2 display sorunu olabilir)"
    fi
fi

###############################################################################
# 2) Micro XRCE-DDS Agent
###############################################################################
info "2/7 Micro XRCE-DDS Agent başlatılıyor..."
cd "$DDS_DIR"
MicroXRCEAgent udp4 -p 8888 > /tmp/dds_agent.log 2>&1 &
DDS_PID=$!
echo "$DDS_PID" > /tmp/drone_sim_dds.pid
sleep 3

if kill -0 "$DDS_PID" 2>/dev/null; then
    ok "DDS Agent çalışıyor (PID: $DDS_PID)"
else
    err "DDS Agent başlatılamadı!"
    exit 1
fi

###############################################################################
# 3) ROS-Gazebo Bridge
###############################################################################
info "3/7 ROS-Gazebo Bridge başlatılıyor..."
cd "$PROJECT_DIR"
ros2 launch drone_sim_bringup bridge.launch.py > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!
echo "$BRIDGE_PID" > /tmp/drone_sim_bridge.pid
sleep 3

if kill -0 "$BRIDGE_PID" 2>/dev/null; then
    ok "Bridge çalışıyor (PID: $BRIDGE_PID)"
else
    err "Bridge başlatılamadı!"
    exit 1
fi

###############################################################################
# 4) SLAM (odom_publisher + RTAB-Map)
###############################################################################
info "4/7 SLAM başlatılıyor (odom_publisher + RTAB-Map)..."
ros2 launch drone_sim_bringup slam.launch.py > /tmp/slam.log 2>&1 &
SLAM_PID=$!
echo "$SLAM_PID" > /tmp/drone_sim_slam.pid
sleep 5

if kill -0 "$SLAM_PID" 2>/dev/null; then
    ok "SLAM çalışıyor (PID: $SLAM_PID)"
else
    err "SLAM başlatılamadı! Log: /tmp/slam.log"
    exit 1
fi

###############################################################################
# 5) Nav2 Planner (sadece nav modunda)
###############################################################################
if [ "$MODE" = "nav" ] || [ "$MODE" = "explore" ]; then
    info "5/7 Nav2 Planner başlatılıyor..."
    ros2 launch drone_sim_bringup nav2.launch.py > /tmp/nav2.log 2>&1 &
    NAV2_PID=$!
    echo "$NAV2_PID" > /tmp/drone_sim_nav2.pid
    sleep 5

    if kill -0 "$NAV2_PID" 2>/dev/null; then
        ok "Nav2 Planner çalışıyor (PID: $NAV2_PID)"
    else
        warn "Nav2 Planner başlatılamadı — direkt navigasyon kullanılacak"
    fi
else
    info "5/7 Nav2 Planner atlandı (mod: $MODE)"
fi

###############################################################################
# 6) RViz2 Görselleştirme
###############################################################################
if [ "$VIS" = "true" ]; then
    info "6/7 RViz2 başlatılıyor..."
    ros2 launch drone_sim_bringup view_slam.launch.py > /tmp/rviz.log 2>&1 &
    RVIZ_PID=$!
    echo "$RVIZ_PID" > /tmp/drone_sim_rviz.pid
    sleep 2
    if kill -0 "$RVIZ_PID" 2>/dev/null; then
        ok "RViz2 çalışıyor (PID: $RVIZ_PID)"
    else
        warn "RViz2 başlatılamadı (GUI mevcut olmayabilir)"
    fi
else
    info "6/7 RViz2 atlandı (novis)"
fi

###############################################################################
# 7) Uçuş Kontrolü (foreground)
###############################################################################
echo ""
echo -e "${GREEN}════════════════════════════════════════════════════════${NC}"

if [ "$MODE" = "nav" ]; then
    info "7/7 Drone Navigator başlatılıyor (Nav2 otonom navigasyon)..."
    echo -e "${GREEN}════════════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${YELLOW}  Drone otomatik kalkış yapacak, sonra HOVER'da bekleyecek.${NC}"
    echo -e "${YELLOW}  RViz2'de '2D Nav Goal' ile harita üzerinde hedef belirleyin.${NC}"
    echo -e "${YELLOW}  Ctrl+C ile iniş yapılır.${NC}"
    echo ""
    ros2 run px4_offboard drone_navigator

elif [ "$MODE" = "explore" ]; then
    info "7/7 Otonom Keşif başlatılıyor (Frontier Explorer + Navigator)..."
    echo -e "${GREEN}════════════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${YELLOW}  Drone otomatik kalkış yapıp bina içini keşfedecek.${NC}"
    echo -e "${YELLOW}  Frontier tabanlı otonom keşif aktif.${NC}"
    echo -e "${YELLOW}  Ctrl+C ile iniş yapılır.${NC}"
    echo ""
    # Navigator arka planda çalışır
    ros2 run px4_offboard drone_navigator &
    NAV_PID=$!
    echo "$NAV_PID" > /tmp/drone_sim_navigator.pid
    sleep 2
    ok "Navigator çalışıyor (PID: $NAV_PID)"
    # Frontier Explorer foreground'da
    ros2 run px4_offboard frontier_explorer

elif [ "$MODE" = "teleop" ]; then
    info "7/7 Drone Teleop başlatılıyor (klavye kontrolü)..."
    echo -e "${GREEN}════════════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${YELLOW}  Teleop başlamadan önce 't' ile arm+takeoff yapın!${NC}"
    echo ""
    ros2 run px4_offboard drone_teleop

else
    info "7/7 Offboard Control başlatılıyor (otomatik hover)..."
    echo -e "${GREEN}════════════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${YELLOW}  Drone otomatik olarak 1.0m'ye yükselecek ve hover yapacak.${NC}"
    echo -e "${YELLOW}  Ctrl+C ile iniş yapılır.${NC}"
    echo ""
    ros2 run px4_offboard offboard_control
fi
