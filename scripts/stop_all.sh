#!/bin/bash
###############################################################################
# stop_all.sh — Drone SLAM Simülasyonu Tüm Süreçleri Durdur
# ──────────────────────────────────────────────────────────
# start_all.sh tarafından başlatılan tüm süreçleri temizler.
#
# Kullanım:
#   bash scripts/stop_all.sh
###############################################################################

RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
NC='\033[0m'

info() { echo -e "${CYAN}[INFO]${NC}  $1"; }
ok()   { echo -e "${GREEN}[ OK ]${NC}  $1"; }

echo ""
echo -e "${RED}╔══════════════════════════════════════════════╗${NC}"
echo -e "${RED}║   Drone Simülasyonu Durduruluyor...          ║${NC}"
echo -e "${RED}╚══════════════════════════════════════════════╝${NC}"
echo ""

# ── PID dosyalarından oku ve kapat ──
for name in rviz slam bridge dds px4; do
    pidfile="/tmp/drone_sim_${name}.pid"
    if [ -f "$pidfile" ]; then
        pid=$(cat "$pidfile")
        if kill -0 "$pid" 2>/dev/null; then
            info "$name durduruluyor (PID: $pid)..."
            kill "$pid" 2>/dev/null
            sleep 0.5
            kill -9 "$pid" 2>/dev/null || true
        fi
        rm -f "$pidfile"
    fi
done

# ── İlgili tüm süreçleri temizle ──
info "Kalan süreçler temizleniyor..."

pkill -f "ros2 run px4_offboard" 2>/dev/null || true
pkill -f "ros2 launch drone_sim" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "odom_publisher" 2>/dev/null || true
pkill -f "offboard_control" 2>/dev/null || true
pkill -f "drone_teleop" 2>/dev/null || true
pkill -f "rtabmap" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "MicroXRCEAgent" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true

# PX4 + Gazebo (bunları en son kapat)
sleep 1
pkill -f "px4$" 2>/dev/null || true
pkill -f "px4 " 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ruby" 2>/dev/null || true

sleep 1

# ── Kontrol ──
REMAINING=$(pgrep -f "px4|gz sim|MicroXRCE|rtabmap|rviz2|parameter_bridge" 2>/dev/null | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    info "Zorla kapatılıyor ($REMAINING süreç kaldı)..."
    pkill -9 -f "px4" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "ruby" 2>/dev/null || true
    pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
    pkill -9 -f "rtabmap" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    sleep 1
fi

# ── Log dosyalarını temizle (opsiyonel) ──
# rm -f /tmp/px4_sitl.log /tmp/dds_agent.log /tmp/bridge.log /tmp/slam.log /tmp/rviz.log

# ── Stale PX4 parametre dosyalarını temizle ──
rm -f "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters"*.bson 2>/dev/null || true

ok "Tüm süreçler durduruldu!"
echo ""
