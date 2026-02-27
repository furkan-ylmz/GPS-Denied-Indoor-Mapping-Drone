#!/bin/bash
# ─────────────────────────────────────────────────
# Micro XRCE-DDS Agent başlat
# PX4 <-> ROS 2 haberleşme köprüsü
# Container içinde ayrı terminal:
#   bash /root/drone_project/scripts/run_dds_agent.sh
# ─────────────────────────────────────────────────

echo "🔗 Micro XRCE-DDS Agent başlatılıyor..."
echo "   PX4 ↔ ROS 2 köprüsü (UDP port 8888)"
echo ""

MicroXRCEAgent udp4 -p 8888
