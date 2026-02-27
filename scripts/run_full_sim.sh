#!/bin/bash
# ─────────────────────────────────────────────────
# Tam PX4 SITL simülasyonu — TÜM BİLEŞENLER
# Her birini ayrı terminalde çalıştırmanız gerekir.
#
# Bu script sadece sıralamayı gösterir.
# ─────────────────────────────────────────────────

cat << 'EOF'
╔══════════════════════════════════════════════════════════════════╗
║              PX4 SITL Simülasyon - Başlatma Rehberi             ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  4 ayrı terminal açın (docker exec -it drone_sim bash):         ║
║                                                                  ║
║  Terminal 1 — PX4 SITL + Gazebo:                                ║
║    bash /root/drone_project/scripts/run_px4_sitl.sh             ║
║                                                                  ║
║  Terminal 2 — DDS Agent (PX4 ↔ ROS 2):                         ║
║    bash /root/drone_project/scripts/run_dds_agent.sh            ║
║                                                                  ║
║  Terminal 3 — ROS Bridge (Gazebo sensörleri ↔ ROS 2):          ║
║    bash /root/drone_project/scripts/run_ros_bridge.sh           ║
║                                                                  ║
║  Terminal 4 — Drone Kontrol:                                    ║
║    # Offboard otomatik uçuş:                                   ║
║    ros2 run px4_offboard offboard_control                       ║
║                                                                  ║
║    # VEYA klavye ile manuel kontrol:                            ║
║    ros2 run px4_offboard drone_teleop                           ║
║                                                                  ║
║  Terminal 5 (opsiyonel) — RViz:                                 ║
║    rviz2 -d /root/drone_project/src/drone_sim_bringup/config/   ║
║           drone_sim.rviz                                        ║
║                                                                  ║
╠══════════════════════════════════════════════════════════════════╣
║  Faydalı Komutlar:                                              ║
║    ros2 topic list           # Aktif topic'ler                  ║
║    ros2 topic echo /drone/imu --once   # IMU testi              ║
║    ros2 topic hz /drone/lidar/points   # LiDAR frekansı         ║
║    ros2 topic echo /fmu/out/vehicle_status --once               ║
╚══════════════════════════════════════════════════════════════════╝
EOF
