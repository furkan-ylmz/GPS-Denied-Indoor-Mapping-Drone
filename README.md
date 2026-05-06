# Kapalı Alan Otonom Drone Projesi

> 2 katlı kapalı bir binada 3D LiDAR SLAM ile harita çıkartan, Nav2 ile otonom navigasyon yapan ve kapı numaralarını tanıyacak drone sistemi.

## Genel Bakış

| Bileşen | Açıklama | Durum |
|---------|----------|-------|
| **3D LiDAR SLAM** | RTAB-Map ICP ile kapalı alan haritalama | ✅ Çalışıyor |
| **Otonom Keşif & Navigasyon** | Frontier-based keşif + Nav2 path planning + PX4 kontrolü | ✅ Çalışıyor |
| **Offboard Uçuş** | PX4 offboard mod, arm/takeoff/hover/navigate/land | ✅ Çalışıyor |
| **Sensör Füzyonu** | 16 kanal 3D LiDAR + RGB kamera | ✅ Çalışıyor |
| **Kapı Tanıma (OCR)** | Kamera ile kapı numaralarını okuma | ⬜ Planlanıyor |

## Sistem Mimarisi

```
WSL2 (Ubuntu 24.04)
═══════════════════════════════════════════════════════════════
  PX4 SITL ◄────► Micro XRCE-DDS Agent ◄────► ROS 2 Jazzy
  (gz_x500_lidar)        (UDP 8888)              │
      │                                          ├─ odom_publisher (NED→ENU + TF)
      │                                          ├─ offboard_control / drone_navigator
      ▼                                          ├─ RTAB-Map (ICP SLAM)
  Gazebo Harmonic ◄──► ros_gz_bridge ◄──────────►├─ Nav2 Planner (path planning)
      │                                          └─ RViz2 (görselleştirme)
   ┌──┴──┐
   │Bina │   Sensörler:              PX4 Topics:
   │ SDF │   /drone/lidar/points     /fmu/in/trajectory_setpoint
   └─────┘   /drone/camera/image_raw /fmu/out/vehicle_odometry

  TF Zinciri:  map → odom → base_link → lidar_link
                                      → camera_link
```

## Donanım (Gerçek Sistem — Hedef)

| Donanım | Görev |
|---------|-------|
| Raspberry Pi 5 | Ana bilgisayar (companion computer) |
| Hailo 26T AI HAT | Yapay zeka çıkarımı (OCR, nesne tespiti) |
| Pixhawk 6C | Uçuş kontrol kartı (flight controller) |
| 3D LiDAR | Ortam haritalaması |
| RGB Kamera | Kapı numarası tanıma |

## Yazılım Ortamı

| Yazılım | Versiyon |
|---------|----------|
| Host OS | Windows + WSL2 (Ubuntu 24.04 Noble) |
| ROS 2 | Jazzy Jalisco |
| Simülatör | Gazebo Harmonic (gz-sim) |
| Uçuş Kontrolü | PX4 Autopilot SITL (main branch) |
| PX4 ↔ ROS 2 | Micro XRCE-DDS Agent (UDP 8888) |
| SLAM | RTAB-Map v0.22.1 (ICP, 3D LiDAR) |
| Navigasyon | Nav2 (NavfnPlanner + Global Costmap) |
| OCR | EasyOCR / PaddleOCR *(planlanan)* |

## Proje Yapısı

```
drone_project/
├── README.md
├── .gitignore
│
├── models/
│   ├── x500_lidar/                  # PX4 drone + 3D LiDAR + kamera
│   │   ├── model.config
│   │   └── model.sdf               # 16-ch LiDAR (25m, 10Hz) + RGB cam
│   └── test/                        # Bina modeli
│       ├── model.config / model.sdf
│       └── meshes/eg1.obj + eg1.mtl
│
├── src/
│   ├── px4_offboard/                # Uçuş kontrol paketi (ament_python)
│   │   ├── package.xml / setup.py
│   │   └── px4_offboard/
│   │       ├── offboard_control.py  # Otomatik arm + takeoff + hover
│   │       ├── drone_teleop.py      # Klavye ile uçuş (wasd)
│   │       ├── frontier_explorer.py # Otonom sınır (frontier) tabanlı keşif
│   │       ├── map_cleaner.py       # Harita gürültü filtresi
│   │       ├── odom_publisher.py    # PX4 NED→ENU + TF (odom→base_link)
│   │       └── drone_navigator.py   # Nav2 + PX4 otonom hedef takibi
│   │
│   ├── drone_sim_bringup/           # Launch & config paketi (ament_cmake)
│   │   ├── package.xml / CMakeLists.txt
│   │   ├── config/
│   │   │   ├── slam_view.rviz       # SLAM + Navigasyon RViz konfigürasyonu
│   │   │   └── nav2_params.yaml     # Nav2 planner + costmap parametreleri
│   │   └── launch/
│   │       ├── bridge.launch.py     # Gazebo ↔ ROS 2 sensör köprüsü + TF
│   │       ├── slam.launch.py       # RTAB-Map SLAM + odom_publisher
│   │       ├── nav2.launch.py       # Nav2 planner + lifecycle manager
│   │       └── view_slam.launch.py  # RViz2 görselleştirme
│   │
│   └── px4_msgs/                    # PX4 mesaj tanımları (auto-clone)
│
├── scripts/
│   ├── start_all.sh                 # Tek komutla tüm sistemi başlat
│   ├── stop_all.sh                  # Tüm süreçleri temiz durdur
│   ├── build_workspace.sh           # colcon build
│   ├── run_px4_sitl.sh              # PX4 SITL başlatma
│   └── run_ros_bridge.sh            # Sensör bridge
│
└── worlds/
    └── test_building.sdf            # Gazebo world
```

## Ön Gereksinimler

| Bileşen | Kurulum | Konum |
|---------|---------|-------|
| ROS 2 Jazzy | `sudo apt install ros-jazzy-desktop` | `/opt/ros/jazzy` |
| Gazebo + Bridge | `sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge` | sistem |
| PX4 Autopilot | `git clone --recursive .../PX4-Autopilot && make px4_sitl_default` | `~/PX4-Autopilot` |
| XRCE-DDS Agent | [Build from source](https://micro-xrce-dds.docs.eprosima.com/) | `~/Micro-XRCE-DDS-Agent` |
| RTAB-Map | `sudo apt install ros-jazzy-rtabmap-ros` | sistem |
| Nav2 | `sudo apt install ros-jazzy-nav2-planner ros-jazzy-nav2-lifecycle-manager ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-navfn-planner` | sistem |

`~/.bashrc` ayarları:
```bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/drone_project/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/PX4-Autopilot/build/px4_sitl_default/build_gz_plugins"
```

## Kurulum ve Build

```bash
cd ~/drone_project
bash scripts/build_workspace.sh
# veya:
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 1  # px4_msgs OOM önlemi
```

## Çalıştırma

### Tek Komutla Başlatma (Önerilen)

```bash
cd ~/drone_project

# Otonom Keşif — Frontier tabanlı haritalama
bash scripts/start_all.sh explore

# Offboard Hover — otomatik kalkış + havada bekle
bash scripts/start_all.sh

# Klavye Teleop — wasd kontrolü
bash scripts/start_all.sh teleop

# RViz2 olmadan (headless test)
bash scripts/start_all.sh explore novis
bash scripts/start_all.sh novis
```

### Durdurma

```bash
bash scripts/stop_all.sh    # Tüm süreçleri temiz durdur
# veya Ctrl+C               # Foreground process'i durdur
```

### Manuel Başlatma (Ayrı Terminaller)

```bash
# Terminal 1 — PX4 SITL + Gazebo
cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500_lidar

# Terminal 2 — DDS Agent
cd ~/Micro-XRCE-DDS-Agent && MicroXRCEAgent udp4 -p 8888

# Terminal 3 — Bridge + SLAM
source ~/drone_project/install/setup.bash
ros2 launch drone_sim_bringup bridge.launch.py
# (yeni terminal) ros2 launch drone_sim_bringup slam.launch.py

# Terminal 4 — Nav2 + Navigator
ros2 launch drone_sim_bringup nav2.launch.py
# (yeni terminal) ros2 run px4_offboard drone_navigator

# Terminal 5 — RViz2
ros2 launch drone_sim_bringup view_slam.launch.py
```

## RViz2'de Canlı Görselleştirme

| Katman | Renk | Topic | Açıklama |
|--------|------|-------|----------|
| 2D Map | Gri tonları | `/map` | RTAB-Map occupancy grid |
| 3D Cloud Map | Mavi→Kırmızı | `/cloud_map` | Biriken 3D nokta bulutu |
| LiDAR Points | Yeşil | `/drone/lidar/points` | Canlı 16-ch LiDAR |
| Planned Path | Mor | `/drone/planned_path` | Nav2 planlanan yol |
| Global Costmap | Renkli | `/global_costmap/costmap` | Engel + inflation |
| Odometry | Sarı oklar | `/drone/odom` | Drone izlediği yol |
| Camera | RGB | `/drone/camera/image_raw` | Ön kamera akış |
| TF | RGB eksenleri | TF | map→odom→base_link zinciri |

**Nav2 ile hedef gönderme:** RViz2 toolbar'da **"2D Nav Goal"** butonuna tıkla, haritada hedef noktayı seç.

## Teknik Detaylar

### PX4 Topic'leri (DDS Versiyonlu)

PX4 main branch'te DDS topic'leri versiyonludur:
- Versiyon 0 → son ek yok: `/fmu/out/vehicle_odometry`
- Versiyon N > 0 → `_vN`: `/fmu/out/vehicle_status_v2`, `/fmu/out/vehicle_local_position_v1`

| Topic | Yön | Mesaj | Açıklama |
|-------|-----|-------|----------|
| `/fmu/out/vehicle_local_position_v1` | PX4→ROS | VehicleLocalPosition | Drone pozisyonu (NED) |
| `/fmu/out/vehicle_status_v2` | PX4→ROS | VehicleStatus | Arm durumu, nav state |
| `/fmu/out/vehicle_odometry` | PX4→ROS | VehicleOdometry | Odometry (NED/FRD) |
| `/fmu/in/offboard_control_mode` | ROS→PX4 | OffboardControlMode | Offboard parametreleri |
| `/fmu/in/trajectory_setpoint` | ROS→PX4 | TrajectorySetpoint | Hedef pozisyon (NED) |
| `/fmu/in/vehicle_command` | ROS→PX4 | VehicleCommand | Arm, mod, land |

### QoS Ayarları

PX4 uXRCE-DDS: **BEST_EFFORT + VOLATILE** (tüm pub/sub için).

### Koordinat Dönüşümleri

```
PX4 (NED/FRD)           odom_publisher.py           ROS 2 (ENU/FLU)
──────────────    ────────────────────────    ─────────────────────
x = North         x_enu = y_ned (East)        x = East
y = East          y_enu = x_ned (North)       y = North
z = Down          z_enu = -z_ned (Up)         z = Up
```

### PX4 Airframe (4022_gz_x500_lidar)

Özel airframe parametreleri (RC/GCS kontrolsüz otonom uçuş):
```
param set-default NAV_DLL_ACT 0       # GCS kaybında aksiyon yok
param set-default NAV_RCL_ACT 0       # RC kaybında aksiyon yok
param set-default COM_RCL_EXCEPT 4    # Offboard modda RC kaybı muaf
param set-default COM_RC_IN_MODE 4    # Manuel kontrol devre dışı
```

## Debug

```bash
# Topic listesi
ros2 topic list | grep -E "drone|fmu|map|costmap"

# Drone durumu (QoS önemli!)
ros2 topic echo /fmu/out/vehicle_status_v2 \
  --qos-reliability best_effort --qos-durability volatile --once

# Pozisyon
ros2 topic echo /fmu/out/vehicle_local_position_v1 \
  --qos-reliability best_effort --qos-durability volatile --once

# SLAM harita
ros2 topic echo /map --once | grep -E "resolution|width|height"

# TF ağacı
ros2 run tf2_ros tf2_echo map base_link

# Nav2'ye goal gönderme (CLI)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"

# Log dosyaları
tail -f /tmp/px4_sitl.log   # PX4
tail -f /tmp/slam.log       # RTAB-Map
tail -f /tmp/nav2.log       # Nav2
```

## Yol Haritası

### Faz 1 — Temel Simülasyon Ortamı ✅
- [x] WSL2 + ROS 2 Jazzy + Gazebo Harmonic kurulumu
- [x] PX4 SITL build ve entegrasyonu
- [x] Micro XRCE-DDS Agent (PX4 ↔ ROS 2 iletişimi)
- [x] Bina modelini Gazebo world'e entegre etme
- [x] Temel uçuş doğrulaması

### Faz 2 — Offboard Drone Kontrolü ✅
- [x] QoS düzeltmesi (BEST_EFFORT + VOLATILE)
- [x] PX4 versiyonlu topic isimleri (_v1, _v2)
- [x] offboard_control.py — otomatik arm/takeoff/hover state machine
- [x] drone_teleop.py — klavye ile drone kontrolü (wasd + 20Hz heartbeat)

### Faz 3 — Sensör Entegrasyonu ✅
- [x] x500_lidar modeli (16-ch 3D LiDAR + RGB kamera SDF)
- [x] PX4 airframe 4022_gz_x500_lidar + CMakeLists kaydı
- [x] Gazebo ↔ ROS 2 bridge (lidar/points, camera, camera_info, clock)
- [x] Static TF (base_link → lidar_link, camera_link)
- [x] LiDAR doğrulama: 16x360 points, 8Hz, frame_id: lidar_link

### Faz 4 — 3D SLAM (Haritalama) ✅
- [x] odom_publisher.py — PX4 NED/FRD → ROS ENU/FLU + TF odom→base_link
- [x] RTAB-Map ICP SLAM (Reg/Strategy=1, point-to-plane, g2o optimizer)
- [x] 2D OccupancyGrid (/map, 0.1m çözünürlük)
- [x] 3D Point Cloud Map (/cloud_map)
- [x] OctoMap (/octomap_binary)
- [x] TF zinciri: map → odom → base_link → lidar_link/camera_link
- [x] COM_RC_IN_MODE=4 ile SITL arming sorunu çözüldü

### Faz 5 — Otonom Keşif ve Navigasyon ✅
- [x] Nav2 planner_server + NavfnPlanner (A\*)
- [x] Global costmap (StaticLayer + InflationLayer, 0.1m, RTAB-Map /map)
- [x] map_cleaner.py — Harita gürültü filtresi
- [x] frontier_explorer.py — Otonom sınır (frontier) tabanlı keşif algoritması
- [x] drone_navigator.py — Nav2 ComputePathToPose + PX4 waypoint takibi
- [x] ENU↔NED koordinat dönüşümü (TF map→odom + swap)
- [x] /drone/planned_path görselleştirmesi
- [x] start_all.sh explore modu

### Faz 6 — Kapı Numarası Tanıma (OCR) ⬜
- [ ] OCR modeli seçimi ve kurulumu (EasyOCR / PaddleOCR)
- [ ] Kamera görüntüsünde kapı numarası tespiti
- [ ] Tespit edilen numarayı haritada konumlandırma
- [ ] Alfanumerik tanıma doğrulaması

### Faz 7 — Sistem Entegrasyonu ⬜
- [ ] Tam otonom senaryo: kalkış → harita çıkar → kapıları bul → iniş
- [ ] Davranış ağacı (behavior tree) ile görev yönetimi
- [ ] Hata durumları yönetimi

### Faz 8 — Gerçek Donanım (Raspberry Pi 5 + Pixhawk 6C) ⬜
- [ ] Pi 5 Ubuntu 24.04 + ROS 2 Jazzy + Hailo 26T sürücü
- [ ] PX4 ↔ Pi 5 seri bağlantı (XRCE-DDS)
- [ ] Gerçek LiDAR + kamera entegrasyonu
- [ ] OCR modelini Hailo NPU'da çalıştırma
- [ ] Uçuş testleri

## Lisans

MIT
