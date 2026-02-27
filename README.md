# Kapalı Alan Otonom Drone Projesi

> 2 katlı kapalı bir binada 3D LiDAR ile harita çıkartan, otonom hareket eden ve kapı numaralarını tanıyan drone sistemi.

## Genel Bakış

Bu proje, kapalı bir alanda (2 katlı bina) çalışacak bir otonom drone sistemi geliştirmeyi amaçlamaktadır. Sistem şu temel bileşenlerden oluşur:

| Bileşen | Açıklama |
|---------|----------|
| **3D LiDAR SLAM** | Binanın 3 boyutlu haritasını çıkartma |
| **Otonom Navigasyon** | Harita üzerinde otonom uçuş ve path planning |
| **Kapı Tanıma (OCR)** | Kamera ile kapı numaralarını okuma ve haritada işaretleme |
| **PX4 + Pixhawk 6C** | Uçuş kontrolü (simülasyonda SITL) |

## Sistem Mimarisi

```
┌─────────────────────────────────────────────────────────────┐
│                     Docker Container                        │
│                                                             │
│  ┌──────────┐    ┌──────────────┐    ┌──────────────────┐   │
│  │ PX4 SITL │◄──►│  Micro XRCE  │◄──►│    ROS 2 Jazzy   │   │
│  │ (gz_x500)│    │  DDS Agent   │    │                  │   │
│  └────┬─────┘    └──────────────┘    │  ┌────────────┐  │   │
│       │                              │  │px4_offboard│  │   │
│       ▼                              │  │  (kontrol) │  │   │
│  ┌──────────┐    ┌──────────────┐    │  └────────────┘  │   │
│  │ Gazebo   │◄──►│  ros_gz_     │◄──►│  ┌────────────┐  │   │
│  │ Harmonic │    │  bridge      │    │  │    SLAM    │  │   │
│  │          │    └──────────────┘    │  │(planlanan) │  │   │
│  │ ┌──────┐ │     Sensör Topics:     │  └────────────┘  │   │
│  │ │Build.│ │     /drone/lidar/pts   │  ┌────────────┐  │   │
│  │ │ .obj │ │     /drone/camera      │  │    Nav2    │  │   │
│  │ └──────┘ │                        │  │(planlanan) │  │   │
│  │ ┌──────┐ │     PX4 Topics:        │  └────────────┘  │   │
│  │ │x500_ │ │     /fmu/in/*          │  ┌────────────┐  │   │
│  │ │lidar │ │     /fmu/out/*         │  │    OCR     │  │   │
│  │ └──────┘ │                        │  │(planlanan) │  │   │
│  └──────────┘                        │  └────────────┘  │   │
│                                      └──────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## Donanım (Gerçek Sistem)

| Donanım | Görev |
|---------|-------|
| Raspberry Pi 5 | Ana bilgisayar (companion computer) |
| Hailo 26T AI HAT | Yapay zeka çıkarımı (OCR, nesne tespiti) |
| Pixhawk 6C | Uçuş kontrol kartı (flight controller) |
| 3D LiDAR | Ortam haritalaması |
| Kamera | Kapı numarası tanıma |

## Yazılım Ortamı

| Yazılım | Versiyon / Açıklama |
|---------|---------------------|
| Host OS | Windows + WSL2 |
| Container | Docker (Ubuntu 24.04 Noble) |
| ROS 2 | Jazzy Jalisco |
| Simülatör | Gazebo Harmonic (gz-sim) |
| Uçuş Kontrolü | PX4 Autopilot SITL |
| PX4 ↔ ROS 2 | Micro XRCE-DDS Agent |
| SLAM | RTAB-Map / LIO-SAM *(planlanıyor)* |
| Navigasyon | Nav2 *(planlanıyor)* |
| OCR | EasyOCR / Tesseract *(planlanıyor)* |

## Proje Yapısı

```
drone_project/
├── Dockerfile                              # PX4 + Gazebo + ROS 2 Jazzy
├── README.md
│
├── models/
│   ├── test/                               # Bina modeli (takım arkadaşından)
│   │   ├── model.config
│   │   ├── model.sdf
│   │   └── meshes/
│   │       └── test.obj                    # Blender'dan export edilen bina
│   │
│   ├── x500_lidar/                         # PX4 x500 + sensörler
│   │   ├── model.config
│   │   └── model.sdf                      # x500 include + 3D LiDAR + Kamera
│   │
│   └── simple_drone/                       # Geçici test modeli (PX4'süz)
│       ├── model.config
│       └── model.sdf
│
├── worlds/
│   └── test_building.sdf                  # Gazebo world dosyası
│
├── src/
│   ├── px4_msgs/                           # PX4 ROS 2 mesaj tanımları (git clone)
│   │
│   ├── px4_offboard/                       # Offboard kontrol paketi
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── px4_offboard/
│   │       ├── offboard_control.py         # Otomatik offboard uçuş
│   │       └── drone_teleop.py             # Klavye ile drone kontrolü
│   │
│   └── drone_sim_bringup/                  # Launch & config paketi
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── config/
│       │   └── drone_sim.rviz
│       └── launch/
│           ├── gazebo.launch.py            # Sadece Gazebo
│           ├── bridge.launch.py            # Gazebo ↔ ROS 2 köprüsü
│           └── sim_bringup.launch.py       # Tüm sistemi başlat
│
└── scripts/
    ├── build_docker.sh                     # Docker build
    ├── build_workspace.sh                  # colcon build (container içi)
    ├── run_px4_sitl.sh                     # PX4 SITL + Gazebo başlat
    ├── run_dds_agent.sh                    # Micro XRCE-DDS Agent
    ├── run_ros_bridge.sh                   # Sensör bridge
    ├── run_gazebo.sh                       # Sadece Gazebo (PX4'süz)
    ├── run_sim.sh                          # Tam simülasyon (PX4'süz)
    └── run_full_sim.sh                     # Başlatma rehberi
```

## Kurulum

### 1. Docker İmajını Build Et (WSL terminalinde)

```bash
cd ~/drone_project
docker build -t drone_jazzy_env .
```

> **Not:** İlk build PX4 derlemeyi içerdiği için ~30-60 dk sürebilir.

### 2. Container'ı Başlat

```bash
# İlk çalıştırma
docker run -it \
    --name drone_sim \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/drone_project:/root/drone_project \
    drone_jazzy_env

# Sonraki kullanımlarda
docker start drone_sim
docker exec -it drone_sim bash
```

### 3. ROS 2 Workspace Build (Container İçi)

```bash
bash /root/drone_project/scripts/build_workspace.sh
```

## Çalıştırma

### Yöntem A: PX4 SITL ile Tam Simülasyon (Ana Kullanım)

**4 ayrı terminal** açın (`docker exec -it drone_sim bash`):

```bash
# Terminal 1 — PX4 SITL + Gazebo
bash /root/drone_project/scripts/run_px4_sitl.sh

# Terminal 2 — DDS Agent (PX4 ↔ ROS 2 köprüsü)
bash /root/drone_project/scripts/run_dds_agent.sh

# Terminal 3 — Sensör Bridge (Gazebo ↔ ROS 2)
bash /root/drone_project/scripts/run_ros_bridge.sh

# Terminal 4 — Drone Kontrol
# Otomatik uçuş:
ros2 run px4_offboard offboard_control

# VEYA klavye ile kontrol:
ros2 run px4_offboard drone_teleop
```

### Yöntem B: Sadece Gazebo (PX4'süz Hızlı Test)

```bash
bash /root/drone_project/scripts/run_gazebo.sh
```

### Başlatma Rehberini Göster

```bash
bash /root/drone_project/scripts/run_full_sim.sh
```

## Drone Teleop Kontrolleri

```
╔═══════════════════════════════════╗
║  w/s : İleri / Geri     [0.5m]    ║
║  a/d : Sol / Sağ        [0.5m]    ║
║  q/e : Yukarı / Aşağı   [0.3m]    ║
║  t   : Takeoff (kalkış)           ║
║  l   : Land (iniş)                ║
║  SPC : Hover (yerinde dur)        ║
║  ESC : Çıkış                      ║
╚═══════════════════════════════════╝
```

## Sensörler

### x500_lidar Drone Modeli

PX4 x500 quadrotor'una eklenen sensörler:

| Sensör | Özellikler | ROS 2 Topic |
|--------|-----------|-------------|
| **3D LiDAR** | 360°, 16 kanal, 25m menzil, 10 Hz | `/drone/lidar/points` |
| **Ön Kamera** | 640x480, 80° FOV, 15 Hz | `/drone/camera/image_raw` |
| **IMU** | PX4 dahili | `/fmu/out/vehicle_imu` |

### PX4 Topic'leri

| Topic | Yön | Açıklama |
|-------|-----|----------|
| `/fmu/out/vehicle_local_position` | PX4→ROS | Drone pozisyonu (NED) |
| `/fmu/out/vehicle_status` | PX4→ROS | Arm/disarm, mod durumu |
| `/fmu/out/vehicle_odometry` | PX4→ROS | Odometry verisi |
| `/fmu/in/offboard_control_mode` | ROS→PX4 | Offboard mod parametreleri |
| `/fmu/in/trajectory_setpoint` | ROS→PX4 | Hedef pozisyon/hız |
| `/fmu/in/vehicle_command` | ROS→PX4 | Arm, mod değişikliği, iniş |

## Bina Modeli Hakkında

- `test.obj` Blender 5.0.1'den export edilmiştir
- **`testroom2.mtl` dosyası eksik** — takım arkadaşından istenmeli
  - MTL olmadan bina gri renkte görünür (geometri düzgün yüklenir)
  - Blender'dan export ederken MTL dahil edilmeli, veya
  - **DAE (COLLADA)** formatında export önerilir (Gazebo uyumluluğu daha iyi)

## Kontrol Etme ve Debug

```bash
# Aktif topic'leri listele
ros2 topic list

# LiDAR point cloud kontrol
ros2 topic echo /drone/lidar/points --once

# Kamera frekansı
ros2 topic hz /drone/camera/image_raw

# PX4 durum kontrolü
ros2 topic echo /fmu/out/vehicle_status --once

# Drone pozisyonu
ros2 topic echo /fmu/out/vehicle_local_position --once

# TF tree görselleştirme
ros2 run tf2_tools view_frames
```

## Yol Haritası

- [x] Simülasyon ortamı kurulumu (Docker + ROS 2 + Gazebo)
- [x] Bina modelini Gazebo'ya entegre etme
- [x] Geçici drone modeli (sensörlerle)
- [x] ROS-Gazebo bridge & launch dosyaları
- [x] PX4 SITL entegrasyonu (Dockerfile + scripts)
- [x] PX4 x500 + sensörlü özel drone modeli (x500_lidar)
- [x] Offboard kontrol node'u (otomatik + teleop)
- [ ] 3D LiDAR SLAM (RTAB-Map veya LIO-SAM)
- [ ] Nav2 ile 3D otonom navigasyon
- [ ] Kamera + OCR ile kapı numarası tanıma
- [ ] Gerçek drone modeli entegrasyonu
- [ ] Raspberry Pi 5 + Hailo 26T deployment

## Ekip Görev Dağılımı

| Üye | Görev |
|-----|-------|
| Furkan | Otonomi algoritması, kapı numarası tanıma, sistem entegrasyonu |
| Takım Arkadaşı 1 | Bina 3D modelleme (Blender → OBJ/DAE) |
| Takım Arkadaşı 2 | Drone fiziksel tasarım ve modelleme |

## Lisans

MIT
