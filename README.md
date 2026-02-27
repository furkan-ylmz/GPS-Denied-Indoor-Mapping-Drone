# Kapalı Alan Otonom Drone Projesi

> 2 katlı kapalı bir binada 3D LiDAR ile harita çıkartan, otonom hareket eden ve kapı numaralarını tanıyan drone sistemi.

## Genel Bakış

Bu proje, kapalı bir alanda (2 katlı bina) çalışacak bir otonom drone sistemi geliştirmeyi amaçlamaktadır:

| Bileşen | Açıklama |
|---------|----------|
| **3D LiDAR SLAM** | Binanın 3 boyutlu haritasını çıkartma |
| **Otonom Navigasyon** | Harita üzerinde otonom uçuş ve path planning |
| **Kapı Tanıma (OCR)** | Kamera ile kapı numaralarını okuma ve haritada işaretleme |
| **PX4 + Pixhawk 6C** | Uçuş kontrolü (simülasyonda SITL) |

## Sistem Mimarisi

```
WSL2 (Ubuntu 24.04)
─────────────────────────────────────────────────────
  PX4 SITL ◄──► Micro XRCE-DDS Agent ◄──► ROS 2 Jazzy
  (gz_x500)           (UDP 8888)            │
      │                                     ├─ px4_offboard (kontrol)
      ▼                                     ├─ SLAM (planlanan)
  Gazebo Harmonic ◄──► ros_gz_bridge ◄─────►├─ Nav2 (planlanan)
      │                                     └─ OCR (planlanan)
   ┌──┴──┐
   │Bina │    Sensör Topics:         PX4 Topics:
   │.obj │    /drone/lidar/points    /fmu/in/*
   └─────┘    /drone/camera          /fmu/out/*
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

| Yazılım | Versiyon |
|---------|----------|
| Host OS | Windows + WSL2 (Ubuntu 24.04 Noble) |
| ROS 2 | Jazzy Jalisco |
| Simülatör | Gazebo Harmonic (gz-sim) |
| Uçuş Kontrolü | PX4 Autopilot SITL |
| PX4 ↔ ROS 2 | Micro XRCE-DDS Agent |
| SLAM | RTAB-Map *(planlanan)* |
| Navigasyon | Nav2 *(planlanan)* |
| OCR | EasyOCR / PaddleOCR *(planlanan)* |

## Proje Yapısı

```
drone_project/
├── README.md
├── .gitignore
│
├── models/
│   └── test/                               # Bina modeli
│       ├── model.config
│       ├── model.sdf
│       └── meshes/
│           └── test.obj                    # Blender'dan export
│
├── worlds/
│   └── test_building.sdf                  # Gazebo world (fizik + sensör plugin'leri)
│
├── src/
│   ├── px4_offboard/                       # Offboard kontrol paketi (ament_python)
│   │   ├── package.xml
│   │   ├── setup.py / setup.cfg
│   │   └── px4_offboard/
│   │       ├── offboard_control.py         # Otomatik offboard uçuş
│   │       └── drone_teleop.py             # Klavye ile drone kontrolü
│   │
│   └── drone_sim_bringup/                  # Launch & config paketi (ament_cmake)
│       ├── package.xml / CMakeLists.txt
│       ├── config/drone_sim.rviz
│       └── launch/
│           ├── gazebo.launch.py            # Sadece Gazebo
│           ├── bridge.launch.py            # Gazebo ↔ ROS 2 köprüsü
│           └── sim_bringup.launch.py       # Tüm sistemi başlat
│
└── scripts/
    ├── build_workspace.sh                  # colcon build (px4_msgs auto-clone)
    ├── run_px4_sitl.sh                     # PX4 SITL + Gazebo başlat
    └── run_ros_bridge.sh                   # Sensör bridge
```

> **Not:** `src/px4_msgs/` build sırasında otomatik clone edilir, repoda tutulmaz.

## Ön Gereksinimler

Aşağıdakiler WSL2 Ubuntu 24.04 üzerinde kurulu olmalıdır:

| Bileşen | Kurulum | Konum |
|---------|---------|-------|
| ROS 2 Jazzy | `sudo apt install ros-jazzy-desktop` | `/opt/ros/jazzy` |
| Gazebo + ROS bridge | `sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge` | sistem |
| PX4 Autopilot | `git clone --recursive https://github.com/PX4/PX4-Autopilot` + `make px4_sitl_default` | `~/PX4-Autopilot` |
| Micro XRCE-DDS Agent | [Build from source](https://micro-xrce-dds.docs.eprosima.com/) | `~/Micro-XRCE-DDS-Agent` |

`~/.bashrc` ayarları:
```bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/drone_project/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/PX4-Autopilot/build/px4_sitl_default/build_gz_plugins"
export PX4_ROOT="$HOME/PX4-Autopilot"
```

## Kurulum ve Build

```bash
cd ~/drone_project
bash scripts/build_workspace.sh
```

## Çalıştırma

**3 ayrı terminal** açın:

```bash
# Terminal 1 — PX4 SITL + Gazebo
bash ~/drone_project/scripts/run_px4_sitl.sh          # custom world
bash ~/drone_project/scripts/run_px4_sitl.sh default   # default world

# Terminal 2 — Micro XRCE-DDS Agent (PX4 ↔ ROS 2 köprüsü)
MicroXRCEAgent udp4 -p 8888

# Terminal 3 — Drone Kontrol
source ~/drone_project/install/setup.bash
ros2 run px4_offboard offboard_control   # otomatik uçuş
ros2 run px4_offboard drone_teleop       # klavye kontrolü
```

### PX4 Konsolundan Hızlı Test

PX4 shell (pxh>) açıldıktan sonra:
```
pxh> commander takeoff     # kalkış
pxh> commander land        # iniş
```

## PX4 Topic'leri

| Topic | Yön | Açıklama |
|-------|-----|----------|
| `/fmu/out/vehicle_local_position` | PX4→ROS | Drone pozisyonu (NED) |
| `/fmu/out/vehicle_status` | PX4→ROS | Arm/disarm, mod durumu |
| `/fmu/out/vehicle_odometry` | PX4→ROS | Odometry verisi |
| `/fmu/in/offboard_control_mode` | ROS→PX4 | Offboard mod parametreleri |
| `/fmu/in/trajectory_setpoint` | ROS→PX4 | Hedef pozisyon/hız |
| `/fmu/in/vehicle_command` | ROS→PX4 | Arm, mod değişikliği, iniş |

## Debug

```bash
ros2 topic list                                        # tüm topic'ler
ros2 topic echo /fmu/out/vehicle_status --once         # PX4 durumu
ros2 topic echo /fmu/out/vehicle_local_position --once # pozisyon
ros2 topic hz /fmu/out/sensor_combined                 # sensör frekansı
```

## Bina Modeli

- `test.obj` — Blender 5.0.1 export
- **`testroom2.mtl` dosyası eksik** — MTL olmadan bina gri renkte görünür
- Önerilen: DAE (COLLADA) formatında export (Gazebo uyumluluğu daha iyi)

---

## Yol Haritası

### Faz 1 — Temel Simülasyon Ortamı ✅
- [x] WSL2 + ROS 2 Jazzy + Gazebo Harmonic kurulumu
- [x] PX4 SITL build ve entegrasyonu
- [x] Micro XRCE-DDS Agent (PX4 ↔ ROS 2 iletişimi)
- [x] Bina modelini Gazebo world'e entegre etme
- [x] `commander takeoff/land` ile temel uçuş doğrulaması
- [x] Workspace yapısı, launch dosyaları, yardımcı script'ler

### Faz 2 — Offboard Drone Kontrolü 🔧 *(aktif)*
- [ ] **ROS 2 offboard kontrol düzeltmesi** — teleop/offboard setpoint'leri PX4'e ulaşmıyor
  - QoS profili uyumu kontrol edilmeli (BEST_EFFORT vs RELIABLE)
  - `px4_msgs` versiyonu ile PX4 firmware uyumu doğrulanmalı
  - `OffboardControlMode` + `TrajectorySetpoint` mesaj akışı debug
- [ ] Teleop ile güvenilir kalkış / iniş / yön kontrolü
- [ ] Offboard state machine testi (IDLE→ARM→TAKEOFF→HOVER→NAVIGATE→LAND)

### Faz 3 — Sensör Entegrasyonu
- [ ] **Özel drone modeli** — x500'e 3D LiDAR + kamera ekleme (SDF model)
- [ ] Gazebo → ROS 2 sensör bridge (lidar + kamera topic'leri)
- [ ] LiDAR point cloud RViz'de görselleştirme
- [ ] Kamera görüntüsü doğrulama
- [ ] Bina modeline kapı numarası texture'ları ekleme

### Faz 4 — 3D SLAM (Haritalama)
- [ ] RTAB-Map kurulumu ve konfigürasyonu
- [ ] 3D LiDAR → RTAB-Map entegrasyonu
- [ ] Odometry kaynağı (PX4 VIO / LiDAR odometry)
- [ ] Gerçek zamanlı 3D harita oluşturma
- [ ] Harita kaydetme ve yükleme
- [ ] Bina içi multi-floor harita yönetimi

### Faz 5 — Otonom Navigasyon
- [ ] Nav2 kurulumu ve 3D costmap konfigürasyonu
- [ ] Global planner — bina içi yol planlama
- [ ] Local planner — engel kaçınma
- [ ] Waypoint takibi (oda oda gezme)
- [ ] Kat geçişi stratejisi (merdiven / asansör algılama)

### Faz 6 — Kapı Numarası Tanıma (OCR)
- [ ] OCR modeli seçimi ve kurulumu (EasyOCR / PaddleOCR)
- [ ] Kamera görüntüsünde kapı numarası tespiti
- [ ] Tespit edilen numarayı haritada konumlandırma
- [ ] "egc16" gibi karışık alfanumerik tanıma doğrulaması
- [ ] ROS 2 service/action olarak OCR entegrasyonu

### Faz 7 — Sistem Entegrasyonu
- [ ] Tam otonom senaryo: kalkış → harita çıkar → kapıları bul → iniş
- [ ] Davranış ağacı (behavior tree) ile görev yönetimi
- [ ] Hata durumları yönetimi (düşük batarya, kayıp pozisyon, vb.)
- [ ] Performans optimizasyonu

### Faz 8 — Gerçek Donanım (Raspberry Pi 5 + Pixhawk 6C)
- [ ] Pi 5 üzerine Ubuntu 24.04 + ROS 2 Jazzy kurulumu
- [ ] Hailo 26T AI HAT sürücü ve SDK kurulumu
- [ ] PX4 ↔ Pi 5 seri bağlantı (MAVROS veya XRCE-DDS)
- [ ] Gerçek LiDAR ve kamera entegrasyonu
- [ ] OCR modelini Hailo NPU'da çalıştırma
- [ ] Uçuş testleri (kapalı alanda)

## Ekip Görev Dağılımı

| Üye | Görev |
|-----|-------|
| Furkan | Otonomi algoritması, kapı numarası tanıma, sistem entegrasyonu |
| Takım Arkadaşı 1 | Bina 3D modelleme (Blender → OBJ/DAE) |
| Takım Arkadaşı 2 | Drone fiziksel tasarım ve modelleme |

## Lisans

MIT
