<div align="center">

[English](#english) | [Türkçe](#türkçe)

</div>

---

<a name="english"></a>
# Autonomous Indoor Mapping & Exploration System

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-SITL%20v1.14+-red.svg)](https://px4.io/)
[![SLAM](https://img.shields.io/badge/SLAM-RTAB--Map%203D%20ICP-brightgreen.svg)](https://introlab.github.io/rtabmap/)
[![Navigation](https://img.shields.io/badge/Navigation-Nav2%20MPPI%20Omni-yellow.svg)](https://navigation.ros.org/)
[![Platform](https://img.shields.io/badge/OS-Ubuntu%2024.04%20LTS-purple.svg)](https://ubuntu.com/)

An advanced, fully autonomous indoor quadrotor system combining **PX4 SITL**, **3D LiDAR**, and **RGB Camera** simulation in **Gazebo Harmonic**. The system performs high-fidelity real-time 3D SLAM via RTAB-Map point-to-plane ICP, holonomic obstacle avoidance with Nav2 MPPI, autonomous frontier exploration with BFS clustering and anti-stuck heuristics, and seamless NED ↔ ENU coordinate bridging for GPS-denied environments.

---

## System Architecture & Core Components

The architecture consists of four primary operational modules:

1. **Spatial Perception & 3D SLAM (RTAB-Map ICP Engine):**
   - Real-time 3D LiDAR mapping up to 25 meters using Point-to-Plane ICP.
   - Graph-based loop closure and drift correction to keep the map accurate.
   - 2D obstacle grid generation by filtering out floor and ceiling reflections.

2. **Holonomic Path Planning & Control:**
   - **Global Planner:** A* path planning over 2D occupancy grids with unknown space traversal support.
   - **Local Controller:** Nav2 MPPI controller generating 500 omnidirectional rollouts at 20 Hz.
   - **Trajectory Critics:** Evaluates collision safety, obstacle clearance, path tracking, and goal heading alignment.

3. **Autonomous Frontier Exploration:**
   - Real-time frontier cell extraction at the boundary of explored and unknown map regions.
   - BFS clustering to group frontier cells and filter out sensor noise.
   - **3-Tier Distance Selection Strategy:**
     - *Local Zone (< 3.5 m):* Clears nearby room pockets first to prevent oscillation.
     - *Mid Zone (3.5 – 7.5 m):* Balances cluster size against distance for optimal targets.
     - *Far Zone (> 7.5 m):* Focuses on large unexplored areas across the building.
   - **Dual-Stage Anti-Stuck Supervisor:**
     - *Proximity Timeout:* Blacklists unreachable goals if progress stalls for 8 seconds.
     - *Physical Trap Detection:* Re-plans immediately if blocked for over 12 seconds.
   - **Panoramic 200° Yaw Sweep:** Rotates 200° at waypoints to expand sensor coverage.
   - **Return-to-Home & Landing:** Flies back to takeoff position and lands safely when complete.

4. **Offboard Flight Bridge & Kinematics:**
   - **Coordinate Frame Conversion:** Real-time translation between PX4 and ROS frames.
   - **Flight State Machine:** Manages autonomous arming, takeoff, flight, and landing.
   - **Altitude Lock & Velocity Bridge:** Holds 1.5 m altitude and translates velocity commands.
   - **Smooth Heading Alignment:** Aligns drone heading with flight path for sensor coverage.

---

## Technical Stack

- **Operating System:** Ubuntu 24.04 LTS
- **Middleware:** ROS 2 Jazzy Jalisco
- **Physics Simulator:** Gazebo Harmonic with ODE engine at 200 Hz
- **Flight Controller:** PX4 Autopilot SITL v1.14+
- **Communications Bridge:** Micro XRCE-DDS Agent and ros_gz_bridge
- **SLAM & Mapping:** RTAB-Map with Point-to-Plane ICP and g2o optimizer
- **Navigation Stack:** Nav2 with Navfn A* planner and MPPI controller
- **Programming Languages:** Python 3.12, C++17, CMake
- **Visualization:** RViz2

---

## System Architecture & Data Flow

```text
                                  +---------------------------------------+
                                  |     Gazebo Harmonic (gz-sim)          |
                                  |  - test_building.sdf (ODE Physics)    |
                                  |  - Holybro x500 Quadrotor             |
                                  |  - GPU 3D LiDAR (180x45, 25m)         |
                                  |  - Front RGB Camera (1280x720)        |
                                  +-------------------+-------------------+
                                                      |
                         +----------------------------+----------------------------+
                         | ros_gz_bridge                                           | Micro XRCE-DDS Agent (UDP 8888)
                         v                                                         v
              +--------------------+                                    +--------------------+
              |   /lidar/points    |                                    |    /fmu/out/*      |
              |   /camera          |                                    | (Odometry, Status) |
              +---------+----------+                                    +---------+----------+
                        |                                                         |
         +--------------+--------------+                                          v
         |                             |                               +----------------------+
         v                             v                               |  px4_tf_broadcaster  |
+------------------+         +--------------------+                    |     (NED -> ENU)     |
|     RTAB-Map     |         |   local_costmap    |                    +----------+-----------+
|  Point-to-Plane  |         | (ObstacleLayer 3D) |                               | odom -> base_link TF
+--------+---------+         +---------+----------+                               v
         | /map                        |                            +----------------------------+
         v                             v                            |     Nav2 Architecture      |
+------------------+         +--------------------+                 | - Planner: Navfn (A*)      |
| frontier_explore |         |   global_costmap   |                 | - Controller: MPPI (Omni)  |
| - BFS Clustering |         | (Static+Inflation) |                 | - BT Navigator & Recovery  |
| - Anti-Stuck     +-------->+--------------------+                 +--------------+-------------+
| - Yaw Sweeping   |   NavigateToPose Action                                       | /cmd_vel (Twist)
+------------------+                                                               v
                                                                    +----------------------------+
                                                                    |      drone_navigator       |
                                                                    | - State Machine            |
                                                                    | - Altitude Hold (1.5m)     |
                                                                    | - Hysteresis Yaw Control   |
                                                                    | - Body FLU -> World NED    |
                                                                    +--------------+-------------+
                                                                                   | TrajectorySetpoint
                                                                                   v
                                                                    +----------------------------+
                                                                    |          PX4 SITL          |
                                                                    |    (Offboard Flight)       |
                                                                    +----------------------------+
```


---

## Project Structure

```
GPS-Denied-Indoor-Mapping-Drone/
├── models/
│   ├── test/                                # Test building 3D collision & visual meshes
│   │   ├── meshes/
│   │   │   ├── testroom.obj                 # Wavefront 3D interior architecture model
│   │   │   └── testroom.mtl                 # Surface material properties
│   │   ├── model.config                     # Gazebo model descriptor
│   │   └── model.sdf                        # SDF model definition
│   └── x500_lidar/                          # Holybro x500 + 3D LiDAR + RGB Camera
│       ├── model.config                     # Sensorized drone descriptor
│       └── model.sdf                        # GPU LiDAR and pinhole camera SDF
├── script/
│   ├── install_dependencies.sh              # Automated OS check, apt, rosdep & Micro-XRCE installer
│   ├── start_teleop.sh                      # Shell orchestrator: Manual keyboard teleop mode
│   ├── start_autonomous.sh                  # Shell orchestrator: Nav2 goal navigation mode
│   └── start_explore.sh                     # Shell orchestrator: Full autonomous frontier exploration
├── src/
│   ├── drone_sim_bringup/                   # CMake Bringup Package
│   │   ├── CMakeLists.txt                   # Build rules & directory installations
│   │   ├── package.xml                      # Package dependencies
│   │   ├── config/
│   │   │   ├── nav2_params.yaml             # MPPI controller, A* planner & costmap configurations
│   │   │   └── rtabmap_params.yaml          # 3D LiDAR ICP & g2o optimization parameters
│   │   ├── launch/
│   │   │   ├── bringup.launch.py            # Master launch orchestrator (Bridge, SLAM, Nav2, Nodes)
│   │   │   └── nav2.launch.py               # Nav2 modular lifecycle stack launcher
│   │   └── rviz/
│   │       └── slam_view.rviz               # Multi-layer RViz2 visualization setup
│   └── px4_offboard/                        # Python Offboard & Autonomy Package
│       ├── setup.py / setup.cfg             # Python build & entry point definitions
│       ├── package.xml                      # ROS 2 Python package dependencies
│       └── px4_offboard/
│           ├── __init__.py
│           ├── autonomous.py                # Nav2 cmd_vel -> PX4 TrajectorySetpoint flight bridge
│           ├── explore.py                   # Frontier detector, BFS clustering & supervisor node
│           ├── teleop.py                    # Terminal non-blocking keyboard flight controller
│           └── tf_broadcaster.py            # PX4 VehicleOdometry -> ROS TF (NED to ENU)
├── worlds/
│   └── test_building.sdf                    # Gazebo Harmonic world (Physics, Lighting, Building)
├── README.md                                # Bilingual system documentation
└── .gitignore                               # Colcon, build & system exclusions
```

---

## Prerequisites & Installation

### 1. Platform & OS Compatibility

> [!IMPORTANT]
> - **Target OS:** **Ubuntu 24.04 LTS (Noble Numbat)** with **ROS 2 Jazzy Jalisco**.
> - **Windows Users:** This system **cannot** run natively on Windows due to Linux-dependent POSIX terminal controls, Gazebo Harmonic Linux graphics bridges, and PX4 SITL toolchains. Windows users must use **WSL 2 (Ubuntu 24.04 LTS)** with WSLg enabled, or a direct **Dual-Boot Ubuntu 24.04** partition.
> - **Other Ubuntu Versions (22.04, 20.04, 26.04+):** Not supported out-of-the-box. ROS 2 Jazzy Jalisco and its Gazebo Harmonic / Nav2 MPPI package stack are tightly coupled to Ubuntu 24.04 LTS. Other versions use different ROS 2 distributions (e.g., Humble on 22.04) and incompatible API dependencies.

### 2. Automated Dependency Installation (Recommended)

Run the included dependency installation script to automatically verify your OS, install ROS 2 Jazzy desktop and simulation packages, resolve `rosdep` keys, and build Micro XRCE-DDS Agent:

```bash
chmod +x ./script/install_dependencies.sh
./script/install_dependencies.sh
```

---

### 3. Manual Installation Steps

If you prefer to install dependencies manually:

#### A. Install ROS 2 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-tf2-ros \
  ros-jazzy-visualization-msgs
```

#### B. Build PX4 SITL & Micro-XRCE Agent

```bash
# Clone and build PX4 Autopilot
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500

# Install Micro XRCE-DDS Agent
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

#### C. Build Workspace

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
colcon build --symlink-install
source install/setup.bash
```

---

## Execution & Quick Start

### Mode 1: Manual Teleoperation

- **Active Nodes:** `drone_teleop`, `rtabmap`, `rviz2`, `px4_tf_broadcaster`
- **Description:** Manual keyboard flight for testing and mapping.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_teleop.sh
```

**Key Controls:**
- `t`: Arm drone, switch to Offboard mode, and take off to 1.0 m
- `w` / `s`: Move Forward / Backward by 0.5 m
- `a` / `d`: Move Left / Right by 0.5 m
- `q` / `e`: Move Up / Down by 0.15 m
- `z` / `c`: Rotate Left / Right
- `Space`: Immediate Hover to hold position
- `l`: Land drone

### Mode 2: Autonomous Goal Navigation

- **Active Nodes:** Nav2, `drone_navigator`, `rtabmap`, `rviz2`, `px4_tf_broadcaster`
- **Description:** Point-to-point A* navigation with MPPI obstacle avoidance.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_autonomous.sh
```

1. Press **PLAY (▶)** in the Gazebo GUI.
2. The drone will automatically arm, take off, and hover at **1.5 m**.
3. In RViz2, click the **2D Goal Pose** tool and select a destination on the map.
4. The drone will compute the A* path and track it using the MPPI controller while dodging obstacles.

### Mode 3: Fully Autonomous Frontier Exploration

- **Active Nodes:** `frontier_explorer`, Nav2, `drone_navigator`, `rtabmap`, `rviz2`
- **Description:** Autonomous room exploration, mapping, and return-to-home.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_explore.sh
```

1. Press **PLAY (▶)** in Gazebo.
2. The drone will auto-takeoff and hover.
3. After a 10-second initialization window, the frontier exploration node will automatically discover unmapped frontiers, select optimal clusters, navigate between rooms, perform panoramic yaw sweeps, and return to the home location to land once exploration is complete.

---

## RViz2 Visualization Layers

The included RViz2 profile (`slam_view.rviz`) loads the following telemetry and map streams:

| Display Layer | Topic Name | Description |
|---|---|---|
| **MapCloud** | `/mapData` | Persistent 3D dense point cloud generated by RTAB-Map |
| **PointCloud2** | `/lidar/points` | Raw live 3D LiDAR point cloud |
| **Map** | `/map` | 2D OccupancyGrid map for path planning |
| **Global Path** | `/plan` | A* global trajectory (Green line) |
| **Local Plan** | `/local_plan` | MPPI dynamic rollout trajectory (Blue line) |
| **Global Costmap** | `/global_costmap/costmap` | Static obstacle inflation layer |
| **Local Costmap** | `/local_costmap/costmap` | Live 3D obstacle avoidance costmap |
| **Frontier Clusters** | `/explore/frontiers` | Unexplored boundary markers (Blue cubes) |
| **Active Target** | `/explore/target` | Selected frontier destination centroid (Green sphere) |
| **Coordinate Frames** | `TF` | Transform tree (map -> odom -> base_link -> sensors) |

<br>

---

<a name="türkçe"></a>
# Kapalı Alan Otonom Haritalama ve Keşif Sistemi

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-SITL%20v1.14+-red.svg)](https://px4.io/)
[![SLAM](https://img.shields.io/badge/SLAM-RTAB--Map%203D%20ICP-brightgreen.svg)](https://introlab.github.io/rtabmap/)
[![Navigation](https://img.shields.io/badge/Navigation-Nav2%20MPPI%20Omni-yellow.svg)](https://navigation.ros.org/)
[![Platform](https://img.shields.io/badge/OS-Ubuntu%2024.04%20LTS-purple.svg)](https://ubuntu.com/)

**PX4 SITL**, **3D LiDAR** ve **RGB Kamera** simülasyonunu **Gazebo Harmonic** ortamında birleştiren gelişmiş ve tam otonom bir kapalı alan quadrotor sistemi. Sistem; RTAB-Map Point-to-Plane ICP ile yüksek doğruluklu gerçek zamanlı 3D SLAM, Nav2 MPPI ile holonomik engelden kaçınma, BFS kümeleme ve sıkışma önleme ile otonom sınır keşfi ve GPS'siz ortamlar için kesintisiz NED ↔ ENU koordinat köprülemesi sağlar.

---

## Sistem Mimarisi ve Temel Bileşenler

Sistem mimarisi dört ana işlem modülünden oluşmaktadır:

1. **Uzaysal Algılama ve 3D SLAM (RTAB-Map ICP Motoru):**
   - Noktadan-Düzleme ICP ile 25 metreye kadar gerçek zamanlı 3D LiDAR haritalama.
   - Harita kaymalarını önleyen döngü kapatma ve graf optimizasyonu.
   - Zemin ve tavanı filtreleyerek temiz bir 2D engel haritası üretimi.

2. **Holonomic Yol Planlama ve Kontrol:**
   - **Global Planlayıcı:** 2D doluluk haritasında bilinmeyen alanlardan da geçebilen Navfn A* rota planlayıcısı.
   - **Yerel Kontrolcü:** Saniyede 20 kez çok yönlü rota tahminleri üreten Nav2 MPPI kontrolcüsü.
   - **Yörünge Kriterleri:** Çarpışma güvenliği, engel mesafesi, rota takibi ve hedefe yönelme.

3. **Otonom Sınır Keşif Motoru:**
   - Haritada keşfedilmiş ve bilinmeyen alanların sınırlarını anlık tespit etme.
   - BFS kümeleme algoritması ile sınır noktalarını gruplama ve sensör gürültülerini filtreleme.
   - **3 Kademeli Keşif Stratejisi:**
     - *Yakın Bölge (< 3.5 m):* Odalar arasında savrulmayı önlemek için cepleri temizler.
     - *Orta Bölge (3.5 – 7.5 m):* Boyut ve mesafe dengesine göre en uygun hedefi seçer.
     - *Uzak Bölge (> 7.5 m):* Yapı içerisindeki geniş ve henüz gidilmemiş alanlara odaklanır.
   - **Çift Kademeli Sıkışma Önleme:**
     - *Yakınlık Zaman Aşımı:* 8 saniye boyunca ilerleme durursa hedefi kara listeye alır.
     - *Fiziksel Engel Algılama:* Dron 12 saniye boyunca sıkışıp kalırsa görevi yeniler.
   - **Panoramik 200° Tarama:** Hedefe varıldığında 200° dönerek sensör görüşünü haritaya işler.
   - **Otonom Eve Dönüş ve İniş:** Keşif tamamlandığında kalkış noktasına dönüp güvenle iner.

4. **Offboard Uçuş Köprüsü ve Kinematik:**
   - **Koordinat Köprüsü:** PX4 havacılık ve ROS robotik eksenleri arasında anlık dönüşüm.
   - **Uçuş Durum Makinesi:** Kalkış, navigasyon, bekleme ve iniş aşamalarını güvenle yönetir.
   - **İrtifa ve Hız Kontrolü:** 1.5 m irtifayı sabit tutarak hız komutlarını PX4'e iletir.
   - **Yumuşak Yönlenme:** Sensör görüşünü artırmak için dronun burnunu uçuş yönüne çevirir.

---

## Teknolojik Altyapı

- **İşletim Sistemi:** Ubuntu 24.04 LTS
- **Robotik Ara Katman:** ROS 2 Jazzy Jalisco
- **Fiziksel Simülatör:** Gazebo Harmonic ve 200 Hz ODE fizik motoru
- **Otopilot:** PX4 Autopilot SITL v1.14+
- **Haberleşme Köprüsü:** Micro XRCE-DDS Agent ve ros_gz_bridge
- **Haritalama:** RTAB-Map, Point-to-Plane ICP ve g2o optimizasyon motoru
- **Navigasyon Paketi:** Nav2, Navfn A* planlayıcı ve MPPI kontrolcüsü
- **Programlama Dilleri:** Python 3.12, C++17, CMake
- **Görselleştirme:** RViz2

---

## Sistem Mimarisi ve Veri Akışı

```text
                                  +---------------------------------------+
                                  |     Gazebo Harmonic (gz-sim)          |
                                  |  - test_building.sdf (ODE Fizik)      |
                                  |  - Holybro x500 Quadrotor             |
                                  |  - GPU 3D LiDAR (180x45, 25m)         |
                                  |  - Ön RGB Kamera (1280x720)           |
                                  +-------------------+-------------------+
                                                      |
                         +----------------------------+----------------------------+
                         | ros_gz_bridge                                           | Micro XRCE-DDS Agent (UDP 8888)
                         v                                                         v
              +--------------------+                                    +--------------------+
              |   /lidar/points    |                                    |    /fmu/out/*      |
              |   /camera          |                                    | (Odometry, Status) |
              +---------+----------+                                    +---------+----------+
                        |                                                         |
         +--------------+--------------+                                          v
         |                             |                               +----------------------+
         v                             v                               |  px4_tf_broadcaster  |
+------------------+         +--------------------+                    |     (NED -> ENU)     |
|     RTAB-Map     |         |   local_costmap    |                    +----------+-----------+
|  Point-to-Plane  |         | (ObstacleLayer 3D) |                               | odom -> base_link TF
+--------+---------+         +---------+----------+                               v
         | /map                        |                            +----------------------------+
         v                             v                            |      Nav2 Mimarisi         |
+------------------+         +--------------------+                 | - Planlayıcı: Navfn (A*)   |
| frontier_explore |         |   global_costmap   |                 | - Kontrolcü: MPPI (Omni)   |
| - BFS Kümeleme   |         | (Statik+Inflation) |                 | - BT Navigator & Recovery  |
| - Sıkışma Takibi +-------->+--------------------+                 +--------------+-------------+
| - Yaw Süpürme    |   NavigateToPose Action                                       | /cmd_vel (Twist)
+------------------+                                                               v
                                                                    +----------------------------+
                                                                    |      drone_navigator       |
                                                                    | - Durum Makinesi           |
                                                                    | - İrtifa Sabitleme (1.5m)  |
                                                                    | - Histerezisli Yaw Kontrol |
                                                                    | - Gövde FLU -> Dünya NED   |
                                                                    +--------------+-------------+
                                                                                   | TrajectorySetpoint
                                                                                   v
                                                                    +----------------------------+
                                                                    |          PX4 SITL          |
                                                                    |     (Offboard Uçuş)        |
                                                                    +----------------------------+
```



---

## Proje Dizin Yapısı

```
GPS-Denied-Indoor-Mapping-Drone/
├── models/
│   ├── test/                                # Test binası 3D çarpışma ve görsel mesh modelleri
│   │   ├── meshes/
│   │   │   ├── testroom.obj                 # Wavefront 3D bina iç mimari modeli
│   │   │   └── testroom.mtl                 # Yüzey malzeme özellikleri
│   │   ├── model.config                     # Gazebo model tanım dosyası
│   │   └── model.sdf                        # SDF model yapılandırması
│   └── x500_lidar/                          # Holybro x500 + 3D LiDAR + RGB Kamera modeli
│       ├── model.config                     # Sensörlü drone modeli tanımlayıcısı
│       └── model.sdf                        # GPU LiDAR ve kamera sensör SDF tanımı
├── script/
│   ├── install_dependencies.sh              # Otomatik OS kontrolü, apt, rosdep ve Micro-XRCE kurulum betiği
│   ├── start_teleop.sh                      # Manuel klavye teleop başlatıcı betiği
│   ├── start_autonomous.sh                  # Nav2 hedefli otonom navigasyon başlatıcı betiği
│   └── start_explore.sh                     # Tam otonom sınır keşfi başlatıcı betiği
├── src/
│   ├── drone_sim_bringup/                   # CMake Başlatıcı Paketi
│   │   ├── CMakeLists.txt                   # Derleme ve dizin kurulum kuralları
│   │   ├── package.xml                      # Paket bağımlılık bildirimleri
│   │   ├── config/
│   │   │   ├── nav2_params.yaml             # MPPI kontrolcü, A* planlayıcı ve costmap ayarları
│   │   │   └── rtabmap_params.yaml          # 3D LiDAR ICP ve g2o optimizasyon ayarları
│   │   ├── launch/
│   │   │   ├── bringup.launch.py            # Ana orkestrasyon launch dosyası
│   │   │   └── nav2.launch.py               # Nav2 modüler yaşam döngüsü launch dosyası
│   │   └── rviz/
│   │       └── slam_view.rviz               # Çok katmanlı RViz2 arayüz konfigürasyonu
│   └── px4_offboard/                        # Python Kontrol ve Otonomi Paketi
│       ├── setup.py / setup.cfg             # Python paket derleme ve entry point tanımları
│       ├── package.xml                      # ROS 2 Python paket bağımlılıkları
│       └── px4_offboard/
│           ├── __init__.py
│           ├── autonomous.py                # Nav2 cmd_vel -> PX4 TrajectorySetpoint köprü düğümü
│           ├── explore.py                   # Sınır tespiti, BFS kümeleme ve keşif denetçisi
│           ├── teleop.py                    # Terminal non-blocking klavye uçuş kontrolcüsü
│           └── tf_broadcaster.py            # PX4 VehicleOdometry -> ROS TF (NED -> ENU)
├── worlds/
│   └── test_building.sdf                    # Gazebo Harmonic dünyası (Fizik, Işıklandırma, Bina)
├── README.md                                # İki dilli kapsamlı sistem dokümantasyonu
└── .gitignore                               # Colcon, derleme ve sistem dışlama kuralları
```

---

## Kurulum ve Ön Gereksinimler

### 1. Platform ve İşletim Sistemi Uyumluluğu

> [!IMPORTANT]
> - **Hedef İşletim Sistemi:** **Ubuntu 24.04 LTS (Noble Numbat)** ve **ROS 2 Jazzy Jalisco**.
> - **Windows Kullanıcıları:** Bu proje; Linux tabanlı POSIX terminal girdi kontrolleri, Gazebo Harmonic Linux köprüleri ve PX4 derleme zinciri nedeniyle doğrudan Windows üzerinde **çalıştırılamaz**. Windows kullanıcıları GUI destekli **WSL 2 (Ubuntu 24.04 LTS)** veya doğrudan **Dual-Boot Ubuntu 24.04** kullanmalıdır.
> - **Diğer Ubuntu Sürümleri (22.04, 20.04, 26.04+):** Doğrudan desteklenmemektedir. ROS 2 Jazzy Jalisco, Gazebo Harmonic ve Nav2 MPPI bağımlılıkları doğrudan Ubuntu 24.04 LTS için derlenmiştir. Diğer sürümler farklı ROS 2 dağıtımları (22.04 için Humble vb.) ve uyumsuz paket sürümleri gerektirir.

### 2. Otomatik Bağımlılık Kurulumu (Önerilen)

Sistem kontrollerini yapmak, ROS 2 Jazzy paketlerini kurmak, `rosdep` anahtarlarını çözmek ve Micro XRCE-DDS Agent'ı derlemek için hazırlanan kurulum betiğini çalıştırın:

```bash
chmod +x ./script/install_dependencies.sh
./script/install_dependencies.sh
```

---

### 3. Manuel Kurulum Adımları

Bağımlılıkları adım adım elle kurmak isterseniz:

#### A. ROS 2 Paketlerinin Kurulumu

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-tf2-ros \
  ros-jazzy-visualization-msgs
```

#### B. PX4 SITL ve Micro-XRCE Agent Derlenmesi

```bash
# PX4 Autopilot kaynak kodunu indirip derleyin
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500

# Micro XRCE-DDS Agent kurulumu
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

#### C. Çalışma Alanının (Workspace) Derlenmesi

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
colcon build --symlink-install
source install/setup.bash
```

---

## Çalıştırma ve Kullanım

### Mod 1: Manuel Klavye Kontrolü (Teleop)

- **Aktif Düğümler:** `drone_teleop`, `rtabmap`, `rviz2`, `px4_tf_broadcaster`
- **Açıklama:** Klavye ile test ve haritalama uçuşu.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_teleop.sh
```

**Klavye Kontrol Tuşları:**
- `t`: Motorları çalıştır, Offboard moda geç ve 1.0 m otomatik kalkış yap
- `w` / `s`: İleri / Geri 0.5 m hareket
- `a` / `d`: Sol / Sağ 0.5 m hareket
- `q` / `e`: Yukarı / Aşağı 0.15 m hareket
- `z` / `c`: Sola / Sağa dönüş
- `Space`: Anlık Hover ile konumunu koru
- `l`: Güvenli iniş prosedürünü başlat

### Mod 2: Otonom Hedef Navigasyonu

- **Aktif Düğümler:** Nav2, `drone_navigator`, `rtabmap`, `rviz2`, `px4_tf_broadcaster`
- **Açıklama:** A* planlama ve MPPI engelden kaçınma ile noktadan noktaya uçuş.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_autonomous.sh
```

1. Gazebo arayüzünde **PLAY (▶)** butonuna basın.
2. Dron otomatik olarak kalkacak ve **1.5 m** irtifada beklemeye geçecektir.
3. RViz2 arayüzünde üst bardan **2D Goal Pose** aracını seçip haritada gitmek istediğiniz noktaya tıklayın.
4. Dron A* rotasını MPPI kontrolcüsü ile takip ederek engellerin etrafından dolaşıp hedefe varacaktır.

### Mod 3: Tam Otonom Sınır Keşfi (Frontier Exploration)

- **Aktif Düğümler:** `frontier_explorer`, Nav2, `drone_navigator`, `rtabmap`, `rviz2`
- **Açıklama:** Otonom sınır keşfi, haritalama ve kalkış noktasına güvenli iniş.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_explore.sh
```

1. Gazebo'da **PLAY (▶)** butonuna basın.
2. Dron otomatik kalkış yapıp 1.5 m'de beklemeye başlar.
3. 10 saniyelik harita ilklendirme süresi dolduktan sonra otonom keşif düğümü haritada keşfedilmemiş sınır noktalarını belirler, en uygun kümeyi seçip odalar arasında navigasyonu başlatır, hedeflerde süpürme dönüşü yapar ve tüm bina haritalandığında kalkış noktasına dönüp iniş yapar.

---

## RViz2 Görselleştirme Katmanları

Varsayılan konfigürasyonda (`slam_view.rviz`) aşağıdaki veri katmanları canlı olarak izlenir:

| Katman | Topic Adı | Açıklama |
|---|---|---|
| **MapCloud** | `/mapData` | RTAB-Map tarafından üretilen kalıcı ve birleştirilmiş 3D yoğun nokta bulutu |
| **PointCloud2** | `/lidar/points` | Canlı ham 3D LiDAR nokta bulutu |
| **Map** | `/map` | Navigasyon ve planlama için kullanılan 2D doluluk haritası |
| **Global Path** | `/plan` | A* küresel yol planlayıcısı tarafından çizilen ana rota (Yeşil) |
| **Local Plan** | `/local_plan` | MPPI kontrolcüsünün hesapladığı anlık yerel kaçış rotası (Mavi) |
| **Global Costmap** | `/global_costmap/costmap` | Statik harita duvarları ve şişirme katmanı |
| **Local Costmap** | `/local_costmap/costmap` | Anlık sensör verisinden beslenen 3D engel kaçınma katmanı |
| **Frontier Kümeleri** | `/explore/frontiers` | Tespit edilen keşfedilmemiş sınır noktaları (Mavi küpler) |
| **Aktif Hedef** | `/explore/target` | Seçilen hedef sınır kümesi ağırlık merkezi (Yeşil küre) |
| **Eksen Takımları** | `TF` | Dönüşüm ağacı (map -> odom -> base_link -> sensörler) |
