<div align="center">

[English](#english) | [Türkçe](#türkçe)

</div>

---

<a name="english"></a>
# Autonomous Indoor Mapping & Exploration Drone System

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
   - 3D LiDAR point cloud processing ($180 \times 45$ beams, 25 m range) with Point-to-Plane Iterative Closest Point (ICP) registration.
   - Downsampled 0.08 m voxel grid filtering with graph-based loop closure and drift correction powered by `g2o`.
   - Real-time 2.5D OccupancyGrid projection filtered between 1.2 m and 1.8 m to suppress ground and ceiling reflections.

2. **Holonomic Path Planning & Control (Nav2 Stack):**
   - **Global Planner:** $A^*$ search over 2D occupancy grids via `NavfnPlanner` with unknown space traversal support.
   - **Local Controller:** Model Predictive Path Integral (`MPPIController`) with holonomic `Omni` motion model generating 500 predictive trajectory rollouts across a 2.0 s (40 steps $\times$ 0.05 s) horizon at 20 Hz.
   - **7 Critic Cost Evaluators:** Collision avoidance (`ConstraintCritic`), obstacle clearance (`CostCritic`), path tracking (`PathAlignCritic`, `PathFollowCritic`), and heading alignment (`PathAngleCritic`, `GoalAngleCritic`).

3. **Autonomous Frontier Exploration (`FrontierExplorer`):**
   - Real-time frontier cell extraction at the boundary of free ($0$) and unknown ($-1$) grid regions.
   - Breadth-First Search (BFS) connected components clustering with minimum cluster size thresholds.
   - **3-Tier Distance Selection Strategy:**
     - *Local Zone ($< 3.5\text{ m}$):* Cleans up local alcoves/pockets (smallest cluster first) to eliminate ping-pong oscillations.
     - *Mid Zone ($3.5 - 7.5\text{ m}$):* Distance-squared normalized scoring ($\text{Score} = \text{Size} / (\text{Dist}^2 + 0.1)$).
     - *Far Zone ($> 7.5\text{ m}$):* Global frontier prioritization.
   - **Dual-Stage Anti-Stuck Supervisor:**
     - *Proximity Stuck:* Cancels and blacklists target if no progress ($> 10\text{ cm}$) is made within 2.5 m of goal for 8 seconds.
     - *Physical Stuck:* Detects mechanical/obstacle traps if net displacement over 12 seconds is under 0.40 m.
   - **Post-Goal 200° Yaw Sweep:** Performs a 200° panoramic in-place sensor sweep upon reaching waypoints to accelerate map expansion.
   - **Autonomous Return-to-Home & Landing:** Automatically plans back to the takeoff pose when all frontiers are exhausted and triggers PX4 `AUTO.LAND`.

4. **Offboard Flight Bridge & Kinematics (`DroneNavigator` & `PX4TfBroadcaster`):**
   - **Coordinate Transformer:** Analytical closed-form conversion between PX4 NED/FRD and ROS 2 ENU/FLU frames.
   - **State Machine:** Deterministic lifecycle (`IDLE` $\to$ `ARMING` $\to$ `TAKING_OFF` $\to$ `NAVIGATING` $\leftrightarrow$ `HOVERING` $\to$ `LANDING`).
   - **Altitude Lock & Velocity Bridge:** Clamps Z-axis to $-1.5\text{ m}$ NED while translating ROS FLU body velocities into world NED setpoints.
   - **Hysteresis Yaw Direction Controller:** Aligns drone heading with velocity vectors ($> 45^\circ$ initiates hover-and-turn; $< 3^\circ$ deadband stabilizes cruise).

---

## Technical Stack

- **Operating System:** Ubuntu 24.04 LTS (Noble Numbat)
- **Middleware:** ROS 2 Jazzy Jalisco
- **Physics Simulator:** Gazebo Harmonic (gz-sim 8.x) with ODE engine (200 Hz update rate)
- **Flight Controller:** PX4 Autopilot SITL v1.14+
- **Communications Bridge:** Micro XRCE-DDS Agent (UDP 8888) & `ros_gz_bridge`
- **SLAM & Mapping:** RTAB-Map (ICP Point-to-Plane, g2o optimizer)
- **Navigation Stack:** Nav2 (Navfn $A^*$, MPPI Omni Controller, Costmap2D)
- **Programming Languages:** Python 3.12, C++17, CMake
- **Visualization:** RViz2 (3D Point Cloud, TF, OccupancyGrid, Costmaps, Frontier Markers)

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

- **Active Nodes:** `teleop` (`drone_teleop`), `rtabmap`, `rviz2`, `px4_tf_broadcaster`, `ros_gz_bridge`
- **Description:** Direct keyboard flight (WASD, QE, ZC) for manual inspection, debugging, and baseline mapping.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_teleop.sh
```

**Key Controls:**
- `t`: Arm drone, switch to Offboard mode, and automatic takeoff ($1.0\text{ m}$)
- `w` / `s`: Forward / Backward ($\pm 0.5\text{ m}$)
- `a` / `d`: Left / Right ($\pm 0.5\text{ m}$)
- `q` / `e`: Up / Down ($\pm 0.15\text{ m}$)
- `z` / `c`: Rotate Left / Right Yaw ($\pm 0.2\text{ rad}$)
- `Space`: Immediate Hover (hold position)
- `l`: Trigger Land procedure

### Mode 2: Autonomous Goal Navigation

- **Active Nodes:** `nav2` (`planner_server`, `controller_server`, `bt_navigator`), `autonomous` (`drone_navigator`), `rtabmap`, `rviz2`, `px4_tf_broadcaster`, `ros_gz_bridge`
- **Description:** Point-to-point 2D Goal Pose navigation using $A^*$ global planning and MPPI dynamic obstacle avoidance.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_autonomous.sh
```

1. Press **PLAY (▶)** in the Gazebo GUI.
2. The drone will automatically arm, take off, and hover at **1.5 m**.
3. In RViz2, click the **2D Goal Pose** tool and select a destination on the map.
4. The drone will compute the $A^*$ path and track it using the MPPI controller while dodging obstacles.

### Mode 3: Fully Autonomous Frontier Exploration

- **Active Nodes:** `explore` (`frontier_explorer`), `nav2`, `autonomous` (`drone_navigator`), `rtabmap`, `rviz2`, `px4_tf_broadcaster`, `ros_gz_bridge`
- **Description:** Fully autonomous frontier-driven room discovery, mapping, panoramic yaw sweeping, and return-to-home landing.

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
| **Global Path** | `/plan` | $A^*$ global trajectory (Green line) |
| **Local Plan** | `/local_plan` | MPPI dynamic rollout trajectory (Blue line) |
| **Global Costmap** | `/global_costmap/costmap` | Static obstacle inflation layer |
| **Local Costmap** | `/local_costmap/costmap` | Live 3D obstacle avoidance costmap |
| **Frontier Clusters** | `/explore/frontiers` | Unexplored boundary markers (Blue cubes) |
| **Active Target** | `/explore/target` | Selected frontier destination & centroid (Green sphere) |
| **Coordinate Frames** | `TF` | Full kinematic transform tree (`map` $\to$ `odom` $\to$ `base_link` $\to$ sensors) |

<br>

---

<a name="türkçe"></a>
# Kapalı Alan Otonom Haritalama ve Keşif Drone Sistemi

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic)
[![PX4 Autopilot](https://img.shields.io/badge/PX4-SITL%20v1.14+-red.svg)](https://px4.io/)
[![SLAM](https://img.shields.io/badge/SLAM-RTAB--Map%203D%20ICP-brightgreen.svg)](https://introlab.github.io/rtabmap/)
[![Navigation](https://img.shields.io/badge/Navigation-Nav2%20MPPI%20Omni-yellow.svg)](https://navigation.ros.org/)
[![Platform](https://img.shields.io/badge/OS-Ubuntu%2024.04%20LTS-purple.svg)](https://ubuntu.com/)

Bu proje; GPS sinyalinin bulunmadığı kapalı alanlarda **PX4 SITL**, **3D LiDAR** ve **RGB Kamera** sensör füzyonu kullanarak **Gazebo Harmonic** ortamında çalışan tam otonom bir quadrotor sistemidir. Sistem; RTAB-Map Point-to-Plane ICP algoritmasıyla yüksek doğruluklu 3D SLAM haritalama, Nav2 MPPI ile holonomik engelden kaçınma, BFS kümeleme ve akıllı sıkışma tespiti içeren otonom sınır keşfi (Frontier Exploration) ile kesintisiz NED ↔ ENU koordinat köprülemesi sağlar.

---

## Sistem Mimarisi ve Temel Bileşenler

Sistem mimarisi dört ana işlem modülünden oluşmaktadır:

1. **Uzaysal Algılama ve 3D SLAM (RTAB-Map ICP Motoru):**
   - 3D LiDAR nokta bulutu ($180 \times 45$ ışın, 25 m menzil) üzerinde Noktadan-Düzleme (Point-to-Plane) ICP eşleştirmesi.
   - İşlemci yükünü optimize eden 0.08 m voxel ızgaralama, grafik tabanlı döngü kapatma (loop closure) ve `g2o` graf optimizasyonu.
   - Tavan ve zemin yansımalarını temizlemek amacıyla 1.2 m ile 1.8 m arasında filtrelenen 2.5D OccupancyGrid harita projeksiyonu.

2. **Holonomik Yol Planlama ve Kontrol (Nav2 Stack):**
   - **Global Planlayıcı:** 2D doluluk haritası üzerinde bilinmeyen bölgelerden geçebilen `NavfnPlanner` ($A^*$) rotası.
   - **Yerel Kontrolcü:** Holonomik `Omni` hareket modeliyle 20 Hz frekansında, 2.0 saniyelik ufukta (40 adım $\times$ 0.05 s) 500 rastgele yörünge tahmini üreten Model Predictive Path Integral (`MPPIController`).
   - **7 Eleştirmen (Critic) Puanlama Jürisi:** Çarpışma engelleme (`ConstraintCritic`), duvardan uzaklık (`CostCritic`), rota sadakati (`PathAlignCritic`, `PathFollowCritic`) ve burun açısı yönlendirme (`PathAngleCritic`, `GoalAngleCritic`).

3. **Otonom Sınır Keşif Motoru (`FrontierExplorer`):**
   - Doluluk haritasında serbest ($0$) ve bilinmeyen ($-1$) alan sınırındaki sınır (frontier) hücrelerinin anlık tespiti.
   - Breadth-First Search (BFS) bağlı bileşenler algoritması ile kümeleme ve küçük gürültülerin elenmesi.
   - **3 Kademeli Mesafe Seçim Stratejisi:**
     - *Yerel Bölge ($< 3.5\text{ m}$):* Dronun odalar arasında ileri-geri savrulmasını (ping-pong etkisi) engellemek için mevcut odadaki/cepteki en küçük kümeyi temizler.
     - *Orta Bölge ($3.5 - 7.5\text{ m}$):* Mesafe karesi ağırlıklı boyut skoru ($\text{Skor} = \text{Boyut} / (\text{Mesafe}^2 + 0.1)$).
     - *Uzak Bölge ($> 7.5\text{ m}$):* Uzaktaki büyük sınır kümelerine odaklanma.
   - **Çift Kademeli Sıkışma Önleme Denetimi:**
     - *Yakınlık Sıkışması:* Hedefe 2.5 m mesafede 8 saniye boyunca 10 cm'den fazla ilerleme kaydedilemezse hedef iptal edilir ve kara listeye alınır.
     - *Fiziksel Sıkışma:* Son 12 saniyede toplam yer değiştirme 0.40 m'nin altında kalırsa fiziksel sıkışma tespit edilip hedef bırakılır.
   - **Hedef Sonrası 200° Sabit Hızlı Süpürme:** Hedefe ulaşıldığında dron yerinde 200° dönerek LiDAR ve kameranın yeni alanı haritaya katmasını sağlar.
   - **Otonom Eve Dönüş ve İniş:** Haritada keşfedilecek sınır kalmadığında başlangıç noktasına (`home_position`) geri döner ve PX4 `AUTO.LAND` modunu tetikleyerek güvenle iner.

4. **Offboard Uçuş Köprüsü ve Kinematik (`DroneNavigator` & `PX4TfBroadcaster`):**
   - **Koordinat Dönüştürücü:** PX4 NED/FRD eksenleri ile ROS 2 ENU/FLU eksenleri arasında analitik kapalı form dönüşüm.
   - **Durum Makinesi:** Belirlenimci durum yönetimi (`IDLE` $\to$ `ARMING` $\to$ `TAKING_OFF` $\to$ `NAVIGATING` $\leftrightarrow$ `HOVERING` $\to$ `LANDING`).
   - **İrtifa Kilidi ve Hız Köprüsü:** Z eksenini $-1.5\text{ m}$ NED irtifasında kilitlerken ROS FLU gövde hızlarını dünya NED hız setpoint'lerine dönüştürür.
   - **Histerezisli Yaw Yönlendirmesi:** Dronun burnunu hareket yönüne hizalar ($> 45^\circ$ sapmada durup döner; $< 3^\circ$ hata payında seyir hızına geçer).

---

## Teknolojik Altyapı

- **İşletim Sistemi:** Ubuntu 24.04 LTS (Noble Numbat)
- **Robotik Ara Katman:** ROS 2 Jazzy Jalisco
- **Fiziksel Simülatör:** Gazebo Harmonic (gz-sim 8.x) ODE motoru (200 Hz güncelleme hızı)
- **Otopilot:** PX4 Autopilot SITL v1.14+
- **Haberleşme Köprüsü:** Micro XRCE-DDS Agent (UDP 8888) ve `ros_gz_bridge`
- **Haritalama (SLAM):** RTAB-Map (ICP Point-to-Plane, g2o optimizasyon motoru)
- **Navigasyon Paketi:** Nav2 (Navfn $A^*$, MPPI Omni Controller, Costmap2D)
- **Programlama Dilleri:** Python 3.12, C++17, CMake
- **Görselleştirme:** RViz2 (3D Nokta Bulutu, TF, Doluluk Haritası, Maliyet Haritaları, Keşif Marker'ları)

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

- **Aktif Düğümler:** `teleop` (`drone_teleop`), `rtabmap`, `rviz2`, `px4_tf_broadcaster`, `ros_gz_bridge`
- **Açıklama:** Klavye ile doğrudan (WASD, QE, ZC) manuel uçuş kontrolü, test ve haritalama.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_teleop.sh
```

**Klavye Kontrol Tuşları:**
- `t`: Motorları ARM et, Offboard moda geç ve otomatik kalkış yap ($1.0\text{ m}$)
- `w` / `s`: İleri / Geri ($\pm 0.5\text{ m}$)
- `a` / `d`: Sol / Sağ ($\pm 0.5\text{ m}$)
- `q` / `e`: Yukarı / Aşağı ($\pm 0.15\text{ m}$)
- `z` / `c`: Sola Dön / Sağa Dön Yaw ($\pm 0.2\text{ rad}$)
- `Space`: Anlık Hover (olduğun yerde sabit kal)
- `l`: Güvenli iniş prosedürünü başlat

### Mod 2: Otonom Hedef Navigasyonu

- **Aktif Düğümler:** `nav2` (`planner_server`, `controller_server`, `bt_navigator`), `autonomous` (`drone_navigator`), `rtabmap`, `rviz2`, `px4_tf_broadcaster`, `ros_gz_bridge`
- **Açıklama:** RViz2 üzerinden 2D Goal Pose ile verilen hedefe $A^*$ küresel planlaması ve MPPI dinamik engelden kaçınma ile otonom uçuş.

```bash
cd ~/GPS-Denied-Indoor-Mapping-Drone
./script/start_autonomous.sh
```

1. Gazebo arayüzünde **PLAY (▶)** butonuna basın.
2. Dron otomatik olarak ARM olup kalkacak ve **1.5 m** irtifada hover moduna geçecektir.
3. RViz2 arayüzünde üst bardan **2D Goal Pose** aracını seçip haritada gitmek istediğiniz noktaya tıklayın.
4. Dron $A^*$ rotasını MPPI kontrolcüsü ile takip ederek engellerin etrafından dolaşıp hedefe varacaktır.

### Mod 3: Tam Otonom Sınır Keşfi (Frontier Exploration)

- **Aktif Düğümler:** `explore` (`frontier_explorer`), `nav2`, `autonomous` (`drone_navigator`), `rtabmap`, `rviz2`, `px4_tf_broadcaster`, `ros_gz_bridge`
- **Açıklama:** Sınır (frontier) tabanlı tam otonom oda keşfi, haritalama, panoramik süpürme ve başlangıç noktasına dönüş/iniş.

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
| **Map** | `/map` | Navigasyon ve planlama için kullanılan 2D doluluk ızgarası (OccupancyGrid) |
| **Global Path** | `/plan` | $A^*$ küresel yol planlayıcısı tarafından çizilen ana rota (Yeşil) |
| **Local Plan** | `/local_plan` | MPPI kontrolcüsünün hesapladığı anlık yerel kaçış rotası (Mavi) |
| **Global Costmap** | `/global_costmap/costmap` | Statik harita duvarları ve şişirme (inflation) katmanı |
| **Local Costmap** | `/local_costmap/costmap` | Anlık sensör verisinden beslenen 3D engel kaçınma katmanı |
| **Frontier Kümeleri** | `/explore/frontiers` | Tespit edilen keşfedilmemiş sınır noktaları (Mavi küpler) |
| **Aktif Hedef** | `/explore/target` | Seçilen hedef sınır kümesi ve ağırlık merkezi (Yeşil küre) |
| **Koordinat Eksenleri** | `TF` | Tüm sistem koordinat ağacı (`map` $\to$ `odom` $\to$ `base_link` $\to$ sensörler) |
