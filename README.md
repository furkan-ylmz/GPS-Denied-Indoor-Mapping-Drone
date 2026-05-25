# Kapalı Alan Otonom Drone Projesi (3D SLAM & Nav2 Navigasyon)

> PX4 SITL, 3D LiDAR ve RGB Kamera sensör füzyonu kullanarak kapalı alanlarda yüksek çözünürlüklü 3 boyutlu nokta bulutu haritası çıkartan, Görsel Döngü Kapatma (Visual Loop Closure) destekli ve **Nav2 otonom navigasyon** yetenekli drone projesi.

## Genel Bakış

| Bileşen | Açıklama | Durum |
|---------|----------|-------|
| **3D LiDAR SLAM** | RTAB-Map Point-to-Plane ICP ile kapalı alan haritalama | ✅ Çalışıyor |
| **Görsel Hafıza (Loop Closure)** | RGB Kamera ile birbirine benzeyen katları ayırt etme | ✅ Çalışıyor |
| **Manuel Uçuş (Teleop)** | Klavye ile drone uçuş kontrolü | ✅ Çalışıyor |
| **Sensör Füzyonu** | 16 kanal 3D LiDAR + 120° RGB Geniş Açı Kamera | ✅ Çalışıyor |
| **Otonom Navigasyon (Nav2)** | A* + MPPI ile hedefe otonom uçuş ve engelden kaçınma | ✅ Çalışıyor |

## Sistem Mimarisi

```text
Ubuntu 24.04 (ROS 2 Jazzy)
═══════════════════════════════════════════════════════════════
  PX4 SITL ◄────► Micro XRCE-DDS Agent ◄────► ROS 2 Jazzy
  (gz_x500_lidar)        (UDP 8888)              │
      │                                          ├─ px4_tf_broadcaster (NED→ENU + TF)
      │                                          ├─ teleop.py (Klavye Kontrol)
      ▼                                          ├─ autonomous.py (Nav2 ↔ PX4 Köprüsü)
  Gazebo Harmonic ◄──► ros_gz_bridge ◄──────────►├─ RTAB-Map (ICP SLAM + Visual BoW)
      │                                          ├─ Nav2 (A* Planner + MPPI Controller)
   ┌──┴──┐                                      └─ RViz2 (Canlı Harita + Navigasyon)
   │Bina │   Sensörler:              TF Zinciri:
   │ SDF │   /lidar/points           map → odom → base_link → lidar_link
   └─────┘   /camera                 map → odom → base_link → camera_link
```

## Yazılım Ortamı

| Yazılım | Versiyon |
|---------|----------|
| İşletim Sistemi | Ubuntu 24.04 Noble |
| ROS 2 | Jazzy Jalisco |
| Simülatör | Gazebo Harmonic (gz-sim) |
| Uçuş Kontrolü | PX4 Autopilot SITL |
| PX4 ↔ ROS 2 | Micro XRCE-DDS Agent |
| SLAM | RTAB-Map (ICP, 3D LiDAR, RGB) |
| Navigasyon | Nav2 (NavFn A* + MPPI Omni) |

## Proje Yapısı

```bash
drone_project/
├── models/
│   ├── test/                            # Test binası 3D modeli (mesh)
│   └── x500_lidar/                      # PX4 drone + 3D LiDAR + RGB Kamera SDF Modeli
├── script/
│   ├── start_teleop.sh                  # Manuel teleop modu başlatıcı
│   └── start_autonomous.sh             # Otonom navigasyon modu başlatıcı
├── src/
│   ├── drone_sim_bringup/              # CMake paketi: Launch, Config, RViz
│   │   ├── config/
│   │   │   ├── rtabmap_params.yaml      # RTAB-Map SLAM parametreleri
│   │   │   └── nav2_params.yaml         # Nav2 navigasyon parametreleri
│   │   ├── launch/
│   │   │   ├── bringup.launch.py        # Ana sistem launch dosyası
│   │   │   └── nav2.launch.py           # Nav2 stack launch dosyası
│   │   └── rviz/
│   │       └── slam_view.rviz           # RViz2 arayüz konfigürasyonu
│   └── px4_offboard/                   # Python paketi: Kontrol düğümleri
│       └── px4_offboard/
│           ├── teleop.py                # Klavye (WASD) ile drone uçuş kontrolcüsü
│           ├── tf_broadcaster.py        # PX4 Odometry → TF (NED → ENU)
│           └── autonomous.py            # Nav2 cmd_vel → PX4 köprü düğümü
├── worlds/
│   └── test_building.sdf               # Gazebo test bina dünyası
└── README.md
```

## Ön Gereksinimler

| Bileşen | Kurulum Komutu |
|---------|----------------|
| ROS 2 Jazzy | `sudo apt install ros-jazzy-desktop` |
| Gazebo Bridge | `sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge` |
| RTAB-Map | `sudo apt install ros-jazzy-rtabmap-ros` |
| Nav2 | `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup` |
| PX4 & Micro-DDS | *GitHub üzerinden kaynaktan derlenmelidir.* |

## Çalıştırma

### Manuel Kontrol (Teleop) Modu
```bash
cd ~/drone_project
./script/start_teleop.sh
```

### Otonom Navigasyon Modu
```bash
cd ~/drone_project
./script/start_autonomous.sh
```

**Otonom Mod Kullanımı:**
1. Gazebo'da **PLAY (▶)** tuşuna basın
2. Dron otomatik olarak kalkacak ve **1.5m** yükseklikte hover edecek
3. RViz'de harita oluştuğunda **2D Goal Pose** aracı ile hedef verin
4. Dron A* rotasını MPPI kontrolcüsü ile takip ederek hedefe ulaşacak
5. Yolda engel varsa MPPI otomatik olarak etrafından dolanacak

*Not: Eski asılı kalmış süreçler script tarafından otomatik olarak temizlenir.*

## RViz2'de Canlı Görselleştirme

RViz2 script ile otomatik olarak açılır ve aşağıdaki katmanları içerir:

| Katman / Eklenti | Topic | Açıklama |
|------------------|-------|----------|
| **MapCloud** | `/mapData` | RTAB-Map'in oluşturduğu kalıcı 3D nokta bulutu haritası |
| **PointCloud2** | `/lidar/points` | Anlık ham LiDAR nokta bulutu |
| **Map** | `/map` | RTAB-Map 2D OccupancyGrid haritası |
| **GlobalPath** | `/plan` | A* tarafından çizilen ana rota (yeşil) |
| **LocalPath** | `/local_plan` | MPPI'nın anlık yörüngesi (mavi) |
| **GlobalCostmap** | `/global_costmap/costmap` | Duvar etrafındaki maliyet haritası |
| **LocalCostmap** | `/local_costmap/costmap` | Anlık engel maliyet haritası |
| **TF** | `TF` | Koordinat eksenleri |

## Teknik Detaylar

### Otonom Navigasyon Mimarisi (2.5D)
Nav2 algoritmaları 2D düzlemde (X, Y) rota çizerken, `autonomous.py` bu komutları PX4'e aktarır ve dronun Z eksenindeki yüksekliğini **1.5m**'de sabit tutarak 3 boyutlu uzayda güvenli uçuşunu sağlar.

| Bileşen | Plugin | Görev |
|---------|--------|-------|
| Global Planner | NavfnPlanner (A*) | RTAB-Map haritası üzerinden en kısa rota |
| Local Controller | MPPIController (Omni) | Holonomik engelden kaçınma (2000 sample) |
| Global Costmap | StaticLayer + Inflation | `/map`'den duvar bilgisi |
| Local Costmap | VoxelLayer + Inflation | `/lidar/points`'den anlık engeller |

### Sensör Füzyonu ve Loop Closure Mantığı
Sistem, ana haritalama omurgası olarak **3D Lidar** kullanır. Ancak kapalı alanlarda mimari birbirine benzediğinde Lidar'ın yanılmasını engellemek için **RGB Kamera** entegre edilmiştir. RTAB-Map, kameradan "Görsel Kelime Çantası" (Visual Bag-of-Words) oluşturarak odaları ve katları ezberler, böylece yanlış harita birleştirmelerinin (False Positive Loop Closure) önüne geçer.

### Koordinat Dönüşümleri (TF)
Havacılık standartları (PX4) ile Robotik standartları (ROS) arasındaki eksen farkı `px4_tf_broadcaster.py` tarafından çözülür:

```text
PX4 (NED)               px4_tf_broadcaster.py           ROS 2 (ENU)
──────────────    ──────────────────────────────    ─────────────────────
x = Kuzey (North)       x_enu = y_ned (East)        x = Doğu (East)
y = Doğu (East)         y_enu = x_ned (North)       y = Kuzey (North)
z = Aşağı (Down)        z_enu = -z_ned (Up)         z = Yukarı (Up)
```

## Yol Haritası

### Faz 1 — Temel Simülasyon ve Donanım Ortamı ✅
- [x] ROS 2 Jazzy + Gazebo Harmonic entegrasyonu
- [x] PX4 SITL build ve Micro XRCE-DDS Agent bağlantısı
- [x] x500 dron modeline 3D Lidar ve 120° FOV Kamera eklenmesi

### Faz 2 — Teleop ve Sensör Köprüleme ✅
- [x] `teleop.py` ile klavye üzerinden Offboard uçuş kontrolü
- [x] `ros_gz_bridge` ile `/lidar/points` ve `/camera` verilerinin ROS 2'ye aktarımı
- [x] TF ağacının (NED → ENU) kurulması ve Statik Lidar/Kamera eksenlerinin tanımlanması

### Faz 3 — Gelişmiş 3D SLAM ve Optimizasyon ✅
- [x] RTAB-Map ile 3D Lidar Point-to-Plane ICP eşleştirmesi
- [x] Harita performansını artırmak için `0.08m Voxel` ve `g2o` optimizasyon ayarları
- [x] Yanlış kat eşleşmelerini önlemek için **Visual Loop Closure** (Kamera Bag-of-Words) entegrasyonu
- [x] Tüm ayarların `config/rtabmap_params.yaml` dosyasına izole edilmesi

### Faz 4 — Otonom Navigasyon ✅
- [x] Nav2 parametre dosyası (NavfnPlanner A* + MPPI Omni Controller)
- [x] Nav2 launch dosyası (Planner, Controller, BT Navigator, Behaviors, Lifecycle)
- [x] `autonomous.py` köprü düğümü (cmd_vel → PX4 TrajectorySetpoint)
- [x] RViz2 Nav2 katmanları (Path, Costmap görselleştirme)
- [x] Otonom başlatıcı script (`start_autonomous.sh`)

### Faz 5 — Gelişmiş Özellikler (Planlanıyor) ⬜
- [ ] Otonom ortam keşfi (Frontier Exploration)
- [ ] Çoklu waypoint görevleri
- [ ] Dinamik engel takibi
