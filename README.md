# Kapalı Alan Otonom Drone Projesi (3D SLAM & Sensör Füzyonu)

> PX4 SITL, 3D LiDAR ve RGB Kamera sensör füzyonu kullanarak kapalı alanlarda yüksek çözünürlüklü 3 boyutlu nokta bulutu haritası çıkartan, Görsel Döngü Kapatma (Visual Loop Closure) destekli drone projesi.

## Genel Bakış

| Bileşen | Açıklama | Durum |
|---------|----------|-------|
| **3D LiDAR SLAM** | RTAB-Map Point-to-Plane ICP ile kapalı alan haritalama | ✅ Çalışıyor |
| **Görsel Hafıza (Loop Closure)** | RGB Kamera ile birbirine benzeyen katları ayırt etme | ✅ Çalışıyor |
| **Manuel Uçuş (Teleop)** | Klavye ile drone uçuş kontrolü | ✅ Çalışıyor |
| **Sensör Füzyonu** | 16 kanal 3D LiDAR + 120° RGB Geniş Açı Kamera | ✅ Çalışıyor |
| **Otonom Navigasyon (Nav2)** | Hedef noktasına engellerden kaçarak gitme | ⬜ Planlanıyor |

## Sistem Mimarisi

```text
Ubuntu 24.04 (ROS 2 Jazzy)
═══════════════════════════════════════════════════════════════
  PX4 SITL ◄────► Micro XRCE-DDS Agent ◄────► ROS 2 Jazzy
  (gz_x500_lidar)        (UDP 8888)              │
      │                                          ├─ px4_tf_broadcaster (NED→ENU + TF)
      │                                          ├─ teleop.py (Klavye Kontrol)
      ▼                                          ├─ RTAB-Map (ICP SLAM + Visual BoW)
  Gazebo Harmonic ◄──► ros_gz_bridge ◄──────────►├─ RViz2 (Canlı Harita Görselleştirme)
      │                                          
   ┌──┴──┐
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

## Proje Yapısı

```bash
drone_project/
├── config/
│   └── rtabmap_params.yaml          # RTAB-Map ileri düzey ICP, Voxel ve Loop Closure ayarları
├── models/
│   └── x500_lidar/                  # PX4 drone + 3D LiDAR (360°) + 120° RGB Kamera SDF Modeli
├── rviz/
│   └── teleop.rviz                  # Hazır RViz2 arayüz konfigürasyonu
├── script/
│   └── start_teleop.sh              # Tüm simülasyonu, köprüleri ve SLAM'i başlatan bash scripti
├── src/
│   ├── px4_tf_broadcaster.py        # PX4 Odometry koordinat çeviricisi (NED → ENU)
│   └── teleop.py                    # Klavye (WASD) ile dron uçuş kontrolcüsü
├── worlds/
│   └── test_building.sdf            # Haritalama testi için 2 katlı kapalı bina simülasyonu
└── README.md
```

## Ön Gereksinimler

| Bileşen | Kurulum Komutu |
|---------|----------------|
| ROS 2 Jazzy | `sudo apt install ros-jazzy-desktop` |
| Gazebo Bridge | `sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge` |
| RTAB-Map | `sudo apt install ros-jazzy-rtabmap-ros` |
| PX4 & Micro-DDS | *GitHub üzerinden kaynaktan derlenmelidir.* |

## Çalıştırma

Tüm sistemi (Simülasyon, SLAM, TF, RViz ve Teleop) senkronize bir şekilde tek komutla başlatmak için:

```bash
cd ~/drone_project
./script/start_teleop.sh
```
*Not: Eski asılı kalmış süreçler script tarafından otomatik olarak temizlenir.*

## RViz2'de Canlı Görselleştirme

RViz2 script ile otomatik olarak açılır ve aşağıdaki katmanları içerir:

| Katman / Eklenti | Topic | Açıklama |
|------------------|-------|----------|
| **MapCloud** | `/mapData` | RTAB-Map'in oluşturduğu, hafızada tutulan kalıcı 3 boyutlu nokta bulutu haritası. |
| **PointCloud2** | `/lidar/points` | Dronun anlık olarak Lidar'dan okuduğu ham nokta bulutu (Kırmızı renkli). |
| **TF** | `TF` | `map`, `odom`, `base_link`, `lidar_link` ve `camera_link` eksen koordinatları. |

**İpuçları:**
* `MapCloud` ayarlarında uzaktaki duvarları görmek için **Cloud max depth (m)** değerini `0` yapabilirsiniz.
* Haritayı yüksekliğe göre (kat kat) renklendirmek için **Color Transformer** ayarını `AxisColor` ve ekseni `Z` olarak seçin.

## Teknik Detaylar

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

### Faz 4 — Otonom Navigasyon (Planlanıyor) ⬜
- [ ] Nav2 planner ve costmap entegrasyonu
- [ ] RViz2 üzerinden 3D haritada hedefe otonom gidiş
- [ ] Otonom ortam keşfi (Frontier Exploration)
