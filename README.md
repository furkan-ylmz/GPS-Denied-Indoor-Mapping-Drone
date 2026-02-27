# Drone Projesi - 2 Katlı Kapalı Alan Simülasyonu

## Proje Yapısı

```
drone_project/
├── Dockerfile                  # Gazebo Harmonic + ROS 2 Jazzy ortamı
├── models/
│   └── test/                   # Bina modeli
│       ├── model.config
│       ├── model.sdf
│       └── meshes/
│           └── test.obj        # Blender'dan export edilen 3D bina
├── worlds/
│   └── test_building.sdf       # Gazebo world dosyası
├── src/
│   └── drone_sim_bringup/      # ROS 2 launch paketi
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── config/
│       │   └── drone_sim.rviz  # RViz konfigürasyonu
│       └── launch/
│           ├── gazebo.launch.py     # Sadece Gazebo
│           ├── bridge.launch.py     # ROS-Gazebo köprüsü
│           └── sim_bringup.launch.py # Hepsini başlatan ana launch
└── scripts/
    ├── build_docker.sh         # Docker build
    ├── build_workspace.sh      # ROS 2 workspace build (container içi)
    ├── run_gazebo.sh           # Sadece Gazebo test (container içi)
    └── run_sim.sh              # Tam simülasyon (container içi)
```

## Kurulum & Çalıştırma

### 1. Docker İmajını Build Et (WSL terminalinde)
```bash
cd ~/drone_project
docker build -t drone_jazzy_env .
```

### 2. Container'ı Başlat (ilk seferlik)
```bash
docker run -it \
    --name drone_sim \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/drone_project:/root/drone_project \
    drone_jazzy_env
```

### 3. Container İçinde (exec ile bağlandıktan sonra)

#### ROS 2 Workspace Build
```bash
bash /root/drone_project/scripts/build_workspace.sh
```

#### Sadece Gazebo'yu Test Et
```bash
bash /root/drone_project/scripts/run_gazebo.sh
```

#### Tam Simülasyonu Başlat (Gazebo + Bridge + RViz)
```bash
bash /root/drone_project/scripts/run_sim.sh
```

## OBJ Dosyası Hakkında

- `test.obj` Blender 5.0.1'den export edilmiş
- `testroom2.mtl` dosyası eksik — bina gri renkte görünecektir
  - Blender'da tekrar export ederken MTL dosyasını da dahil edin
  - Veya DAE (COLLADA) formatında export edin (Gazebo ile daha uyumlu)
- `test.obj:Zone.Identifier` Windows artifact'i, silinebilir

## Sonraki Adımlar

1. **Drone modeli** — Takım arkadaşınızdan URDF/Xacro formatında drone modeli
2. **3D LiDAR sensör** — Drone modeline Gazebo LiDAR plugin eklemek
3. **PX4 SITL entegrasyonu** — Pixhawk 6C simülasyonu
4. **SLAM** — 3D LiDAR ile harita çıkartma (RTAB-Map veya LIO-SAM)
5. **Otonom navigasyon** — Nav2 ile path planning
6. **Kapı numarası tanıma** — Kamera + OCR (Tesseract/EasyOCR)
