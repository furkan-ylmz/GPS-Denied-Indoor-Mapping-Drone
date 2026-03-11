"""
RTAB-Map 3D LiDAR SLAM Launch
──────────────────────────────
PX4 drone ile 3D LiDAR (16 kanal) kullanarak kapalı alan SLAM.

TF ağacı:
  map → odom → base_link → lidar_link
                          → camera_link

Gerekli topic'ler (bridge.launch.py tarafından sağlanır):
  /drone/lidar/points  → sensor_msgs/PointCloud2 (3D LiDAR)
  /drone/odom          → nav_msgs/Odometry (PX4 odometry → odom_publisher)

Kullanım:
  # 1) PX4 SITL çalışıyor olmalı
  # 2) DDS agent çalışıyor olmalı
  # 3) Bridge çalışıyor olmalı
  ros2 launch drone_sim_bringup slam.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ──
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Gazebo sim time kullan")

    localization_arg = DeclareLaunchArgument(
        "localization", default_value="false",
        description="true=sadece lokalizasyon (mevcut haritayla), false=SLAM (haritalama)")

    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")

    # ══════════════════════════════════════════════
    #  PX4 Odometry → ROS 2 Odometry + TF
    # ══════════════════════════════════════════════
    odom_node = Node(
        package="px4_offboard",
        executable="odom_publisher",
        name="odom_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ══════════════════════════════════════════════
    #  RTAB-Map SLAM (3D LiDAR ICP modu)
    # ══════════════════════════════════════════════
    # RTAB-Map parametreleri:
    #   - Reg/Strategy=1: ICP (point cloud registration)
    #   - ICP/VoxelSize: Downsampling (kapalı alan için 0.1m yeterli)
    #   - ICP/MaxCorrespondenceDistance: ICP eşleştirme mesafesi
    #   - Grid/FromDepth=false: LiDAR'dan 3D grid oluştur
    #   - RGBD/ProximityBySpace=true: Yakın node'lar arasında loop closure
    rtabmap_slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,

            # ── Genel ──
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_scan_cloud": True,
            "approx_sync": True,
            "queue_size": 10,

            # ── RTAB-Map parametreleri ──
            # Registration: ICP (LiDAR tabanlı)
            "Reg/Strategy": "1",
            "Reg/Force3DoF": "false",

            # ICP ayarları
            "ICP/VoxelSize": "0.1",
            "ICP/MaxCorrespondenceDistance": "1.5",
            "ICP/PointToPlane": "true",
            "ICP/PointToPlaneK": "20",
            "ICP/Iterations": "30",
            "ICP/Epsilon": "0.001",
            "ICP/MaxTranslation": "2.0",

            # Graph SLAM
            "RGBD/ProximityBySpace": "true",
            "RGBD/ProximityMaxGraphDepth": "0",
            "RGBD/ProximityPathMaxNeighbors": "10",
            "RGBD/AngularUpdate": "0.05",
            "RGBD/LinearUpdate": "0.05",
            "RGBD/OptimizeFromGraphEnd": "false",
            "RGBD/NeighborLinkRefining": "true",

            # Bellek yönetimi (WSL sınırlı RAM)
            "Mem/NotLinkedNodesKept": "false",
            "Mem/STMSize": "30",

            # 3D Grid / OccupancyGrid (iç mekan optimizasyonu)
            "Grid/FromDepth": "false",
            "Grid/RangeMax": "20.0",
            "Grid/RangeMin": "0.5",
            "Grid/CellSize": "0.1",
            "Grid/ClusterRadius": "0.5",
            "Grid/3D": "true",
            "Grid/MaxGroundHeight": "0.3",
            "Grid/MaxObstacleHeight": "2.0",
            "Grid/NormalsSegmentation": "true",
            "Grid/NoiseFilteringRadius": "0.5",
            "Grid/NoiseFilteringMinNeighbors": "5",

            # Optimizer
            "Optimizer/Strategy": "1",  # g2o
            "Optimizer/GravitySigma": "0.3",
        }],
        remappings=[
            ("scan_cloud", "/drone/lidar/points"),
            ("odom", "/drone/odom"),
            ("map", "/map"),
        ],
        arguments=["--delete_db_on_start"],
    )

    # ══════════════════════════════════════════════
    #  RTAB-Map Visualisation (point cloud map)
    # ══════════════════════════════════════════════
    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "subscribe_scan_cloud": True,
            "subscribe_odom_info": True,
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "approx_sync": True,
            "queue_size": 10,
        }],
        remappings=[
            ("scan_cloud", "/drone/lidar/points"),
            ("odom", "/drone/odom"),
        ],
        condition=None,  # Her zaman çalışsın
    )

    # ══════════════════════════════════════════════
    #  Map → OccupancyGrid 2D (Nav2 uyumlu)
    # ══════════════════════════════════════════════
    map_assembler = Node(
        package="rtabmap_util",
        executable="map_assembler",
        name="map_assembler",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "Grid/FromDepth": "false",
            "Grid/RangeMax": "20.0",
            "Grid/CellSize": "0.1",
        }],
        remappings=[
            ("map", "/map"),
            ("mapData", "/rtabmap/mapData"),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        localization_arg,
        odom_node,
        rtabmap_slam,
        # rtabmap_viz,  # WSL'de GUI yok, rviz2 kullan
        # map_assembler,  # İhtiyaç duyulduğunda aktifleştir
    ])
