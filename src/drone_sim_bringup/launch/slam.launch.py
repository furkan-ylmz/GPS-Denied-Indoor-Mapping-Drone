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
from launch.conditions import IfCondition, UnlessCondition
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

    resume_arg = DeclareLaunchArgument(
        "resume", default_value="false",
        description="true=önceki rtabmap.db üzerinden haritaya devam et, false=sıfırdan başla ve db'yi sil")

    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    resume = LaunchConfiguration("resume")

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
    rtabmap_parameters = {
        "use_sim_time": use_sim_time,
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "subscribe_scan_cloud": True,
        "approx_sync": True,
        "queue_size": 10,
        "Reg/Strategy": "1",
        "Reg/Force3DoF": "false",
        # ── ICP parametreleri (harita kaymasını azaltmak için optimize edildi) ──
        "ICP/VoxelSize": "0.08",                   # 0.1 → 0.08: daha hassas scan eşleşme
        "ICP/MaxCorrespondenceDistance": "1.0",      # 1.5 → 1.0: daha sıkı eşleşme, yanlış match azalır
        "ICP/PointToPlane": "true",
        "ICP/PointToPlaneK": "20",
        "ICP/Iterations": "50",                     # 30 → 50: daha fazla iterasyon, daha iyi yakınsama
        "ICP/Epsilon": "0.001",
        "ICP/MaxTranslation": "2.0",
        "RGBD/ProximityBySpace": "true",
        "RGBD/ProximityMaxGraphDepth": "0",
        "RGBD/ProximityPathMaxNeighbors": "10",
        # ── Daha sık güncelleme (küçük hareketlerde bile SLAM çalışsın) ──
        "RGBD/AngularUpdate": "0.02",               # 0.05 → 0.02: daha sık angular update
        "RGBD/LinearUpdate": "0.02",                 # 0.05 → 0.02: daha sık linear update
        "RGBD/OptimizeFromGraphEnd": "false",
        "RGBD/NeighborLinkRefining": "true",
        "Mem/NotLinkedNodesKept": "false",
        "Mem/STMSize": "50",                        # 30 → 50: daha fazla node hafızada
        "Grid/FromDepth": "false",
        "Grid/RayTracing": "true",
        "Grid/RangeMax": "25.0",
        "Grid/RangeMin": "0.3",
        "Grid/CellSize": "0.1",
        "Grid/ClusterRadius": "0.3",
        "Grid/3D": "true",
        "Grid/MaxGroundHeight": "0.0",
        "Grid/MaxObstacleHeight": "0.0",
        "Grid/NormalsSegmentation": "true",
        "Grid/NoiseFilteringRadius": "0.3",
        "Grid/NoiseFilteringMinNeighbors": "5",
        "Optimizer/Strategy": "1",
        "Optimizer/GravitySigma": "0.3",
    }

    rtabmap_remappings = [
        ("scan_cloud", "/drone/lidar/points"),
        ("odom", "/drone/odom"),
        ("map", "/rtabmap/map"),
    ]

    rtabmap_slam_new = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings,
        arguments=["--delete_db_on_start"],
        condition=UnlessCondition(resume)
    )

    rtabmap_slam_resume = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_parameters],
        remappings=rtabmap_remappings,
        condition=IfCondition(resume)
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

    return LaunchDescription([
        resume_arg,
        use_sim_time_arg,
        localization_arg,
        odom_node,
        rtabmap_slam_new,
        rtabmap_slam_resume,
        # rtabmap_viz,  # WSL'de GUI yok, rviz2 kullan
        # map_assembler ve map_cleaner kaldırıldı — 3D nokta bulutu kullanılıyor
    ])

