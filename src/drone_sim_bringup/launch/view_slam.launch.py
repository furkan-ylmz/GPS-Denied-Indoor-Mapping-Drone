"""
RViz2 SLAM Görselleştirme Launch
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Drone SLAM verisini canlı izlemek için RViz2 başlatır.

Gösterir:
  - 2D Occupancy Grid Map (/map)
  - 3D Point Cloud Map (/cloud_map)
  - Canlı LiDAR verisi (/drone/lidar/points)
  - Odometri yolu (/drone/odom)
  - Kamera görüntüsü (/drone/camera/image_raw)
  - TF ağacı (map → odom → base_link → lidar_link/camera_link)

Kullanım:
  ros2 launch drone_sim_bringup view_slam.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("drone_sim_bringup")
    default_rviz = os.path.join(pkg_dir, "config", "slam_view.rviz")

    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="RViz2 config dosya yolu",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        rviz_arg,
        rviz_node,
    ])
