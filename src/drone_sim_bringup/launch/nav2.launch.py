"""
Nav2 Path Planner Launch
━━━━━━━━━━━━━━━━━━━━━━━
Drone otonom navigasyonu için Nav2 planner_server + lifecycle_manager başlatır.
Global costmap, RTAB-Map'in /map topic'inden beslenir.

Gerekli:
  - RTAB-Map SLAM çalışıyor olmalı (/map yayınlanmalı)
  - TF: map → odom → base_link zinciri mevcut olmalı

Kullanım:
  ros2 launch drone_sim_bringup nav2.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("drone_sim_bringup")
    params_file = os.path.join(pkg_dir, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Gazebo sim time kullan")

    # ── Nav2 Planner Server ──
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── Lifecycle Manager ──
    # planner_server lifecycle node'dur, lifecycle_manager onu configure→activate eder
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_planning",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["planner_server"],
            "bond_timeout": 0.0,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        planner_server,
        lifecycle_manager,
    ])
