"""
Tüm simülasyonu tek seferde başlatan ana launch dosyası.
Gazebo + ROS Bridge + RViz
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            os.path.dirname(pkg_dir), "config", "drone_sim.rviz"
        ),
        description="Full path to RViz config file",
    )

    # 1. Gazebo simülasyonu
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "gazebo.launch.py")
        ),
    )

    # 2. ROS-Gazebo bridge
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "bridge.launch.py")
        ),
    )

    # 3. RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            rviz_config_arg,
            gazebo_launch,
            bridge_launch,
            rviz_node,
        ]
    )
