"""
Gazebo simülasyonunu başlatan launch dosyası.
Test binası world dosyasını Gazebo Harmonic ile açar.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paths
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    project_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    )

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(project_root, "worlds", "test_building.sdf"),
        description="Full path to the world SDF file",
    )

    gz_verbosity_arg = DeclareLaunchArgument(
        "gz_verbosity",
        default_value="3",
        description="Gazebo verbosity level (0-4)",
    )

    # Set GZ_SIM_RESOURCE_PATH so Gazebo can find our models
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(project_root, "models"),
    )

    # Launch Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                "-r ",  # run immediately
                LaunchConfiguration("world"),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            world_arg,
            gz_verbosity_arg,
            set_gz_resource_path,
            gz_sim,
        ]
    )
