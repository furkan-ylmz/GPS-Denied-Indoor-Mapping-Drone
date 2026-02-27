"""
Gazebo <-> ROS 2 topic köprüsü.
Sensör verilerini (lidar, kamera, IMU) Gazebo'dan ROS 2'ye aktarır.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Bridge configuration
    # Drone sensörleri için Gazebo <-> ROS 2 topic eşleştirmesi
    bridge_config = DeclareLaunchArgument(
        "bridge_config",
        default_value="",
        description="Path to bridge YAML config (optional)",
    )

    # ROS-Gazebo bridge node
    # Topic eşleştirmelerini drone modeli hazır olduğunda güncelleyeceğiz
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            # 3D LiDAR: Gazebo -> ROS 2
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # IMU: Gazebo -> ROS 2
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # Camera: Gazebo -> ROS 2
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # Clock: Gazebo -> ROS 2
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            ("/lidar/points", "/drone/lidar/points"),
            ("/imu", "/drone/imu"),
            ("/camera", "/drone/camera/image_raw"),
            ("/camera_info", "/drone/camera/camera_info"),
        ],
    )

    return LaunchDescription(
        [
            bridge_config,
            ros_gz_bridge,
        ]
    )
