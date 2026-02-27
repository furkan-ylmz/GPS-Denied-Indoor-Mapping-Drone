"""
Gazebo <-> ROS 2 topic köprüsü.
Sensör verilerini (lidar, kamera, IMU) Gazebo'dan ROS 2'ye aktarır.
PX4 x500_lidar modeli için yapılandırılmış.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Gazebo world/model namespace
    # PX4 SITL drone'u "x500_lidar" adıyla spawn eder
    gz_world = "test_building_world"
    gz_model = "x500_lidar"

    # LiDAR ve kamera ayrı link'lerde
    lidar_prefix = f"/world/{gz_world}/model/{gz_model}/link/lidar_link/sensor"
    camera_prefix = f"/world/{gz_world}/model/{gz_model}/link/camera_link/sensor"

    # ROS-Gazebo bridge node
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            # 3D LiDAR: Gazebo -> ROS 2
            f"{lidar_prefix}/lidar_3d/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # Camera image: Gazebo -> ROS 2
            f"{camera_prefix}/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            # Camera info: Gazebo -> ROS 2
            f"{camera_prefix}/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # Clock: Gazebo -> ROS 2
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            (f"{lidar_prefix}/lidar_3d/scan/points", "/drone/lidar/points"),
            (f"{camera_prefix}/front_camera/image", "/drone/camera/image_raw"),
            (f"{camera_prefix}/front_camera/camera_info", "/drone/camera/camera_info"),
        ],
    )

    # TF static: base_link -> sensor frame'leri
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf",
        arguments=["0", "0", "0.08", "0", "0", "0", "base_link", "lidar_frame"],
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        arguments=["0.15", "0", "-0.02", "0", "0.1", "0", "base_link", "camera_frame"],
    )

    imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_frame"],
    )

    return LaunchDescription(
        [
            ros_gz_bridge,
            lidar_tf,
            camera_tf,
            imu_tf,
        ]
    )
