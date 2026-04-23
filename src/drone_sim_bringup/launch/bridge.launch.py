"""
Gazebo <-> ROS 2 topic köprüsü.
Sensör verilerini (lidar, kamera) Gazebo'dan ROS 2'ye aktarır.
PX4 x500_lidar modeli için yapılandırılmış.

Kullanım:
  ros2 launch drone_sim_bringup bridge.launch.py
  ros2 launch drone_sim_bringup bridge.launch.py gz_world:=default gz_model:=x500_lidar
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ──
    world_arg = DeclareLaunchArgument(
        "gz_world", default_value="default",
        description="Gazebo world adı (PX4 SITL tarafından kullanılan)")
    model_arg = DeclareLaunchArgument(
        "gz_model", default_value="x500_lidar",
        description="Gazebo model adı")

    # Sensör topic'leri model'deki <topic> tag'larından gelir:
    #   lidar_3d sensörü → /lidar
    #   front_camera sensörü → /camera
    # Bu topic'ler model-scoped olarak:
    #   /model/{model_name}/lidar  ve  /model/{model_name}/camera
    # NOT: PX4 SITL'de model adı airframe parametresine bağlı.
    # Gazebo Harmonic'te gpu_lidar scan/points topic'i:
    #   /world/{world}/model/{model}/link/{link}/sensor/{sensor}/scan/points

    # ── ROS-Gazebo bridge ──
    # Gazebo sensör topic'leri (model SDF'deki <topic> tag'larından):
    #   /lidar/points → PointCloudPacked (3D LiDAR)
    #   /camera       → Image (RGB kamera)
    #   /camera_info  → CameraInfo (kamera parametreleri)
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            # 3D LiDAR point cloud: Gazebo → ROS 2
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # Camera image: Gazebo → ROS 2
            "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
            # Camera info: Gazebo → ROS 2
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # Clock: Gazebo → ROS 2
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            ("/lidar/points", "/drone/lidar/points"),
            ("/camera", "/drone/camera/image_raw"),
            ("/camera_info", "/drone/camera/camera_info"),
        ],
    )

    # ── TF static: base_link → sensor frame'leri ──
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf",
        arguments=[
            "--x", "0.15", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "1.570796", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "lidar_link",
        ],
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        arguments=[
            "--x", "0.1", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        ros_gz_bridge,
        lidar_tf,
        camera_tf,
    ])
