import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    project_dir = os.path.join(home_dir, 'drone_project')
    px4_dir = os.path.join(home_dir, 'PX4-Autopilot')

    # Environment variables for Gazebo and PX4
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"{project_dir}/models:{project_dir}/worlds:{px4_dir}/Tools/simulation/gz/models:{px4_dir}/Tools/simulation/gz/worlds"
    os.environ['GZ_CONFIG_PATH'] = f"{os.environ.get('GZ_CONFIG_PATH', '')}:/usr/share/gz"
    os.environ['PX4_GZ_NO_FOLLOW'] = '1'
    os.environ['PX4_GZ_MODEL'] = 'x500_lidar'
    os.environ['PX4_GZ_MODEL_POSE'] = '10.00,-2.00,0.62,0,0,0'

    # 1. MicroXRCEAgent
    micro_dds_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # 1.5 Declare Launch Argument for autonomous mode
    autonomous_arg = DeclareLaunchArgument(
        'autonomous',
        default_value='true',
        description='Whether to launch autonomous navigation nodes (Nav2 and Drone Navigator)'
    )

    # 2. Gazebo Simulator
    # NOT: Gazebo, start_teleop.sh / start_autonomous.sh tarafından zamanlama kontrolü için
    # ayrıca başlatılır. Bu değişken burada referans olarak tanımlıdır, LaunchDescription'a dahil DEĞİLDİR.
    gazebo_sim = ExecuteProcess(  # noqa: F841
        cmd=['gz', 'sim', f"{project_dir}/worlds/test_building.sdf"],
        output='screen'
    )

    # 3. ROS-Gazebo Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 4. PX4 TF Broadcaster
    px4_tf = Node(
        package='px4_offboard',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 5. Static Transforms
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=['0.15', '0', '0', '0', '1.570796', '0', 'base_link', 'lidar_link'],
        parameters=[{'use_sim_time': True}]
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=['0.11', '0', '0', '-1.570796', '0', '-1.570796', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': True}]
    )

    # Package directories
    bringup_dir = get_package_share_directory('drone_sim_bringup')

    # 6. RTAB-Map
    rtabmap_params_file = os.path.join(bringup_dir, 'config', 'rtabmap_params.yaml')
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[
            rtabmap_params_file,
            {
                'use_sim_time': True,
                'delete_db_on_start': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom'
            }
        ],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            ('rgb/image', '/camera'),
            ('rgb/camera_info', '/camera_info')
        ],
        output='screen'
    )

    # 7. RViz2
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'slam_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. Nav2 Otonom Navigasyon Stack'i
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'nav2.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('autonomous'))
    )

    # 9. Autonomous Bridge (cmd_vel → PX4 köprüsü)
    autonomous = Node(
        package='px4_offboard',
        executable='autonomous',
        name='autonomous',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('autonomous')),
        output='screen'
    )

    return LaunchDescription([
        autonomous_arg,
        micro_dds_agent,
        # 2 saniye sonra: Temel altyapı (Bridge, TF, SLAM, RViz)
        TimerAction(period=2.0, actions=[
            gz_bridge, px4_tf, static_tf_lidar, static_tf_camera,
            rtabmap_node, rviz_node
        ]),
        # 8 saniye sonra: Nav2 ve Autonomous (RTAB-Map'in /map üretmesini bekle)
        TimerAction(period=8.0, actions=[
            nav2_launch, autonomous
        ]),
    ])
