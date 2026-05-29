import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def _default_ocr_map_json_path():
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    current = launch_dir
    for _ in range(6):
        candidate = os.path.join(current, 'src', 'door_ocr', 'docs')
        if os.path.isdir(candidate):
            return os.path.join(candidate, 'door_labels.json')
        current = os.path.dirname(current)
    return os.path.expanduser('~/drone_project/src/door_ocr/docs/door_labels.json')


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
    ocr_arg = DeclareLaunchArgument(
        'ocr',
        default_value='false',
        description='Whether to launch door OCR nodes'
    )
    ocr_image_topic_arg = DeclareLaunchArgument(
        'ocr_image_topic',
        default_value='/camera',
        description='Camera image topic consumed by door OCR'
    )
    ocr_enable_semantic_mapper_arg = DeclareLaunchArgument(
        'ocr_enable_semantic_mapper',
        default_value='true',
        description='Whether OCR should persist confirmed labels and publish RViz markers'
    )
    ocr_map_json_path_arg = DeclareLaunchArgument(
        'ocr_map_json_path',
        default_value=_default_ocr_map_json_path(),
        description='JSON output path for confirmed OCR door labels'
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
    door_ocr_dir = get_package_share_directory('door_ocr')

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

    # 10. Door OCR (opsiyonel)
    door_ocr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(door_ocr_dir, 'launch', 'ocr.launch.py')
        ),
        launch_arguments={
            'image_topic': LaunchConfiguration('ocr_image_topic'),
            'enable_semantic_mapper': LaunchConfiguration('ocr_enable_semantic_mapper'),
            'map_json_path': LaunchConfiguration('ocr_map_json_path'),
            'use_sim_time': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('ocr'))
    )

    return LaunchDescription([
        autonomous_arg,
        ocr_arg,
        ocr_image_topic_arg,
        ocr_enable_semantic_mapper_arg,
        ocr_map_json_path_arg,
        micro_dds_agent,
        # 2 saniye sonra: Temel altyapı (Bridge, TF, SLAM, RViz)
        TimerAction(period=2.0, actions=[
            gz_bridge, px4_tf, static_tf_lidar, static_tf_camera,
            rtabmap_node, rviz_node
        ]),
        # 4 saniye sonra: OCR, kamera bridge ve odom yayını hazır olduktan sonra başlar
        TimerAction(period=4.0, actions=[
            door_ocr_launch
        ]),
        # 8 saniye sonra: Nav2 ve Autonomous (RTAB-Map'in /map üretmesini bekle)
        TimerAction(period=8.0, actions=[
            nav2_launch, autonomous
        ]),
    ])
