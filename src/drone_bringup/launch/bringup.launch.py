import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ── Proje dizinini script konumundan hesapla ──────────────────────────
    # launch dosyası: <project_dir>/src/drone_bringup/launch/bringup.launch.py
    # Dolayısıyla 4 üst dizin = project_dir
    project_dir = str(Path(__file__).resolve().parents[3])

    # ── Package paylaşım dizini ──────────────────────────────────────────
    bringup_dir = get_package_share_directory('drone_bringup')

    # =====================================================================
    #  Launch Argümanları
    # =====================================================================

    # Çalışma modu: autonomous, explore, veya manual
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='autonomous',
        description='Çalışma modu: autonomous, explore, veya manual'
    )

    # RViz (varsayılan kapalı — RPi 5 üzerinde headless çalışma)
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='RViz2 başlatılsın mı (debug için)'
    )

    # ── Koşullar ─────────────────────────────────────────────────────────
    # Nav2 ve autonomous: hem autonomous hem explore modunda başlatılır
    nav2_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('mode'), "' == 'autonomous' or '",
            LaunchConfiguration('mode'), "' == 'explore'"
        ])
    )
    # Frontier explorer: sadece explore modunda
    explore_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('mode'), "' == 'explore'"
        ])
    )
    # RViz koşulu
    rviz_condition = IfCondition(LaunchConfiguration('rviz'))

    # =====================================================================
    #  1. Micro XRCE-DDS Agent — PX4 ile UART haberleşmesi (TELEM2)
    # =====================================================================
    micro_dds_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
        output='screen'
    )

    # =====================================================================
    #  2. Unitree 4D LiDAR L1 Sürücüsü (USB seri)
    # =====================================================================
    unitree_lidar = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'frame_id': 'unilidar_lidar',
            'rotate_yaw_bias': 0.0,
            'range_start': 0.1,
            'range_end': 20.0,
        }],
        output='screen'
    )

    # =====================================================================
    #  3. Pi Camera Module 3 Wide Sürücüsü (CSI, libcamera)
    # =====================================================================
    pi_camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'width': 640,
            'height': 480,
            'format': 'RGB888',
        }],
        output='screen'
    )

    # =====================================================================
    #  4. PX4 TF Broadcaster
    # =====================================================================
    px4_tf = Node(
        package='px4_offboard',
        executable='tf_broadcaster',
        name='tf_broadcaster',
        output='screen'
    )

    # =====================================================================
    #  5. Statik TF Dönüşümleri (montaj sonrası kalibre edilecek)
    # =====================================================================

    # base_link → unilidar_lidar (LiDAR drone gövdesinin 5 cm üstünde)
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_lidar',
        arguments=[
            '0.0', '0.0', '0.05',       # x, y, z  (TBD — montaj sonrası ayarlanacak)
            '0', '0', '0',              # roll, pitch, yaw
            'base_link', 'unilidar_lidar'
        ],
    )

    # base_link → camera_link (kamera ön yüzde, aşağı bakan eksen dönüşümü)
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera',
        arguments=[
            '0.10', '0.0', '0.0',       # x, y, z  (TBD — montaj sonrası ayarlanacak)
            '-1.570796', '0', '-1.570796',  # roll, pitch, yaw
            'base_link', 'camera_link'
        ],
    )

    # =====================================================================
    #  6. RTAB-Map SLAM
    # =====================================================================
    rtabmap_params_file = os.path.join(bringup_dir, 'config', 'rtabmap_params.yaml')
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[
            rtabmap_params_file,
            {
                'delete_db_on_start': True,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
            }
        ],
        remappings=[
            ('scan_cloud', '/unilidar/cloud'),
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # =====================================================================
    #  7. RViz2 (isteğe bağlı — debug için)
    # =====================================================================
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'slam_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=rviz_condition,
        output='screen'
    )

    # =====================================================================
    #  8. Nav2 Otonom Navigasyon Stack'i (autonomous ve explore modlarında)
    # =====================================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'nav2.launch.py')
        ),
        condition=nav2_condition
    )

    # =====================================================================
    #  9. Autonomous Bridge — cmd_vel → PX4 köprüsü
    #     (autonomous ve explore modlarında)
    # =====================================================================
    autonomous = Node(
        package='px4_offboard',
        executable='autonomous',
        name='autonomous',
        condition=nav2_condition,
        output='screen'
    )

    # =====================================================================
    #  10. Frontier Explorer — Otonom keşif düğümü (sadece explore modunda)
    # =====================================================================
    explore_node = Node(
        package='px4_offboard',
        executable='explore',
        name='frontier_explorer',
        condition=explore_condition,
        output='screen'
    )

    # =====================================================================
    #  11. Güvenlik İzleyicisi — her zaman çalışır
    # =====================================================================
    safety_monitor = Node(
        package='px4_offboard',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen'
    )

    # =====================================================================
    #  12. Operatör Arayüzü — web UI, her zaman çalışır
    # =====================================================================
    operator_interface = Node(
        package='px4_offboard',
        executable='operator_interface',
        name='operator_interface',
        output='screen'
    )

    # =====================================================================
    #  Kademeli Başlatma Sırası (TimerAction)
    #
    #  t=0s  : DDS agent + launch argümanları
    #  t=2s  : Sensörler (LiDAR, Kamera, TF)
    #  t=5s  : SLAM (RTAB-Map) + RViz (eğer etkinse)
    #  t=10s : Nav2 + Autonomous + Explore + Güvenlik + Operatör
    # =====================================================================

    return LaunchDescription([
        # ── Argümanlar ───────────────────────────────────────────────────
        mode_arg,
        rviz_arg,

        # ── t=0s: DDS Agent başlat ──────────────────────────────────────
        micro_dds_agent,

        # ── t=2s: Sensörler ve TF altyapısı ─────────────────────────────
        TimerAction(period=2.0, actions=[
            unitree_lidar,
            pi_camera,
            px4_tf,
            static_tf_lidar,
            static_tf_camera,
        ]),

        # ── t=5s: SLAM başlat (sensörlerin yayın yapmasını bekle) ───────
        TimerAction(period=5.0, actions=[
            rtabmap_node,
            rviz_node,
        ]),

        # ── t=10s: Navigasyon, otonom kontrol, güvenlik ve arayüz ──────
        TimerAction(period=10.0, actions=[
            nav2_launch,
            autonomous,
            explore_node,
            safety_monitor,
            operator_interface,
        ]),
    ])
