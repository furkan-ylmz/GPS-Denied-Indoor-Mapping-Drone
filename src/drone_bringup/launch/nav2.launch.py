"""
Nav2 Otonom Navigasyon Launch Dosyası
Planner, Controller, BT Navigator, Behavior Server ve Lifecycle Manager'ı başlatır.
Gerçek donanım — use_sim_time kullanılmaz.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('drone_bringup')
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # --- Nav2 Node'ları ---

    # A* Global Planlayıcı
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # MPPI Yerel Kontrolcü
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # Davranış Ağacı Navigatörü
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
    )

    # Kurtarma Davranışları (Spin, Backup, Wait)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # Yaşam Döngüsü Yöneticisi — Tüm node'ları otomatik başlatır
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file],
    )

    return LaunchDescription([
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
    ])
