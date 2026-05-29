"""Launch the standalone door OCR node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_map_json_path():
    package_root = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir)
    )
    docs_dir = os.path.join(package_root, "docs")
    if os.path.isdir(docs_dir):
        return os.path.join(docs_dir, "door_labels.json")
    pkg_share = get_package_share_directory("door_ocr")
    return os.path.join(pkg_share, "docs", "door_labels.json")


def generate_launch_description():
    pkg_share = get_package_share_directory("door_ocr")
    default_params = os.path.join(pkg_share, "config", "ocr_params.yaml")

    params_arg = DeclareLaunchArgument(
        "ocr_params",
        default_value=default_params,
        description="Path to OCR params yaml",
    )

    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera",
        description="OCR input image topic",
    )

    enable_semantic_mapper_arg = DeclareLaunchArgument(
        "enable_semantic_mapper",
        default_value="true",
        description="Publish confirmed OCR labels as RViz odom markers and JSON",
    )

    map_json_path_arg = DeclareLaunchArgument(
        "map_json_path",
        default_value=_default_map_json_path(),
        description="Optional semantic door label JSON path override",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock for OCR nodes",
    )

    ocr_node = Node(
        package="door_ocr",
        executable="door_ocr_node",
        name="door_ocr_node",
        output="screen",
        parameters=[
            LaunchConfiguration("ocr_params"),
            {"image_topic": LaunchConfiguration("image_topic")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    semantic_mapper_node = Node(
        package="door_ocr",
        executable="door_semantic_mapper",
        name="door_semantic_mapper",
        output="screen",
        parameters=[
            LaunchConfiguration("ocr_params"),
            {"map_json_path": LaunchConfiguration("map_json_path")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("enable_semantic_mapper")),
    )

    return LaunchDescription([
        params_arg,
        image_topic_arg,
        enable_semantic_mapper_arg,
        map_json_path_arg,
        use_sim_time_arg,
        ocr_node,
        semantic_mapper_node,
    ])
