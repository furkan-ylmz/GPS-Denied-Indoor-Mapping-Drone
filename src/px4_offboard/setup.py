from setuptools import setup
import os
from glob import glob

package_name = "px4_offboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="furkan",
    maintainer_email="furkan@todo.com",
    description="PX4 offboard control for autonomous drone navigation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "offboard_control = px4_offboard.offboard_control:main",
            "drone_teleop = px4_offboard.drone_teleop:main",
            "odom_publisher = px4_offboard.odom_publisher:main",
            "drone_navigator = px4_offboard.drone_navigator:main",
            "frontier_explorer = px4_offboard.frontier_explorer:main",
            "map_cleaner = px4_offboard.map_cleaner:main",
        ],
    },
)
