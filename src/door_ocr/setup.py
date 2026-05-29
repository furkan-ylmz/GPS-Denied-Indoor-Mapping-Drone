from glob import glob
import os

from setuptools import setup


package_name = "door_ocr"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (
            os.path.join("share", package_name, "docs"),
            glob("docs/*.txt") + glob("docs/*.json") + glob("docs/.gitkeep"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ashen",
    maintainer_email="ashen@todo.com",
    description="Standalone OCR node for reading door labels from camera images.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "door_ocr_node = door_ocr.door_ocr_node:main",
            "door_semantic_mapper = door_ocr.door_semantic_mapper:main",
        ],
    },
)
