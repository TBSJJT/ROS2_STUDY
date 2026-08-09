import os
from glob import glob

from setuptools import find_packages, setup


package_name = "gasrobot_navigation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="book",
    maintainer_email="2799572363@qq.com",
    description="GasRobot 激光雷达安全与 Nav2 配置。",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lidar_safety = gasrobot_navigation.lidar_safety:main",
        ],
    },
)
