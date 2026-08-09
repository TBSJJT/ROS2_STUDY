import os
from glob import glob

from setuptools import find_packages, setup


package_name = "gasrobot_base"


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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="book",
    maintainer_email="2799572363@qq.com",
    description="STM32 串口桥接、里程计、IMU 与底盘控制。",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stm32_bridge = gasrobot_base.stm32_bridge:main",
        ],
    },
)
