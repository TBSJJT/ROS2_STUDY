import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'map_demo'


setup(
    name=package_name,
    version='0.0.0',

    # Python 包
    packages=find_packages(exclude=['test', 'tests']),

    # ROS 2 资源文件
    data_files=[
        # 注册软件包到 ament index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml'],
        ),

        # launch 文件
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),

        # YAML 配置文件
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),

        # 地图文件
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*'),
        ),
    ],

    # Python 依赖
    install_requires=['setuptools'],
    zip_safe=True,

    # 软件包信息
    maintainer='book',
    maintainer_email='book@todo.todo',
    description=(
        'ROS2 mecanum robot STM32 serial bridge, '
        'odometry, IMU and lidar safety node'
    ),
    license='Apache-2.0',

    # 测试依赖
    tests_require=['pytest'],

    # ROS 2 可执行节点
    entry_points={
        'console_scripts': [
            'stm32_test = map_demo.stm32_test:main',
            'avoid_node = map_demo.avoid_node:main',
        ],
    },
)
