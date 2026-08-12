import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'map_demo'


setup(
    name=package_name,
    version='0.0.0',

    # 自动查找带有 __init__.py 的 Python 包
    packages=find_packages(
        exclude=[
            'test',
            'tests',
        ]
    ),

    data_files=[
        # 注册 ROS 2 软件包
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name,
            ]
        ),

        # 安装 package.xml
        (
            'share/' + package_name,
            [
                'package.xml',
            ]
        ),

        # 安装 launch 文件
        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob('launch/*.launch.py')
        ),
        # 安装 YAML 参数文件
        (
            os.path.join(
                'share',
                package_name,
                'config',
            ),
            glob('config/*.yaml')
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='book',
    maintainer_email='book@todo.todo',

    description=(
        'ROS2 mecanum robot STM32 serial bridge, '
        'odometry, IMU and lidar safety node'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [
            # 对应 map_demo/stm32_test.py 中的 main()
            'stm32_test = map_demo.stm32_test:main',

            # 对应 map_demo/avoid_node.py 中的 main()
            'avoid_node = map_demo.avoid_node:main',
        ],
    },
)
