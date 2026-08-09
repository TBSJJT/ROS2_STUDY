#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动底盘串口桥接和激光雷达硬件驱动."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """创建仅包含实车硬件驱动的启动描述."""
    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    enable_lidar = LaunchConfiguration("enable_lidar")

    # 复用底盘包安装的完整参数，只允许顶层覆盖与机器相关的串口参数。
    bridge_config = os.path.join(
        get_package_share_directory("gasrobot_base"),
        "config",
        "stm32_bridge.yaml",
    )
    stm32_bridge = Node(
        package="gasrobot_base",
        executable="stm32_bridge",
        name="stm32_bridge",
        output="screen",
        parameters=[
            bridge_config,
            {
                "port": ParameterValue(serial_port, value_type=str),
                "baud": ParameterValue(baud, value_type=int),
            },
        ],
    )

    # 雷达驱动保留在 vendor 中，此处只负责按需包含其上游 Launch。
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lslidar_driver"),
                "launch",
                "lsn10_launch.py",
            )
        ),
        condition=IfCondition(enable_lidar),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="STM32 底盘控制器对应的串口设备。",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="STM32 串口通信波特率。",
            ),
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
                description="是否启动 LS 系列激光雷达驱动。",
            ),
            stm32_bridge,
            lidar_driver,
        ]
    )
