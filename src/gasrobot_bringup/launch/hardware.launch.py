#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_safety = LaunchConfiguration("enable_safety")
    lidar_angle = LaunchConfiguration("lidar_front_angle_deg")
    lidar_stop_distance = LaunchConfiguration("lidar_stop_distance")

    bridge_cmd_topic = PythonExpression(
        [
            "'/cmd_vel_safe' if '",
            enable_safety,
            "'.lower() in ('true', '1', 'yes') else '/cmd_vel'",
        ]
    )

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
                "cmd_vel_topic": ParameterValue(
                    bridge_cmd_topic,
                    value_type=str,
                ),
            },
        ],
    )

    lidar_safety = Node(
        package="gasrobot_navigation",
        executable="lidar_safety",
        name="lidar_safety",
        output="screen",
        condition=IfCondition(enable_safety),
        parameters=[
            {
                "front_angle_deg": ParameterValue(
                    lidar_angle,
                    value_type=float,
                ),
                "stop_distance": ParameterValue(
                    lidar_stop_distance,
                    value_type=float,
                ),
            }
        ],
    )

    lidar_launch = IncludeLaunchDescription(
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
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
            ),
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_safety",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "lidar_front_angle_deg",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "lidar_stop_distance",
                default_value="0.30",
            ),
            stm32_bridge,
            lidar_safety,
            lidar_launch,
        ]
    )
