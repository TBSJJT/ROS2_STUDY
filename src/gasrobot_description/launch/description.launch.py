#!/usr/bin/env python3
# Copyright 2026 book
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -*- coding: utf-8 -*-
"""解析机器人 Xacro 并发布 TF 结构."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """创建机器人状态和可选关节状态发布节点."""
    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_joint_state_publisher = LaunchConfiguration(
        "enable_joint_state_publisher"
    )

    # 通过软件包索引定位默认模型，安装后无需依赖源码路径。
    default_model = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_description"),
            "urdf",
            "gasrobot",
            "gas_robot.urdf.xacro",
        ]
    )

    # 在启动阶段执行 xacro，生成 robot_state_publisher 所需的 URDF 文本。
    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # 实车没有反馈的非驱动关节可由该节点提供默认状态。
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(enable_joint_state_publisher),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="机器人 URDF/Xacro 模型文件。",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时钟；实车必须设为 false。",
            ),
            DeclareLaunchArgument(
                "enable_joint_state_publisher",
                default_value="true",
                description="是否启动默认关节状态发布节点。",
            ),
            robot_state_publisher,
            joint_state_publisher,
        ]
    )
