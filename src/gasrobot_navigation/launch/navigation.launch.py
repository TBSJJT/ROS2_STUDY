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
"""启动已知地图定位与 Nav2 自主导航."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """声明项目级参数并复用 Nav2 官方 Bringup."""
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    respawn = LaunchConfiguration("respawn")

    # 默认参数随软件包安装，地图由顶层 Bringup 或命令行显式传入。
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_navigation"),
            "config",
            "nav2_params.yaml",
        ]
    )

    # 直接复用官方 Bringup，避免复制生命周期管理和节点装配逻辑。
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                ]
            )
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": use_composition,
            "respawn": respawn,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="用于 AMCL 定位的二维栅格地图 YAML 文件。",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Nav2 全部节点使用的参数文件。",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时钟；实车必须设为 false。",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="是否自动激活 Nav2 生命周期节点。",
            ),
            DeclareLaunchArgument(
                "use_composition",
                # Nav2 Humble 的官方 Launch 会把这个值拼入 PythonExpression。
                # 必须使用 Python 可识别的 True/False，不能写成小写 true/false，
                # 否则启动时会出现“name 'true' is not defined”。
                default_value="False",
                description="是否在组件容器中运行 Nav2 节点；默认关闭以便实车诊断。",
            ),
            DeclareLaunchArgument(
                "respawn",
                default_value="False",
                description="独立进程模式下节点退出后是否自动重启。",
            ),
            nav2_bringup,
        ]
    )
