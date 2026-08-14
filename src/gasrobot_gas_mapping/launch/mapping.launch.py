#!/usr/bin/env python3
"""启动用于实车二维建图的异步 SLAM Toolbox."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """将项目参数传递给 SLAM Toolbox 官方 Launch."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # 默认参数随软件包安装，也允许实验时从命令行替换。
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_gas_mapping"),
            "config",
            "slam_toolbox.yaml",
        ]
    )

    # 使用异步模式减少激光处理阻塞底盘和气体采样节点的概率。
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时钟；实车必须设为 false。",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_params,
                description="SLAM Toolbox 参数文件。",
            ),
            slam_toolbox,
        ]
    )
