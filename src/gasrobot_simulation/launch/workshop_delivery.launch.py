"""车间场景的轻量入口：选择 world，并覆盖通用仿真的出生参数。

用于检查场景、键盘驾驶或搭配独立 SLAM 入口建图。完整定位配送请启动
 gasrobot_bringup/delivery_sim.launch.py，由其统一组合 AMCL、Nav2 与配送节点。
此处的出生位姿属于 Gazebo world；配送工位的 map 坐标保存在 stations.yaml。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """复用基础仿真入口，仅声明车间需要覆盖的场景和出生参数。"""
    share = get_package_share_directory("gasrobot_simulation")
    # 物料站地面停靠点为 world (1.5, 1.5)，yaw=0 面向 +X。
    # 高度 0.1 m 用于生成后自然落地；默认值可通过命令行 spawn_*:=... 覆盖。
    # 改动出生点不会自动修改 AMCL 初始位姿，完整配送入口会另行校验这一约束。
    defaults = {
        "gui": "true",
        "spawn_x": "1.5",
        "spawn_y": "1.5",
        "spawn_z": "0.1",
        "spawn_yaw": "0",
    }
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            # 模型与控制器逻辑统一在 gazebo_sim 中维护，避免两个入口参数逐渐不一致。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "gazebo_sim.launch.py")
                ),
                launch_arguments={
                    "world": os.path.join(share, "worlds", "workshop_delivery_v1.world"),
                    # 车间物理仿真固定使用 /clock；定位/建图节点也必须使用同一时钟。
                    "use_sim_time": "true",
                    **{name: LaunchConfiguration(name) for name in defaults},
                }.items(),
            ),
        ]
    )
