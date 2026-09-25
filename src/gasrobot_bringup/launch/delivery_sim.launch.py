"""车间配送统一入口：Gazebo、AMCL、Nav2 和显式开始的配送任务。

OpaqueFunction 在启动参数求值后检查出生配置，避免把默认地图初始位姿用于自定义场景。
导航参数通过临时重写传递，源码中的实车参数不会被覆盖。
"""

from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml


def assemble(context):
    """求值文件路径与出生参数，组装定位、导航和业务节点。"""

    def share(name):
        return Path(get_package_share_directory(name))

    def arg(name):
        return LaunchConfiguration(name).perform(context)

    sim = share("gasrobot_simulation")
    stations = yaml.safe_load(Path(arg("stations_file")).read_text())
    depot = stations["stations"][stations["depot"]]
    default_spawn = all(
        abs(float(arg(key)) - value) < 1e-6
        for key, value in [
            ("spawn_x", 1.5),
            ("spawn_y", 1.5),
            ("spawn_z", 0.1),
            ("spawn_yaw", 0.0),
        ]
    )
    automatic = arg("auto_initial_pose").lower() == "true"
    default_map = share("gasrobot_gas_mapping") / "maps" / "workshop_delivery_v1.yaml"
    if automatic and (not default_spawn or Path(arg("map")).resolve() != default_map.resolve()):
        raise RuntimeError(
            "Custom spawn/map requires auto_initial_pose:=false and manual AMCL initialization"
        )
    rewrites = {
        "use_sim_time": "true",
        "amcl.ros__parameters.set_initial_pose": str(automatic).lower(),
    }
    for key in ["x", "y", "yaw"]:
        rewrites["amcl.ros__parameters.initial_pose." + key] = str(depot[key])
    configured = RewrittenYaml(
        source_file=arg("params_file"), root_key="", param_rewrites=rewrites, convert_types=True
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(sim / "launch/workshop_delivery.launch.py")),
            launch_arguments={
                k: arg(k) for k in ["gui", "spawn_x", "spawn_y", "spawn_z", "spawn_yaw"]
            }.items(),
        ),
        # 先让 Gazebo 建立仿真时钟及控制器，再开始导航生命周期配置。
        TimerAction(
            period=8.0,
            actions=[
                GroupAction(
                    [
                        # 恢复行为也进入平滑器，确保控制器和恢复动作共用最终速度链路。
                        SetRemap(src="behavior_server:cmd_vel", dst="/cmd_vel_nav"),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                str(share("gasrobot_navigation") / "launch/navigation.launch.py")
                            ),
                            launch_arguments={
                                "map": arg("map"),
                                "params_file": configured,
                                "use_sim_time": "true",
                                "use_composition": "False",
                            }.items(),
                        ),
                    ]
                )
            ],
        ),
        Node(
            package="gasrobot_delivery",
            executable="delivery_manager",
            name="delivery_manager",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_delivery")),
            parameters=[
                {
                    "use_sim_time": True,
                    "stations_file": arg("stations_file"),
                    "batch_file": arg("batch_file"),
                    "map_file": arg("map"),
                    "params_file": arg("params_file"),
                    "output_dir": arg("output_dir"),
                    "snapshot_files": [
                        str(sim / "worlds/workshop_delivery_v1.world"),
                        str(sim / "config/gasrobot_ros2_controller.yaml"),
                        str(
                            share("gasrobot_description") / "urdf/gasrobot/wheel/wheel.urdf.xacro"
                        ),
                        str(
                            share("gasrobot_description")
                            / "urdf/gasrobot/plugins/gasrobot_ros2_control.xacro"
                        ),
                    ],
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", str(share("gasrobot_bringup") / "config/delivery.rviz")],
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    delivery = Path(get_package_share_directory("gasrobot_delivery"))
    maps = Path(get_package_share_directory("gasrobot_gas_mapping")) / "maps"
    defaults = {
        "gui": "true",
        "rviz": "true",
        "enable_delivery": "true",
        "auto_initial_pose": "true",
        "spawn_x": "1.5",
        "spawn_y": "1.5",
        "spawn_z": "0.1",
        "spawn_yaw": "0",
        "map": str(maps / "workshop_delivery_v1.yaml"),
        "params_file": str(delivery / "config/nav2_workshop.yaml"),
        "stations_file": str(delivery / "config/stations.yaml"),
        "batch_file": str(delivery / "config/batch.yaml"),
        "output_dir": os.path.abspath("runs"),
    }
    return LaunchDescription(
        [
            *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
            OpaqueFunction(function=assemble),
        ]
    )
