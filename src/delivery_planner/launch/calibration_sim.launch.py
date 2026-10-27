"""独占仿真时间标定：包含 Gazebo/Nav2，结束或失败后自动关闭本次系统。

时间标定不启动配送管理器，避免两个 NavigateToPose 客户端竞争车辆。
配置来自距离归档；安装中的地图/模型也会在采样前校验。使用 ROS 时间。
"""

from datetime import datetime
from pathlib import Path
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def assemble(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    distance = Path(arg("distance_file")).resolve()
    arguments = [
        "--distance",
        str(distance),
        "--output",
        arg("output_dir"),
        "--repeats",
        arg("repeats"),
        "--seed",
        arg("seed"),
        "--hold-sec",
        arg("hold_sec"),
        "--timeout-sec",
        arg("timeout_sec"),
    ]
    if arg("pilot").lower() == "true":
        arguments.append("--pilot")
    collector = Node(
        package="delivery_planner",
        executable="calibrate_time_matrix.py",
        arguments=arguments,
        output="screen",
    )
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=collector,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="Time calibration exited; inspect time_matrix.json")
                    )
                ],
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory("delivery_bringup"))
                    / "launch/delivery_sim.launch.py"
                )
            ),
            launch_arguments={
                "gui": arg("gui"),
                "rviz": arg("rviz"),
                "enable_delivery": "false",
                "stations_file": str(distance.parent / "snapshots/stations.yaml"),
                "params_file": str(distance.parent / "snapshots/navigation.yaml"),
            }.items(),
        ),
        collector,
    ]


def generate_launch_description():
    defaults = {
        "output_dir": str(
            Path.cwd()
            / "runs"
            / (
                "time_matrix_"
                + datetime.now().strftime("%Y%m%dT%H%M%S")
                + "_"
                + uuid.uuid4().hex[:6]
            )
        ),
        "repeats": "5",
        "seed": "20261011",
        "hold_sec": "5.0",
        "timeout_sec": "180.0",
        "pilot": "false",
        "gui": "false",
        "rviz": "false",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("distance_file"),
            *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
            OpaqueFunction(function=assemble),
        ]
    )
