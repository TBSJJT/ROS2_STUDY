"""一键生成静态规划距离矩阵，无需启动 Gazebo，也不会移动机器人。

专用 TF 仅给全局栅格提供固定原点，不是机器人位姿估计；规划目标总是
显式指定工位起点。工具在私有命名空间中运行，采集结束自动关闭节点。
"""

from datetime import datetime
from pathlib import Path
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from delivery_planner.snapshot import prepare


def assemble(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    output = Path(arg("output_dir")).resolve()
    runtime, stations, book, _ = prepare(
        output, arg("map"), arg("stations_file"), arg("params_file")
    )
    depot = book["stations"][book["depot"]]
    # 与运行中的车辆 TF 分离，即使有真实机器人也不会改变其坐标关系。
    remaps = [("/tf", "/cost_matrix/tf"), ("/tf_static", "/cost_matrix/tf_static")]
    builder = Node(
        package="delivery_planner",
        executable="cost_matrix_builder.py",
        output="screen",
        arguments=[
            "--output",
            str(output),
            "--stations",
            str(stations),
            "--planner-id",
            arg("planner_id"),
        ],
    )
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            namespace="cost_matrix",
            name="planning_origin",
            output="screen",
            arguments=[
                "--x",
                str(depot["x"]),
                "--y",
                str(depot["y"]),
                "--z",
                "0",
                "--yaw",
                "0",
                "--pitch",
                "0",
                "--roll",
                "0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "cost_matrix_origin",
            ],
            remappings=remaps,
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            namespace="cost_matrix",
            parameters=[str(runtime)],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace="cost_matrix",
            parameters=[str(runtime)],
            remappings=remaps,
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager",
            namespace="cost_matrix",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "use_sim_time": False,
                    "node_names": ["map_server", "planner_server"],
                }
            ],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=builder,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="Matrix collection finished; inspect matrix.json")
                    )
                ],
            )
        ),
        builder,
    ]


def generate_launch_description():
    tasks = Path(get_package_share_directory("delivery_tasks"))
    maps = Path(get_package_share_directory("delivery_mapping")) / "maps"
    defaults = {
        "output_dir": str(
            Path.cwd()
            / "runs"
            / (
                "cost_matrix_"
                + datetime.now().strftime("%Y%m%dT%H%M%S")
                + "_"
                + uuid.uuid4().hex[:6]
            )
        ),
        "map": str(maps / "workshop_delivery_v1.yaml"),
        "stations_file": str(tasks / "config/stations.yaml"),
        "params_file": str(tasks / "config/nav2_workshop.yaml"),
        "planner_id": "GridBased",
    }
    return LaunchDescription(
        [
            *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
            OpaqueFunction(function=assemble),
        ]
    )
