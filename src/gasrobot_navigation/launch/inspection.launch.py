#!/usr/bin/env python3
"""启动 GasRobot 工程化自主巡检任务节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """声明巡检配置参数并启动任务管理节点。"""

    route_file = LaunchConfiguration("route_file")
    params_file = LaunchConfiguration("params_file")
    default_route = LaunchConfiguration("default_route")
    auto_set_initial_pose = LaunchConfiguration("auto_set_initial_pose")
    auto_start = LaunchConfiguration("auto_start")

    default_routes = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_navigation"),
            "config",
            "inspection_routes.yaml",
        ]
    )
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_navigation"),
            "config",
            "inspection_manager.yaml",
        ]
    )

    inspection_manager = Node(
        package="gasrobot_navigation",
        executable="inspection_manager",
        name="inspection_manager",
        output="screen",
        parameters=[
            params_file,
            {
                "route_file": route_file,
                "default_route": default_route,
                "auto_set_initial_pose": auto_set_initial_pose,
                "auto_start": auto_start,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "route_file",
                default_value=default_routes,
                description="初始化位姿和命名巡检路线 YAML 文件。",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="巡检任务管理节点参数文件。",
            ),
            DeclareLaunchArgument(
                "default_route",
                default_value="standard_route",
                description="服务启动时执行的默认命名路线。",
            ),
            DeclareLaunchArgument(
                "auto_set_initial_pose",
                default_value="true",
                description="启动后是否向 AMCL 发送固定初始位姿。",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="启动后是否自动执行默认巡检路线。",
            ),
            inspection_manager,
        ]
    )
