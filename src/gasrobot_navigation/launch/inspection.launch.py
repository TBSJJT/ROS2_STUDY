#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动 GasRobot 工程化自主巡检任务节点的 Launch 文件。

Launch 文件是 ROS 2 的启动配置机制，相当于"一键启动脚本"。
与直接 ros2 run 相比，Launch 文件的优势：
- 可以同时启动多个节点
- 可以声明和传递参数
- 可以包含其他 launch 文件
- 可以设置节点命名空间和重映射
- 支持条件启动和动态配置

这个 launch 文件单独启动巡检任务管理器节点，
适用于已经启动了 Nav2 导航栈的场景。

使用方式：
  ros2 launch gasrobot_navigation inspection.launch.py \
    route_file:=/path/to/routes.yaml \
    auto_set_initial_pose:=true

关键概念：
- LaunchConfiguration：声明一个可配置的启动参数
- DeclareLaunchArgument：定义参数的名称、默认值和描述
- Node：描述要启动的 ROS 2 节点
- PathJoinSubstitution/FindPackageShare：在运行时查找文件路径
"""

# LaunchDescription：launch 文件的返回值，描述所有要启动的内容
from launch import LaunchDescription
# DeclareLaunchArgument：声明一个命令行可配置的参数
from launch.actions import DeclareLaunchArgument
# LaunchConfiguration：在 launch 文件中引用参数值
# PathJoinSubstitution：安全地拼接文件路径（跨平台）
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# Node：描述一个要启动的 ROS 2 节点
from launch_ros.actions import Node
# FindPackageShare：查找 ROS 2 包的共享目录（安装后的路径）
from launch_ros.substitutions import FindPackageShare


# generate_launch_description() 是 ROS 2 launch 系统的入口函数
# 每个 launch 文件必须定义这个函数，返回一个 LaunchDescription 对象
def generate_launch_description():
    """
    声明巡检配置参数并启动 inspection_manager 节点。

    返回值是一个 LaunchDescription，包含：
    1. 6 个命令行可配置的参数声明
    2. 1 个节点启动描述

    参数说明：
    - route_file：巡检路线 YAML 文件的路径
      默认值：gasrobot_navigation 包目录下的 config/inspection_routes.yaml
      这是运行巡检任务必需的配置文件

    - map：必须与 Nav2 使用同一份地图，用于启动前校验航点安全性

    - params_file：节点参数 YAML 文件
      默认值：gasrobot_navigation 包目录下的 config/inspection_manager.yaml
      可以在文件里预设 route_file、default_route 等参数值

    - default_route：默认巡检路线名称
      默认值："standard_route"
      当通过 ~/start_default 服务启动时使用的路线名

    - auto_set_initial_pose：是否自动设置 AMCL 初始位姿
      默认值："true"
      设为 true 时节点启动 2 秒后自动向 AMCL 发送 initial_pose

    - auto_start：是否自动开始执行默认路线
      默认值："false"
      设为 true 时节点启动 5 秒后自动开始巡检

    """
    # --- 创建 LaunchConfiguration 对象 ---
    # 每个 LaunchConfiguration 对应一个可配置的参数
    # 在后续的 Node 描述中通过这些对象引用参数值
    route_file = LaunchConfiguration("route_file")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    default_route = LaunchConfiguration("default_route")
    auto_set_initial_pose = LaunchConfiguration("auto_set_initial_pose")
    auto_start = LaunchConfiguration("auto_start")

    # --- 构造默认文件路径 ---
    # FindPackageShare("gasrobot_navigation")：
    #   在运行时查找 gasrobot_navigation 包的共享目录
    #   安装后的路径类似：install/gasrobot_navigation/share/gasrobot_navigation/
    #
    # PathJoinSubstitution([...])：
    #   安全地拼接路径片段，自动使用操作系统的路径分隔符
    #   例如 Linux 上用 /，Windows 上用 \

    # 默认路线文件路径：.../share/gasrobot_navigation/config/inspection_routes.yaml
    default_routes = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_navigation"),
            "config",
            "inspection_routes.yaml",
        ]
    )

    # 默认参数文件路径：.../share/gasrobot_navigation/config/inspection_manager.yaml
    default_params = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_navigation"),
            "config",
            "inspection_manager.yaml",
        ]
    )

    default_map = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_gas_mapping"),
            "maps",
            "picopc_1.yaml",
        ]
    )

    # --- 创建 Node 描述 ---
    # Node 对象描述一个要启动的 ROS 2 节点
    inspection_manager = Node(
        # package：节点所在的 ROS 2 包名
        package="gasrobot_navigation",
        # executable：可执行文件名（对应 setup.py 中 entry_points 定义的名称）
        executable="inspection_manager",
        # name：节点实例的名称（可在运行时通过 ROS 2 工具看到）
        name="inspection_manager",
        # output="screen"：节点的日志输出到终端屏幕
        output="screen",
        # parameters：传给节点的参数列表
        # 可以是 YAML 文件路径（自动加载），也可以是字典（直接传值）
        # 这里混合使用两种方式：
        # 1. params_file：从 YAML 文件加载通用参数
        # 2. {...}：直接用 LaunchConfiguration 传入命令行指定的值
        parameters=[
            params_file,           # YAML 文件中的参数
            {
                # 以下参数可以在命令行覆盖
                "route_file": route_file,
                "map_file": map_file,
                "default_route": default_route,
                "auto_set_initial_pose": auto_set_initial_pose,
                "auto_start": auto_start,
            },
        ],
    )

    # --- 返回 LaunchDescription ---
    # LaunchDescription 是一个动作列表，按顺序执行
    # 1. 先声明所有参数（DeclareLaunchArgument）
    # 2. 再启动节点（Node）
    # 注意：节点启动后才真正开始读取参数和连接串口
    return LaunchDescription(
        [
            # 声明参数：route_file
            # 不提供 default_value 时，该参数为必需参数
            DeclareLaunchArgument(
                "route_file",
                default_value=default_routes,
                description="初始化位姿和命名巡检路线 YAML 文件路径。",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="与 Nav2 一致的地图 YAML，用于巡检点安全校验。",
            ),
            # 声明参数：params_file
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="巡检任务管理节点参数 YAML 文件路径。",
            ),
            # 声明参数：default_route
            DeclareLaunchArgument(
                "default_route",
                default_value="standard_route",
                description="~/start_default 服务触发时执行的默认路线名称。",
            ),
            # 声明参数：auto_set_initial_pose
            DeclareLaunchArgument(
                "auto_set_initial_pose",
                default_value="true",
                description="启动后是否自动向 AMCL 发送固定初始位姿。",
            ),
            # 声明参数：auto_start
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="启动后是否自动执行默认巡检路线。",
            ),
            # 启动节点
            inspection_manager,
        ]
    )
