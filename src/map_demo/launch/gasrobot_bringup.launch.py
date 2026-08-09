#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)

from launch.conditions import IfCondition

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Launch 参数
    # ============================================================

    # 工作模式：
    # slam     -> 建图
    # nav      -> 导航
    # hardware -> 只启动机器人硬件
    mode = LaunchConfiguration("mode")

    # URDF/Xacro 模型
    model = LaunchConfiguration("model")

    # 导航地图 yaml 文件
    map_file = LaunchConfiguration("map")

    # slam_toolbox 参数
    slam_params_file = LaunchConfiguration("slam_params_file")

    # Nav2 参数
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    # RViz 配置
    rviz_config = LaunchConfiguration("rviz_config")

    # STM32 串口
    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")

    # 是否启动雷达
    enable_lidar = LaunchConfiguration("enable_lidar")

    # 是否启动安全避障节点
    enable_safety = LaunchConfiguration("enable_safety")

    # 是否启动 RViz
    enable_rviz = LaunchConfiguration("enable_rviz")

    # 是否启动 joint_state_publisher
    enable_joint_state_publisher = LaunchConfiguration(
        "enable_joint_state_publisher"
    )

    # 仿真时间
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Nav2 lifecycle 自动启动
    autostart = LaunchConfiguration("autostart")


    # ============================================================
    # 默认文件路径
    # ============================================================

    # 机器人 URDF/Xacro
    default_model = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_description"),
            "urdf",
            "gasrobot",
            "gas_robot.urdf.xacro",
        ]
    )

    # SLAM 参数文件
    default_slam_params = PathJoinSubstitution(
        [
            FindPackageShare("map_demo"),
            "config",
            "slam_toolbox.yaml",
        ]
    )

    # Nav2 参数文件
    default_nav2_params = PathJoinSubstitution(
        [
            FindPackageShare("map_demo"),
            "config",
            "nav2_params.yaml",
        ]
    )

    # RViz 使用 Nav2 自带配置
    default_rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bringup"),
            "rviz",
            "nav2_default_view.rviz",
        ]
    )

    # ============================================================
    # 默认导航地图
    #
    # 源码：
    #   map_ws/src/map_demo/maps/gasrobot_map.yaml
    #
    # 安装后：
    #   install/map_demo/share/map_demo/maps/gasrobot_map.yaml
    # ============================================================
    default_map = PathJoinSubstitution(
        [
            FindPackageShare("map_demo"),
            "maps",
            "gasrobot_map.yaml",
        ]
    )


    # ============================================================
    # 机器人模型
    # ============================================================

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gasrobot_description"),
                    "launch",
                    "show_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "model": model,
            "use_sim_time": use_sim_time,
            "enable_joint_state_publisher":
                enable_joint_state_publisher,
        }.items(),
    )


    # ============================================================
    # STM32 + 雷达等底层硬件
    # ============================================================

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("map_demo"),
                    "launch",
                    "map_try.launch.py",
                ]
            )
        ),
        launch_arguments={
            "serial_port": serial_port,
            "baud": baud,
            "enable_lidar": enable_lidar,
            "enable_safety": enable_safety,
        }.items(),
    )


    # ============================================================
    # SLAM
    # mode:=slam 时启动
    # ============================================================

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
        condition=IfCondition(
            PythonExpression(
                ["'", mode, "' == 'slam'"]
            )
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )


    # ============================================================
    # Nav2
    # mode:=nav 时启动
    # ============================================================

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                ]
            )
        ),
        condition=IfCondition(
            PythonExpression(
                ["'", mode, "' == 'nav'"]
            )
        ),
        launch_arguments={
            "map": map_file,
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )


    # ============================================================
    # 延迟启动 SLAM / Nav2
    #
    # 先让：
    # robot_state_publisher
    # STM32
    # 雷达
    #
    # 启动完成，再启动 SLAM 或 Nav2。
    # ============================================================

    delayed_slam_or_nav = TimerAction(
        period=2.0,
        actions=[
            slam_toolbox,
            navigation,
        ],
    )


    # ============================================================
    # RViz
    # ============================================================

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )


    # ============================================================
    # LaunchDescription
    # ============================================================

    return LaunchDescription(
        [
            # ----------------------------
            # 工作模式
            # ----------------------------
            DeclareLaunchArgument(
                "mode",
                default_value="slam",
                description="slam, nav, or hardware",
            ),

            # ----------------------------
            # 机器人模型
            # ----------------------------
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
            ),

            # ----------------------------
            # 导航地图
            # ----------------------------
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
            ),

            # ----------------------------
            # SLAM 参数
            # ----------------------------
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
            ),

            # ----------------------------
            # Nav2 参数
            # ----------------------------
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
            ),

            # ----------------------------
            # STM32 串口
            # ----------------------------
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
            ),

            DeclareLaunchArgument(
                "baud",
                default_value="115200",
            ),

            # ----------------------------
            # 雷达
            # ----------------------------
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
            ),

            # ----------------------------
            # 安全避障
            # ----------------------------
            DeclareLaunchArgument(
                "enable_safety",
                default_value="false",
            ),

            # ----------------------------
            # RViz
            # ----------------------------
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
            ),

            # ----------------------------
            # joint_state_publisher
            # ----------------------------
            DeclareLaunchArgument(
                "enable_joint_state_publisher",
                default_value="true",
            ),

            # ----------------------------
            # 时间源
            # ----------------------------
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),

            # ----------------------------
            # Nav2 lifecycle
            # ----------------------------
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
            ),

            # ----------------------------
            # RViz 配置
            # ----------------------------
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
            ),

            # ====================================================
            # 真正启动节点
            # ====================================================

            robot_description_launch,

            hardware_launch,

            delayed_slam_or_nav,

            rviz,
        ]
    )
