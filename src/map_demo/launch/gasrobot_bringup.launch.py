#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    model = LaunchConfiguration("model")
    map_file = LaunchConfiguration("map")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    rviz_config = LaunchConfiguration("rviz_config")

    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_safety = LaunchConfiguration("enable_safety")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_joint_state_publisher = LaunchConfiguration(
        "enable_joint_state_publisher"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    default_model = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_description"),
            "urdf",
            "gasrobot",
            "gas_robot.urdf.xacro",
        ]
    )

    default_slam_params = PathJoinSubstitution(
        [
            FindPackageShare("map_demo"),
            "config",
            "slam_toolbox.yaml",
        ]
    )

    default_nav2_params = PathJoinSubstitution(
        [
            FindPackageShare("map_demo"),
            "config",
            "nav2_params.yaml",
        ]
    )

    default_rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("nav2_bringup"),
            "rviz",
            "nav2_default_view.rviz",
        ]
    )

    default_map = PathJoinSubstitution(
        [
            EnvironmentVariable("HOME"),
            "maps",
            "gasrobot_map.yaml",
        ]
    )

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
            "enable_joint_state_publisher": enable_joint_state_publisher,
        }.items(),
    )

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
            PythonExpression(["'", mode, "' == 'slam'"])
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

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
            PythonExpression(["'", mode, "' == 'nav'"])
        ),
        launch_arguments={
            "map": map_file,
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    delayed_slam_or_nav = TimerAction(
        period=2.0,
        actions=[
            slam_toolbox,
            navigation,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        condition=IfCondition(enable_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="slam",
                description="slam, nav, or hardware",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
            ),
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_safety",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "enable_joint_state_publisher",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
            ),
            robot_description_launch,
            hardware_launch,
            delayed_slam_or_nav,
            rviz,
        ]
    )
