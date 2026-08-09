#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_joint_state_publisher = LaunchConfiguration(
        "enable_joint_state_publisher"
    )

    default_model = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_description"),
            "urdf",
            "gasrobot",
            "gas_robot.urdf.xacro",
        ]
    )

    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(enable_joint_state_publisher),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Robot URDF/Xacro model file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "enable_joint_state_publisher",
                default_value="true",
            ),
            robot_state_publisher,
            joint_state_publisher,
        ]
    )
