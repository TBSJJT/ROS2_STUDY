"""通用 Gazebo 基础入口：场景、机器人、传感器和四轮差速控制器。

调用关系：delivery_sim → workshop_delivery → 本文件 → gazebo_ros/gazebo.launch.py。
本文件默认使用旧场景 L_model.world；workshop_delivery 会覆盖 world 和出生位姿。
这里只提供运动/传感器仿真，AMCL、Nav2、配送任务由 bringup 层组合。

模型经 Xacro 展开后发布到 robot_description，spawn_entity 从该话题生成机器人。
URDF 内的 gazebo_ros2_control 插件创建唯一的 controller_manager；本文件只负责
通过 spawner 加载控制器，不再创建第二个管理器，也不额外启动差速 Gazebo 插件。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """声明可覆盖参数并组装启动动作；LaunchConfiguration 在实际启动时才求值。"""
    # 通过包索引查找安装路径，支持普通安装与 symlink-install，避免写死源码目录。
    share = get_package_share_directory("gasrobot_description")
    sim_share = get_package_share_directory("gasrobot_simulation")
    gazebo_share = get_package_share_directory("gazebo_ros")
    # model 是 Xacro 文件路径，不是展开后的 XML 文本。
    model = LaunchConfiguration("model")
    # 管理器由 Gazebo 插件创建。spawner 通过退出事件串行执行，
    # 避免多个控制器同时请求 controller_manager。
    # 第一阶段发布轮轴位置/速度，robot_state_publisher 据此更新轮子 TF。
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )
    # 第二阶段激活差速控制：接收速度、计算轮速，并提供 /odom 及 odom→base_footprint。
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )
    effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        # 与差速控制器共同加载，但保持未激活，避免同时控制四个轮轴。
        arguments=[
            "gasrobot_effort_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )
    return LaunchDescription(
        [
            # Gazebo Classic 的材质、着色器必须在资源目录中查找。
            # 保留用户现有路径，同时补齐 Ubuntu Gazebo 11 的系统资源与本包模型。
            SetEnvironmentVariable(
                "GAZEBO_RESOURCE_PATH",
                os.pathsep.join(
                    filter(
                        None,
                        [
                            os.environ.get("GAZEBO_RESOURCE_PATH", ""),
                            "/usr/share/gazebo-11",
                            os.path.join(sim_share, "worlds"),
                        ],
                    )
                ),
            ),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH",
                os.pathsep.join(
                    filter(
                        None,
                        [
                            os.environ.get("GAZEBO_MODEL_PATH", ""),
                            os.path.join(sim_share, "models"),
                            "/usr/share/gazebo-11/models",
                        ],
                    )
                ),
            ),
            # 通用默认值：出生坐标属于 Gazebo world；x/y/z 单位 m，yaw 单位 rad。
            # z=0.1 为模型生成时留出落地高度，不代表稳定后的 base_footprint 高度。
            DeclareLaunchArgument(
                "model",
                default_value=os.path.join(share, "urdf", "gasrobot", "gas_robot.urdf.xacro"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="0"),
            DeclareLaunchArgument("spawn_y", default_value="0"),
            DeclareLaunchArgument("spawn_z", default_value="0.1"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.5708"),
            DeclareLaunchArgument(
                "world", default_value=os.path.join(sim_share, "worlds", "L_model.world")
            ),
            # 展开模型并发布 robot_description；simulation:=true 才启用仿真传感器/控制插件。
            # ParameterValue(..., str) 防止 Launch 把整段 URDF 当成 YAML 参数解析。
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        # 由启动层注入参数，模型包无需反向依赖仿真包。
                        "robot_description": ParameterValue(
                            Command(
                                [
                                    "xacro ",
                                    model,
                                    " simulation:=true controllers_file:=",
                                    os.path.join(
                                        sim_share, "config", "gasrobot_ros2_controller.yaml"
                                    ),
                                ]
                            ),
                            value_type=str,
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                    }
                ],
            ),
            # gazebo_ros 负责 gzserver 和可选 gzclient；gui=false 只关闭显示，不停止物理仿真。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_share, "launch", "gazebo.launch.py")
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "verbose": "true",
                }.items(),
            ),
            # spawn_entity 等待 robot_description 与 Gazebo 生成服务；这里只传入 world 出生位姿。
            # 出生位姿与导航目标的 map 坐标属于不同坐标系，不能直接混用。
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                name="spawn_gasrobot",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-entity",
                    "gasrobot",
                    "-x",
                    LaunchConfiguration("spawn_x"),
                    "-y",
                    LaunchConfiguration("spawn_y"),
                    "-z",
                    LaunchConfiguration("spawn_z"),
                    "-Y",
                    LaunchConfiguration("spawn_yaw"),
                ],
            ),
            # spawner 自带服务等待（上限 120 秒），因此可与模型生成进程同时启动。
            joint_state_spawner,
            # 退出事件保证加载请求串行化；OnProcessExit 不是“成功”断言，
            # 若前一 spawner 异常退出，仍需从其退出码/日志和控制器状态排查启动失败。
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_spawner,
                    on_exit=[diff_drive_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=diff_drive_spawner,
                    on_exit=[effort_spawner],
                )
            ),
        ]
    )
