#!/usr/bin/env python3
"""统一启动 GasRobot 硬件、建图或已知地图导航."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """根据 mode 组合机器人描述、硬件和自主功能."""
    mode = LaunchConfiguration("mode")
    model = LaunchConfiguration("model")
    map_file = LaunchConfiguration("map")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_inspection = LaunchConfiguration("enable_inspection")
    inspection_route_file = LaunchConfiguration("inspection_route_file")
    default_route = LaunchConfiguration("default_route")
    auto_set_initial_pose = LaunchConfiguration("auto_set_initial_pose")
    auto_start_inspection = LaunchConfiguration("auto_start_inspection")
    enable_joint_state_publisher = LaunchConfiguration(
        "enable_joint_state_publisher"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    # 默认资源全部通过软件包索引定位，避免依赖当前工作目录。
    default_model = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_description"),
            "urdf",
            "gasrobot",
            "gas_robot.urdf.xacro",
        ]
    )
    default_map = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_gas_mapping"),
            "maps",
            "gasrobot_map.yaml",
        ]
    )
    default_slam_params = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_gas_mapping"),
            "config",
            "slam_toolbox.yaml",
        ]
    )
    default_nav2_params = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_navigation"),
            "config",
            "nav2_params.yaml",
        ]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("gasrobot_bringup"), "config", "gasrobot_nav_light.rviz"]
    )
    default_inspection_routes = PathJoinSubstitution(
        [
            FindPackageShare("gasrobot_navigation"),
            "config",
            "inspection_routes.yaml",
        ]
    )

    # 机器人描述和硬件在三种模式下都需要启动。
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gasrobot_description"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
        launch_arguments={
            "model": model,
            "use_sim_time": use_sim_time,
            "enable_joint_state_publisher": enable_joint_state_publisher,
        }.items(),
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gasrobot_bringup"), "launch", "hardware.launch.py"]
            )
        ),
        launch_arguments={
            "serial_port": serial_port,
            "baud": baud,
            "enable_lidar": enable_lidar,
        }.items(),
    )

    # 建图与导航互斥，由 mode 在运行时选择。
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gasrobot_gas_mapping"),
                    "launch",
                    "mapping.launch.py",
                ]
            )
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gasrobot_navigation"),
                    "launch",
                    "navigation.launch.py",
                ]
            )
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"])),
        launch_arguments={
            "map": map_file,
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    # 给串口、TF 和雷达预留初始化时间后再启动 SLAM 或 Nav2。
    delayed_autonomy = TimerAction(period=2.0, actions=[mapping, navigation])

    # 巡检任务必须等待 AMCL 和 Nav2 激活，因此在导航启动后额外延时加载。
    inspection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gasrobot_navigation"),
                    "launch",
                    "inspection.launch.py",
                ]
            )
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    mode,
                    "' == 'nav' and '",
                    enable_inspection,
                    "'.lower() == 'true'",
                ]
            )
        ),
        launch_arguments={
            "route_file": inspection_route_file,
            "default_route": default_route,
            "auto_set_initial_pose": auto_set_initial_pose,
            "auto_start": auto_start_inspection,
        }.items(),
    )
    delayed_inspection = TimerAction(period=5.0, actions=[inspection])

    # RViz 独立控制，便于无显示器部署时关闭图形界面。
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
                description="运行模式：hardware、slam 或 nav。",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="机器人 URDF/Xacro 模型文件。",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="导航模式使用的二维地图 YAML 文件。",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
                description="SLAM Toolbox 参数文件。",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
                description="Nav2 定位与导航参数文件。",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="STM32 底盘控制器串口。",
            ),
            DeclareLaunchArgument(
                "baud",
                default_value="115200",
                description="STM32 串口波特率。",
            ),
            DeclareLaunchArgument(
                "enable_lidar",
                default_value="true",
                description="是否启动激光雷达驱动。",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="是否启动 RViz 可视化界面。",
            ),
            DeclareLaunchArgument(
                "enable_inspection",
                default_value="false",
                description="导航模式下是否启动自主气体巡检任务层。",
            ),
            DeclareLaunchArgument(
                "inspection_route_file",
                default_value=default_inspection_routes,
                description="初始化位姿和命名巡检路线文件。",
            ),
            DeclareLaunchArgument(
                "default_route",
                default_value="standard_route",
                description="服务或自动启动时使用的默认巡检路线。",
            ),
            DeclareLaunchArgument(
                "auto_set_initial_pose",
                default_value="false",
                description=(
                    "巡检节点启动后是否发布固定 AMCL 初始位姿；"
                    "默认关闭，避免机器人被搬动后覆盖人工定位结果。"
                ),
            ),
            DeclareLaunchArgument(
                "auto_start_inspection",
                default_value="false",
                description="是否在系统就绪后自动执行默认巡检路线。",
            ),
            DeclareLaunchArgument(
                "enable_joint_state_publisher",
                default_value="true",
                description="是否发布非驱动关节的默认状态。",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时钟；实车必须设为 false。",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="导航模式是否自动激活生命周期节点。",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz 配置文件。",
            ),
            description,
            hardware,
            delayed_autonomy,
            delayed_inspection,
            rviz,
        ]
    )
