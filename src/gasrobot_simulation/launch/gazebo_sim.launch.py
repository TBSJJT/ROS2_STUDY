"""启动 Gazebo、机器人及四轮差速 ros2_control 控制器。"""
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
    share = get_package_share_directory('gasrobot_description')
    sim_share = get_package_share_directory('gasrobot_simulation')
    gazebo_share = get_package_share_directory('gazebo_ros')
    model = LaunchConfiguration('model')
    # 管理器由 Gazebo 插件创建。spawner 通过退出事件串行执行，
    # 避免多个控制器同时请求 controller_manager。
    joint_state_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   '/controller_manager', '--controller-manager-timeout', '120'],
    )
    diff_drive_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['diff_drive_controller', '--controller-manager',
                   '/controller_manager', '--controller-manager-timeout', '120'],
    )
    effort_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        # 与差速控制器共同加载，但保持未激活，避免同时控制四个轮轴。
        arguments=['gasrobot_effort_controller', '--inactive',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
    )
    return LaunchDescription([
        # Gazebo Classic 的材质、着色器必须在资源目录中查找。
        # 保留用户现有路径，同时补齐 Ubuntu Gazebo 11 的系统资源与本包模型。
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', os.pathsep.join(filter(None, [
            os.environ.get('GAZEBO_RESOURCE_PATH', ''), '/usr/share/gazebo-11',
            os.path.join(sim_share, 'worlds'),
        ]))),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(filter(None, [
            os.environ.get('GAZEBO_MODEL_PATH', ''),
            os.path.join(sim_share, 'models'), '/usr/share/gazebo-11/models',
        ]))),
        DeclareLaunchArgument('model', default_value=os.path.join(
            share, 'urdf', 'gasrobot', 'gas_robot.urdf.xacro')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            sim_share, 'worlds', 'L_model.world')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[{
                 # 由启动层注入参数，模型包无需反向依赖仿真包。
                 'robot_description': ParameterValue(Command([
                     'xacro ', model, ' simulation:=true controllers_file:=',
                     os.path.join(sim_share, 'config', 'gasrobot_ros2_controller.yaml'),
                 ]), value_type=str),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            gazebo_share, 'launch', 'gazebo.launch.py')), launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'), 'verbose': 'true',
            }.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             name='spawn_gasrobot',
             arguments=['-topic', 'robot_description', '-entity', 'gasrobot',
                        '-x', '0', '-y', '0', '-z', '0.1', '-Y', '1.5708']),
        joint_state_spawner,
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[diff_drive_spawner],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[effort_spawner],
        )),
    ])
