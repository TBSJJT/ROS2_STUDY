"""兼容旧命令；运行前需要构建 gasrobot_simulation。"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """包含正式入口，命令行参数继续由被包含的启动文件解析。"""
    return LaunchDescription([
        LogInfo(msg="仿真入口已迁移至 gasrobot_simulation/gazebo_sim.launch.py"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gasrobot_simulation"), "launch", "gazebo_sim.launch.py",
            ])
        )),
    ])
