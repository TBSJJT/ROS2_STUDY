# GasRobot ROS 2 工作空间

本工程是面向 GasRobot 移动式气体检测平台的 ROS 2 Humble 工作空间。

## 工程结构

```text
gasrobot_ws/
└── src/
    ├── gasrobot_interfaces   # 项目自有 ROS 接口
    ├── gasrobot_description  # URDF、Xacro 与 RViz 资源
    ├── gasrobot_base         # STM32 桥接、里程计与 IMU
    ├── gasrobot_gas          # 气体检测与数据处理
    ├── gasrobot_gas_mapping  # SLAM 配置、地图与气体建图
    ├── gasrobot_navigation   # 激光雷达安全与 Nav2 配置
    ├── gasrobot_bringup      # 顶层启动编排
    └── vendor                # 第三方 ROS 软件包
```

`vendor` 目录当前保存 LS 激光雷达驱动及其配套接口。项目自有软件包不得修改或复用第三方软件包名称。

## 构建

```bash
cd /home/book/ros2_study/gasrobot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 运行

仅启动硬件：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=hardware enable_rviz:=false
```

启动 SLAM：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=slam
```

使用默认已安装地图启动导航：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=nav
```

常用启动参数包括 `serial_port`、`baud`、`enable_lidar`、`enable_safety`、
`enable_rviz`、`use_sim_time` 和 `map`。

## 模块边界

- 硬件通信与底盘状态管理归入 `gasrobot_base`。
- 气体检测算法归入 `gasrobot_gas`。
- SLAM、地图资源及气体地图集成归入 `gasrobot_gas_mapping`。
- 导航与防碰撞安全逻辑归入 `gasrobot_navigation`。
- 跨软件包启动编排归入 `gasrobot_bringup`。
- 第三方源码统一隔离在 `src/vendor`。
