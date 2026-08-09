# GasRobot ROS 2 工作空间

本工程是面向移动式气体泄漏巡检与风险事件定位研究的 ROS 2 Humble 工作空间。
系统通过麦克纳姆底盘、激光雷达、气体传感器和相机完成自主巡检，并利用
气体传感器响应延迟与 TF2 历史位姿修正风险事件的地图位置。

## 工程结构

```text
gasrobot_ws/
└── src/
    ├── gasrobot_interfaces   # 气体读数、风险事件与巡检任务接口
    ├── gasrobot_description  # URDF、Xacro 与 RViz 资源
    ├── gasrobot_base         # STM32 桥接、里程计与 IMU
    ├── gasrobot_gas          # 气体检测与数据处理
    ├── gasrobot_gas_mapping  # SLAM、地图与风险位置补偿
    ├── gasrobot_navigation   # AMCL、Nav2 与自主巡检导航配置
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

常用启动参数包括 `serial_port`、`baud`、`enable_lidar`、`enable_rviz`、
`use_sim_time` 和 `map`。

## 研究数据链

```text
带时间戳的气体采样
  → 查询采样时刻地图位姿
  → 气体异常检测
  → 按传感器响应延迟查询历史 TF
  → 同时保存原始位置与补偿位置
  → 关联现场压缩图像
  → 上传后端风险事件
```

## 模块边界

- 硬件通信与底盘状态管理归入 `gasrobot_base`。
- 气体检测算法归入 `gasrobot_gas`。
- SLAM、地图资源、气体地图及历史位姿补偿归入 `gasrobot_gas_mapping`。
- AMCL、Nav2 参数及巡检路线启动归入 `gasrobot_navigation`。
- 跨软件包启动编排归入 `gasrobot_bringup`。
- 第三方源码统一隔离在 `src/vendor`。
