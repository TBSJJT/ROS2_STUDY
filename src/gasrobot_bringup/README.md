# gasrobot_bringup

GasRobot 的顶层启动包，组合机器人模型、底盘、雷达、SLAM、Nav2 和 RViz。

## 启动

仅启动硬件：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=hardware enable_rviz:=false
```

建图：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=slam
```

导航：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py mode:=nav
```

无 RViz 的自主巡检：

```bash
ros2 launch gasrobot_bringup gasrobot.launch.py \
  mode:=nav enable_inspection:=true enable_rviz:=false
```

巡检点、AMCL 初始化位姿和失败策略位于
`gasrobot_navigation/config/inspection_routes.yaml`。首次使用必须完成实地坐标标定，
并将 `site_configured` 设置为 `true`。

## 常用参数

- `mode`：`hardware`、`slam` 或 `nav`。
- `serial_port`、`baud`：STM32 串口配置。
- `enable_lidar`、`enable_rviz`：功能开关。
- `enable_inspection`：导航模式下是否启动自主巡检任务层。
- `inspection_route_file`、`default_route`：路线文件和默认路线。
- `auto_set_initial_pose`：是否使用固定停靠位初始化 AMCL。
- `auto_start_inspection`：是否在启动后自动执行默认路线。
- `map`：Nav2 使用的地图 YAML。
- `slam_params_file`、`nav2_params_file`：算法参数文件。
- `use_sim_time`：是否使用仿真时间。

`hardware.launch.py` 可单独启动 STM32 和雷达；一般推荐使用
`gasrobot.launch.py` 作为统一入口。
