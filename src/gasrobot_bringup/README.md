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

## 常用参数

- `mode`：`hardware`、`slam` 或 `nav`。
- `serial_port`、`baud`：STM32 串口配置。
- `enable_lidar`、`enable_rviz`：功能开关。
- `map`：Nav2 使用的地图 YAML。
- `slam_params_file`、`nav2_params_file`：算法参数文件。
- `use_sim_time`：是否使用仿真时间。

`hardware.launch.py` 可单独启动 STM32 和雷达；一般推荐使用
`gasrobot.launch.py` 作为统一入口。
