# gasrobot_navigation

GasRobot 的安全行为和 Nav2 配置包。

## 节点

`lidar_safety` 订阅激光扫描和速度指令，在前方障碍物进入停止距离时输出安全
速度指令。可单独运行：

```bash
ros2 run gasrobot_navigation lidar_safety
```

## 导航

- `config/nav2_params.yaml`：AMCL、规划器、控制器、代价地图和行为树参数。
- `launch/navigation.launch.py`：对 Nav2 Bringup 的项目级封装。

```bash
ros2 launch gasrobot_navigation navigation.launch.py \
  map:=/absolute/path/to/map.yaml
```

实验性的草地纹理识别脚本保存在 `prototypes/`，当前不会安装为 ROS 可执行
节点。稳定后应补齐话题、参数、依赖和测试再移入正式模块。
