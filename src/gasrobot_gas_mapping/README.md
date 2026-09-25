# gasrobot_gas_mapping：车间建图与地图资源

本包提供二维 SLAM 启动封装、建图参数和保存的占用栅格地图。
包名沿用工程现有命名；当前车间配送加载 `workshop_delivery_v1` 地图。

## 已有地图

- `maps/workshop_delivery_v1.yaml`：分辨率、原点、图像路径和占用阈值。
- `maps/workshop_delivery_v1.pgm`：实际 SLAM 采集的车间地图，245×206 像素，0.05 m/像素。
- [建图记录](docs/workshop_delivery_v1_mapping.md)：采集过程、地图重载验证及预览。

正式工位已在 map 坐标系标定，六工位往返与跨排导航已通过。
工位定义位于 `gasrobot_delivery/config/stations.yaml`，并与地图文件哈希绑定。
后续更换地图、原点或分辨率时，必须重新标定并验证工位。

## 使用已有地图配送

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gasrobot_bringup delivery_sim.launch.py
```

此模式由 AMCL 发布 map → odom，不同时启动 SLAM。
操作说明见 [配送包 README](../gasrobot_delivery/README.md)。

## 重新建图

在工作空间根目录执行，以下各终端均需加载 ROS 和工作空间环境。
先启动只有场景、机器人及控制器的基础入口：

```bash
ros2 launch gasrobot_simulation workshop_delivery.launch.py
```

另一个终端启动 SLAM，并显式使用仿真时钟：

```bash
ros2 launch gasrobot_gas_mapping mapping.launch.py use_sim_time:=true
```

再用键盘驾驶机器人覆盖车间通道；建图期间不要同时运行定位配送入口。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0
```

地图保存到独立输出目录，核查后再决定是否替换正式地图：

```bash
mkdir -p runs/maps
ros2 run nav2_map_server map_saver_cli -f runs/maps/workshop_candidate \
  --ros-args -p use_sim_time:=true
```

## 文件与职责

- `launch/mapping.launch.py`：调用 SLAM Toolbox 异步建图入口；use_sim_time 默认 false，仿真必须显式设为 true。
- `config/slam_toolbox.yaml`：默认建图参数，也可通过 `slam_params_file` 指定其他文件。
- `maps/`：地图资源；配送入口明确选择车间地图，不依赖目录中的其他地图。
- 差速控制器提供 odom → base_footprint；建图时由 SLAM 提供 map → odom。
- 定位与导航参数位于 `gasrobot_delivery/config/nav2_workshop.yaml`，任务逻辑位于 `gasrobot_delivery`。
