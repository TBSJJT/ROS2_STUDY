# gasrobot_bringup

车间配送的顶层组合启动包，使用 `delivery_sim.launch.py` 统一启动仿真、定位、导航与配送任务。

## 车间配送：编译、启动、开始

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to gasrobot_bringup --symlink-install
source install/setup.bash
ros2 launch gasrobot_bringup delivery_sim.launch.py
```

启动流程：先建立 Gazebo、机器人与控制器，8 秒后开始 AMCL/Nav2 生命周期配置，
同时提供可选 RViz 和配送节点。8 秒是启动缓冲，不代表就绪保证；配送节点还会检查
Nav2 生命周期、TF 新鲜度、停车状态和机器人是否位于物料站附近。
定位模式加载已有地图，不同时启动 SLAM。启动完成后默认等待装货确认：

```bash
# 另一个终端先 source /opt/ros/humble/setup.bash 和 install/setup.bash。
ros2 service call /delivery_manager/start std_srvs/srv/Trigger '{}'
ros2 topic echo /delivery_manager/status
```

完整路线、暂停/继续/取消、批次切换和日志说明见
[配送包 README](../gasrobot_delivery/README.md)。本轮已通过六工位往返、跨排、
异常操作、短路线及连续三批完整配送，见 [验收记录](../gasrobot_delivery/docs/acceptance.md)。

## 配送入口参数

| 参数 | 默认值 | 用途 |
|---|---|---|
| `gui` / `rviz` | `true` / `true` | 是否显示 Gazebo / RViz 窗口；无窗口验收设为 false |
| `enable_delivery` | `true` | 是否加载配送管理器；false 时可单独调试定位导航 |
| `map` | 车间地图 YAML | 地图服务器输入；工位配置必须与地图匹配 |
| `params_file` | `gasrobot_delivery/config/nav2_workshop.yaml` | 仿真专用 AMCL / Nav2 参数 |
| `stations_file` | `gasrobot_delivery/config/stations.yaml` | map 坐标工位、固定朝向、地图哈希 |
| `batch_file` | `gasrobot_delivery/config/batch.yaml` | 配送顺序及服务时间 |
| `output_dir` | 启动时当前目录下的 `runs/` | 每批独立的配置快照和运行记录 |
| `spawn_x/y/z/yaw` | `1.5 / 1.5 / 0.1 / 0` | world 中的出生位姿，单位 m / rad |
| `auto_initial_pose` | `true` | 根据默认出生点对应的标定位姿初始化 AMCL |

更改地图或出生位姿时，必须使用 `auto_initial_pose:=false` 并手动初始化定位；
更换地图还需重新标定 stations_file。不要直接把 world 出生坐标当作 map 初始位姿。

```bash
# 无窗口运行。
ros2 launch gasrobot_bringup delivery_sim.launch.py gui:=false rviz:=false
# 选择短路线；它仍然等待 start，不会自动配送。
ros2 launch gasrobot_bringup delivery_sim.launch.py \
  batch_file:="$(ros2 pkg prefix gasrobot_delivery)/share/gasrobot_delivery/config/batch_short.yaml"
```

## 代码与职责

- `delivery_sim.launch.py`：校验自动初始定位的适用条件，组合仿真/导航/配送，重映射恢复行为速度。
- `config/delivery.rviz`：以 map 为固定坐标系显示地图、激光、机器人、工位和路径。
- `gasrobot_simulation/workshop_delivery.launch.py`：选择车间 world 和出生默认值。
- `gasrobot_delivery`：任务状态机、ROS Action 适配、工位可视化及日志；不直接发布速度。
