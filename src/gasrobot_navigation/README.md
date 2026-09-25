# gasrobot_navigation：车间定位与导航

本包提供 `launch/navigation.launch.py`，封装 Nav2 官方 bringup，启动地图服务器、
AMCL、规划器、控制器、恢复行为和速度平滑器。配送工位、任务顺序及卸货状态机属于
[gasrobot_delivery](../gasrobot_delivery/README.md)。

## 推荐入口

从工作空间根目录编译并启动完整系统：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to gasrobot_bringup --symlink-install
source install/setup.bash
ros2 launch gasrobot_bringup delivery_sim.launch.py
```

该入口组合 Gazebo、地图、定位、导航及配送节点，默认不自动发送配送目标。
开始与任务控制见 [配送操作说明](../gasrobot_delivery/README.md#开始与控制)。
不要在同一仿真中重复启动另一套 AMCL/Nav2。

## 当前实际使用的配置

| 配置 | 文件 | 作用 |
|---|---|---|
| 地图 | `gasrobot_gas_mapping/maps/workshop_delivery_v1.yaml` | 已保存的车间占用栅格 |
| 导航参数 | `gasrobot_delivery/config/nav2_workshop.yaml` | AMCL、Navfn、RPP、代价地图及速度平滑器 |
| 工位 | `gasrobot_delivery/config/stations.yaml` | map 坐标中的位置、固定朝向及地图哈希 |
| 批次 | `gasrobot_delivery/config/batch.yaml` | 访问顺序、服务时间、超时和重试 |

`navigation.launch.py` 的通用默认参数是本包 `config/nav2_params.yaml`，
配送入口会显式传入上述 `nav2_workshop.yaml`。调整车间配送时应修改实际使用的配置。

## 导航启动参数

| 参数 | 含义 |
|---|---|
| `map` | 必填，地图 YAML 路径 |
| `params_file` | Nav2 参数文件；配送入口指定车间专用配置 |
| `use_sim_time` | 仿真中为 true，与 Gazebo `/clock` 一致 |
| `autostart` | 默认 true，自动激活 Nav2 生命周期节点 |
| `use_composition` | 默认 `False`，节点使用独立进程 |
| `respawn` | 默认 `False`，不自动重启退出的节点 |

仅调试导航时，可在完整入口添加 `enable_delivery:=false`，保留定位导航而不加载配送管理器。

## 数据与控制链路

```text
Gazebo /scan → AMCL、代价地图
差速控制器 /odom → Nav2、配送节点
配送节点 → NavigateToPose → Nav2
Nav2 控制器/恢复行为 → /cmd_vel_nav → velocity_smoother → /cmd_vel → 差速控制器
```

TF 为 `map → odom → base_footprint`，前一段由 AMCL 发布，后一段由差速控制器发布。
定位模式不同时启动 SLAM。`odom` 原点与运动中的机器人分离是正常现象，应结合
激光与地图是否匹配、TF 是否连续来判断定位异常。

默认线速度上限 0.15 m/s，角速度上限 0.4 rad/s；代价地图膨胀半径 0.55 m。
内部目标容差为 0.07 m / 0.07 rad，配送节点会在停车后再次检查 0.10 m / 0.10 rad 的 map 位姿容差。

## 检查与验收

在已加载 ROS 和工作空间环境的另一个终端执行：

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 run tf2_ros tf2_echo map base_footprint
```

先确认节点处于 active，定位正常且机器人停车，再开始配送。六工位往返、跨排、
短路线及连续三批完整配送的结果见 [验收记录](../gasrobot_delivery/docs/acceptance.md)。
该记录也说明了 map 估计位姿容差与实际定位精度的区别。
