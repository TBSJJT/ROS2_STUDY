# gasrobot_navigation

GasRobot 的 Nav2 配置与自主巡检任务软件包。当前版本已经实现指定地图中间直走廊的
低速之字形正常巡检，不再需要操作员在 RViz 中逐个点击目标点。

## 核心原则

巡检点只控制机器人运动路线，不控制气体传感器什么时候采样。

正常运行时，gasrobot_gas 应按固定频率持续发布带时间戳的 /gas/readings。机器人
运动、转弯或经过航点时都要采样。气体读数的位置应根据时间戳查询历史 TF，不能把
当前目标航点坐标当作实际采样位置。

路线中的 dwell_sec 是可选的静止观察时间。当前正常巡检路线全部为 0 秒，到点后
立即发送下一个导航目标。

## 当前实现

- 复用 Nav2 的定位、规划、避障和运动控制；
- AMCL 使用 `DifferentialMotionModel`，按普通差速底盘估计运动；
- DWB 和速度平滑器均禁止 `linear.y` 横移，只输出前后速度和角速度；
- 使用标准 NavigateToPose Action 逐点执行路线；
- 启动前校验路线 YAML、地图坐标系和点位安全距离；
- 点位必须位于指定地图的已知自由区域；
- 航点周围至少 0.30 米不能出现障碍物、未知区域或地图边界；
- 支持单点超时、有限重试、暂停、继续、取消和任务状态；
- 默认不自动设置 AMCL 初始位姿，也不自动启动车辆；
- 允许严重 RiskEvent 按路线策略取消当前任务。

气体基线、异常判断、回溯、局部气源搜索和恢复巡检尚未实现为运动逻辑，它们的接口
与伪代码见 docs/active_gas_inspection_architecture.md。

## 首版测试路线

配置文件：config/inspection_routes.yaml

路线只覆盖 PicoPC 实机地图 `picopc_1.yaml` 的中间直走廊：

- X 范围约为 5.40～12.37 米；
- 走廊轴线相对 map 的 X 轴倾斜约 4.31°；
- 六个纵向站位等间隔约 1.387 米，上下轨间距为 0.80 米；
- 向东 6 个点、向西 6 个点，共 12 个点；
- 一圈包含一次完整的之字形往返；
- 所有航点的 dwell_sec 均为 0；
- 巡检线速度限制为 0.15 m/s；
- 航点及相邻折线的最小静态地图净空约为 0.50 米。

这些点已通过静态栅格检查，但静态地图无法识别建图后新增的障碍物。首次实车运行
必须打开 RViz、确认 AMCL 定位和代价地图，并准备随时调用取消服务。

## 文件结构

```text
gasrobot_navigation/
├── config/
│   ├── nav2_params.yaml
│   ├── inspection_routes.yaml
│   └── inspection_manager.yaml
├── docs/
│   └── active_gas_inspection_architecture.md
├── gasrobot_navigation/
│   ├── inspection_manager.py
│   ├── map_route_validator.py
│   └── route_config.py
├── launch/
│   ├── navigation.launch.py
│   └── inspection.launch.py
└── test/
```

## 编译

以下命令在 PicoPC 的 ROS 2 工作空间中执行。不要把 Windows 的 D 盘路径传给
PicoPC；地图会通过 ROS 软件包索引自动定位。

```bash
cd /userdata/iceice/gasrobot_ws
source /opt/ros/humble/setup.bash

colcon build --packages-up-to gasrobot_bringup
source /userdata/iceice/gasrobot_ws/install/setup.bash
```

PicoPC 当前工作空间固定为 /userdata/iceice/gasrobot_ws。

## 首次实车运行

启动硬件、指定地图、Nav2、RViz 和巡检任务管理器：

```bash
cd /userdata/iceice/gasrobot_ws
source /opt/ros/humble/setup.bash
source /userdata/iceice/gasrobot_ws/install/setup.bash

ros2 launch gasrobot_bringup gasrobot.launch.py \
  mode:=nav \
  map:=/userdata/iceice/gasrobot_ws/src/gasrobot_gas_mapping/maps/picopc_1.yaml \
  enable_inspection:=true \
  enable_rviz:=true \
  auto_set_initial_pose:=false \
  auto_start_inspection:=false \
  use_sim_time:=false
```

启动后按顺序检查：

1. RViz 的 Fixed Frame 是 map；
2. 地图、LaserScan 和机器人模型重合；
3. 使用 2D Pose Estimate 设置并确认机器人真实初始位姿；
4. 局部和全局代价地图没有把机器人困在障碍物中；
5. /navigate_to_pose Action 已就绪；
6. 确认急停或取消命令可以立即使用。

检查命令：

```bash
ros2 action list | grep navigate_to_pose
ros2 lifecycle nodes
ros2 topic echo /inspection_manager/state
```

确认无误后启动一圈巡检：

```bash
ros2 service call /inspection_manager/start_default \
  std_srvs/srv/Trigger '{}'
```

## 任务控制

查看当前状态和目标航点：

```bash
ros2 topic echo /inspection_manager/state
ros2 topic echo /inspection_manager/current_waypoint
ros2 topic echo /inspection_manager/active
```

暂停：

```bash
ros2 service call /inspection_manager/pause \
  std_srvs/srv/SetBool '{data: true}'
```

继续：

```bash
ros2 service call /inspection_manager/pause \
  std_srvs/srv/SetBool '{data: false}'
```

取消：

```bash
ros2 service call /inspection_manager/cancel \
  std_srvs/srv/Trigger '{}'
```

空闲时重新加载修改后的路线：

```bash
ros2 service call /inspection_manager/reload_routes \
  std_srvs/srv/Trigger '{}'
```

## 连续采样检查

当 gasrobot_gas 实现后，先确认传感器话题持续发布：

```bash
ros2 topic hz /gas/readings
ros2 topic echo /gas/readings
```

正确结果是机器人尚未到达第一个航点时就已经持续收到数据。当前 gasrobot_gas 仍是
功能骨架，因此正常导航可以运行，但在气体驱动完成前不会自动产生真实气体数据。

## 调整路线

在 config/inspection_routes.yaml 中修改 x、y、yaw_deg 和路线执行策略。修改后重新
编译 gasrobot_navigation，再调用 reload_routes 重新加载已安装配置。

如果任何航点位于未知区、障碍物、地图外，或周围 0.30 米净空不足，任务管理节点会
拒绝启动并给出具体航点 ID。禁止为了让错误路线通过而直接关闭安全校验，应先在
RViz 中重新选择可达点。
